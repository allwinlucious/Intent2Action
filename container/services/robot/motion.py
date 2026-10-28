"""Planning requests for robot_service (phase_3.md decision 21): MotionPlannerService on `world`.

The robot base frame is the world root (the Franka stands at the origin, sim/scenes/default.yaml), and
poses are given relative to object `franka`. Tool frames: TCP = `franka_hand`/`tcp`, FLANGE =
`franka`/`flange`.
"""

import logging
import math
from pathlib import Path

from google.protobuf import duration_pb2
from google.rpc import status_pb2
import grpc
from intrinsic.icon.proto import joint_space_pb2
from intrinsic.math.proto import point_pb2
from intrinsic.math.proto import pose_pb2
from intrinsic.math.proto import quaternion_pb2
from intrinsic.motion_planning.proto.v1 import geometric_constraints_pb2 as gc_pb
from intrinsic.motion_planning.proto.v1 import motion_blending_parameter_pb2 as blend_pb
from intrinsic.motion_planning.proto.v1 import motion_planner_service_pb2 as planner_pb
from intrinsic.motion_planning.proto.v1 import motion_planner_service_pb2_grpc as planner_grpc
from intrinsic.motion_planning.proto.v1 import motion_planning_error_pb2 as error_pb
from intrinsic.motion_planning.proto.v1 import motion_specification_pb2 as spec_pb
from intrinsic.motion_planning.proto.v1 import robot_specification_pb2 as robot_pb
from intrinsic.world.proto import object_world_refs_pb2 as refs_pb
import numpy as np
from robot_model import franka_scene_object as fso
from robot_service import robot_service_pb2 as api

from robot.ops import ApiError

WORLD = "world"
ROBOT = "franka"
TOOL_FRAMES = {api.TOOL_FRAME_UNSPECIFIED: ("franka_hand", "tcp"), api.TCP: ("franka_hand", "tcp"),
               api.FLANGE: ("franka", "flange")}
DOF = 7
PLAN_TIMEOUT_S = 60.0
RPC_TIMEOUT_S = 10.0
URDF_PATH = Path("/opt/app/assets/robots/franka/lula_franka_gen.urdf")
SWEPT_EVERY = 10  # PlanOnly's swept volume uses every 10th discretized point (and the last)
LINK_RADIUS_M = 0.08  # the swept bounds of the link origins are enlarged by this
# A LINEAR replan to the reached joints is accepted only if they reach the requested pose this closely.
LINEAR_REACHED_TOLERANCE = (1e-3, 0.5)  # m, deg
KEPT_PLANNER_CODES = (grpc.StatusCode.INVALID_ARGUMENT, grpc.StatusCode.NOT_FOUND)
log = logging.getLogger("robot_service")


def joints(values, what: str = "joints") -> list[float]:
  values = list(values)
  if len(values) != DOF or not all(math.isfinite(v) for v in values):
    raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"{what} must be {DOF} finite positions (rad), got {values}")
  return values


def to_pose_proto(pose: api.Pose) -> pose_pb2.Pose:
  values = (pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw)
  norm = math.sqrt(pose.qx ** 2 + pose.qy ** 2 + pose.qz ** 2 + pose.qw ** 2)
  if not all(math.isfinite(v) for v in values) or norm < 1e-9:
    raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"invalid pose (need finite values and a non-zero quaternion): {values}")
  return pose_pb2.Pose(
      position=point_pb2.Point(x=pose.x, y=pose.y, z=pose.z),
      orientation=quaternion_pb2.Quaternion(x=pose.qx / norm, y=pose.qy / norm, z=pose.qz / norm, w=pose.qw / norm),
  )


def from_pose_proto(pose: pose_pb2.Pose) -> api.Pose:
  p, q = pose.position, pose.orientation
  return api.Pose(x=p.x, y=p.y, z=p.z, qx=q.x, qy=q.y, qz=q.z, qw=q.w)


def duration_s(trajectory: joint_space_pb2.JointTrajectoryPVA) -> float:
  last: duration_pb2.Duration = trajectory.time_since_start[-1]
  return last.seconds + last.nanos * 1e-9


def _robot() -> robot_pb.RobotReference:
  return robot_pb.RobotReference(object_id=refs_pb.ObjectReference(by_name=refs_pb.ObjectReferenceByName(object_name=ROBOT)))


def _object_node(name: str) -> refs_pb.TransformNodeReference:
  return refs_pb.TransformNodeReference(by_name=refs_pb.TransformNodeReferenceByName(object=refs_pb.ObjectReferenceByName(object_name=name)))


def _tool_node(tool: int) -> refs_pb.TransformNodeReference:
  if tool not in TOOL_FRAMES:
    raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, f"unknown tool frame {tool} (use TCP or FLANGE)")
  object_name, frame = TOOL_FRAMES[tool]
  return refs_pb.TransformNodeReference(
      by_name=refs_pb.TransformNodeReferenceByName(
          frame=refs_pb.FrameReferenceByName(object_name=object_name, frame_name=frame)
      )
  )


def _pose_target(pose: api.Pose, tool: int) -> gc_pb.GeometricConstraint:
  return gc_pb.GeometricConstraint(
      cartesian_pose=gc_pb.PoseEquality(
          moving_frame=_tool_node(tool), target_frame=_object_node(ROBOT), target_frame_offset=to_pose_proto(pose)
      )
  )


def _joint_target(values) -> gc_pb.GeometricConstraint:
  return gc_pb.GeometricConstraint(joint_position=joint_space_pb2.JointVec(joints=joints(values, "target joints")))


def segment(waypoint: api.Waypoint, tool: int) -> spec_pb.MotionSegment:
  if waypoint.WhichOneof("target") == "pose":
    target = _pose_target(waypoint.pose, tool)
  elif waypoint.WhichOneof("target") == "joints":
    target = _joint_target(waypoint.joints.positions)
  else:
    raise ApiError(grpc.StatusCode.INVALID_ARGUMENT, "waypoint has neither pose nor joints")
  motion = spec_pb.MotionSegment.LINEAR if waypoint.motion == api.LINEAR else spec_pb.MotionSegment.ANY
  return spec_pb.MotionSegment(target=target, motion_type=motion)


def _linear_reached(error: grpc.RpcError) -> tuple[int, list[float]] | None:
  """(segment index, reached joints) of a linear Cartesian path error in the status details."""
  for key, value in error.trailing_metadata() or ():
    if key != "grpc-status-details-bin":
      continue
    for detail in status_pb2.Status.FromString(value).details:
      if "linear_cartesian_path_planning_error" not in detail.type_url:
        continue
      for found in error_pb.MotionPipelineError.FromString(detail.value).motion_planning_error:
        reached = list(found.linear_cartesian_path_planning_error.final_joint_positions.joints)
        if len(reached) == DOF:
          # segment_id is the failing range (first, last); the reached joints end the last one.
          return (found.segment_id[-1] if found.segment_id else 0), reached
  return None


def pose_matrix(pose: api.Pose) -> np.ndarray:
  w, x, y, z = np.array([pose.qw, pose.qx, pose.qy, pose.qz]) / math.sqrt(
      pose.qw ** 2 + pose.qx ** 2 + pose.qy ** 2 + pose.qz ** 2)
  m = np.eye(4)
  m[:3, :3] = [
      [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ]
  m[:3, 3] = [pose.x, pose.y, pose.z]
  return m


def pose_error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
  """Position (m) and rotation (deg) between two 4x4 transforms."""
  cos = (np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2
  return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def _rotation(axis: int, angle: float) -> np.ndarray:
  c, s = math.cos(angle), math.sin(angle)
  return np.array([[[1, 0, 0], [0, c, -s], [0, s, c]],
                   [[c, 0, s], [0, 1, 0], [-s, 0, c]],
                   [[c, -s, 0], [s, c, 0], [0, 0, 1]]][axis])


def _origin_matrix(origin: fso.Origin) -> np.ndarray:
  r, p, y = origin.rpy
  m = np.eye(4)
  m[:3, :3] = _rotation(2, y) @ _rotation(1, p) @ _rotation(0, r)
  m[:3, 3] = origin.xyz
  return m


class ArmChain:
  """Link origins of `franka` (panda_link1 … panda_link8) from the URDF the world model is built from.

  The planner's ComputeFk only reaches objects and frames, and `franka` has frames only at the flange.
  """

  def __init__(self, urdf_path: Path = URDF_PATH):
    urdf = fso.parse_urdf(urdf_path)
    by_child = {j.child: j for j in urdf.joints.values()}  # the URDF has side branches; walk back from link8
    self._joints, link = [], fso.ARM_LAST_LINK
    while link != fso.ARM_ROOT_LINK:
      joint = by_child[link]
      revolute = joint.type == "revolute"
      assert not revolute or tuple(joint.axis) == (0.0, 0.0, 1.0), f"{joint.name} does not turn about z"
      self._joints.insert(0, (_origin_matrix(joint.origin), revolute))
      link = joint.parent

  def link_origins(self, q) -> list[np.ndarray]:
    transform, origins, index = np.eye(4), [], 0
    for origin, revolute in self._joints:
      transform = transform @ origin
      if revolute:
        transform[:3, :3] = transform[:3, :3] @ _rotation(2, q[index])
        index += 1
      origins.append(transform[:3, 3].copy())
    return origins


def blending(waypoints) -> blend_pb.BlendingParameters | None:
  """The planner takes one blending setting per request: the largest blend_radius, Cartesian only."""
  radius = max((w.blend_radius for w in waypoints), default=0.0)
  if radius <= 0:
    return None
  if any(w.motion != api.LINEAR for w in waypoints):
    raise ApiError(
        grpc.StatusCode.INVALID_ARGUMENT,
        "blend_radius (m) needs every waypoint to be LINEAR (the planner blends Cartesian corners only)",
    )
  return blend_pb.BlendingParameters(
      cartesian_blending=blend_pb.CartesianBlendingParameters(translation_corner_rounding=radius)
  )


class Planner:

  def __init__(self, planner_address: str):
    self.stub = planner_grpc.MotionPlannerServiceStub(grpc.insecure_channel(planner_address))

  def plan(self, start, segments, curve=None) -> planner_pb.TrajectoryPlanningResponse:
    """PlanTrajectory from `start`; ABORTED with the planner's reason if no trajectory is found.

    A LINEAR pose segment can fail because the planner's IK target and the end of its straight-line
    path are different solutions of the redundant arm (a few mrad apart); as the planner advises,
    that segment is replanned with the reached joints as its target (same pose, still LINEAR).
    """
    request = planner_pb.MotionPlanningRequest(
        world_id=WORLD,
        robot_specification=robot_pb.RobotSpecification(
            robot_reference=_robot(), start_configuration=joint_space_pb2.JointVec(joints=start)
        ),
        motion_specification=spec_pb.MotionSpecification(motion_segments=segments),
        caller_id="robot_service",
    )
    if curve is not None:
      request.motion_specification.curve_parameters.CopyFrom(curve)
    for _ in range(len(segments) + 1):
      try:
        response = self.stub.PlanTrajectory(request, timeout=PLAN_TIMEOUT_S)
        break
      except grpc.RpcError as error:
        if error.code() in (grpc.StatusCode.UNAVAILABLE, grpc.StatusCode.DEADLINE_EXCEEDED):
          raise ApiError(error.code(), f"motion planner: {error.details()}") from error
        reached = _linear_reached(error)  # arrives as NOT_FOUND
        segment_ = reached and request.motion_specification.motion_segments[reached[0]]
        if not segment_ or not segment_.target.HasField("cartesian_pose"):
          code = error.code() if error.code() in KEPT_PLANNER_CODES and not reached else grpc.StatusCode.ABORTED
          raise ApiError(code, f"planning failed: {error.details()}") from error
        # The reached joints may be partway along a blocked line: accept them only at the requested pose.
        target = segment_.target.cartesian_pose
        reached_pose = self._call("ComputeFk", planner_pb.FkRequest(
            world_id=WORLD, joints=joint_space_pb2.JointVec(joints=reached[1]), robot_reference=_robot(),
            reference=_object_node(ROBOT), target=target.moving_frame)).reference_t_target
        position, angle = pose_error(pose_matrix(from_pose_proto(reached_pose)),
                                     pose_matrix(from_pose_proto(target.target_frame_offset)))
        if position > LINEAR_REACHED_TOLERANCE[0] or angle > LINEAR_REACHED_TOLERANCE[1]:
          raise ApiError(grpc.StatusCode.ABORTED, f"planning failed: {error.details()}") from error
        log.info("LINEAR segment %d: replanning to the reached joints %s (%.2f mm / %.3f deg from the pose)",
                 reached[0], [round(q, 4) for q in reached[1]], position * 1e3, angle)
        segment_.target.CopyFrom(_joint_target(reached[1]))
    if not response.discretized.state:
      raise ApiError(grpc.StatusCode.ABORTED, "planning failed: the planner returned an empty trajectory")
    return response

  def _call(self, method: str, request):
    try:
      return getattr(self.stub, method)(request, timeout=RPC_TIMEOUT_S)
    except grpc.RpcError as error:
      kept = (grpc.StatusCode.UNAVAILABLE, *KEPT_PLANNER_CODES)
      code = error.code() if error.code() in kept else grpc.StatusCode.ABORTED
      raise ApiError(code, f"motion planner {method}: {error.details()}") from error

  def ik(self, pose: api.Pose, tool: int, seed) -> list[float]:
    response = self._call(
        "ComputeIk",
        planner_pb.IkRequest(
            world_id=WORLD,
            target=_pose_target(pose, tool),
            starting_joints=joint_space_pb2.JointVec(joints=seed),
            robot_reference=_robot(),
            max_num_solutions=1,
            disable_error_on_collisions=True,
        ),
    )
    if not response.solutions:
      raise ApiError(grpc.StatusCode.ABORTED, "no IK solution for the pose")
    return list(response.solutions[0].joints)

  def fk(self, values, tool: int) -> api.Pose:
    response = self._call(
        "ComputeFk",
        planner_pb.FkRequest(
            world_id=WORLD,
            joints=joint_space_pb2.JointVec(joints=joints(values)),
            robot_reference=_robot(),
            reference=_object_node(ROBOT),
            target=_tool_node(tool),
        ),
    )
    return from_pose_proto(response.reference_t_target)

  def swept_bounds(self, trajectory: joint_space_pb2.JointTrajectoryPVA, chain: ArmChain) -> tuple[api.Pose, api.Pose]:
    """Approximate axis-aligned swept volume: TCP (ComputeFk) and arm link origins on every 10th point."""
    last = len(trajectory.state) - 1
    points = []
    for i in sorted({*range(0, last, SWEPT_EVERY), last}):
      q = list(trajectory.state[i].position)
      tcp = self.fk(q, api.TCP)
      points.append((tcp.x, tcp.y, tcp.z))
      points.extend(chain.link_origins(q))
    low = np.min(points, axis=0) - LINK_RADIUS_M
    high = np.max(points, axis=0) + LINK_RADIUS_M
    return (api.Pose(x=low[0], y=low[1], z=low[2], qw=1.0), api.Pose(x=high[0], y=high[1], z=high[2], qw=1.0))

  def check_collisions(self, values) -> tuple[bool, str]:
    response = self._call(
        "CheckCollisions",
        planner_pb.CheckCollisionsRequest(
            world_id=WORLD, robot_reference=_robot(), waypoint=[joint_space_pb2.JointVec(joints=joints(values))]
        ),
    )
    return response.has_collision, response.collision_debug_msg
