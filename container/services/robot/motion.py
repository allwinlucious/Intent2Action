"""Planning requests for robot_service (phase_3.md decision 21): MotionPlannerService on `world`.

The robot base frame is the world root (the Franka stands at the origin, sim/scenes/default.yaml), and
poses are given relative to object `franka`. Tool frames: TCP = `franka_hand`/`tcp`, FLANGE =
`franka`/`flange`.
"""

import logging
import math

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
from robot_service import robot_service_pb2 as api

from robot.ops import ApiError

WORLD = "world"
ROBOT = "franka"
TOOL_FRAMES = {api.TOOL_FRAME_UNSPECIFIED: ("franka_hand", "tcp"), api.TCP: ("franka_hand", "tcp"),
               api.FLANGE: ("franka", "flange")}
DOF = 7
PLAN_TIMEOUT_S = 60.0
RPC_TIMEOUT_S = 10.0
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
        reached = _linear_reached(error)
        segment_ = reached and request.motion_specification.motion_segments[reached[0]]
        if not segment_ or not segment_.target.HasField("cartesian_pose"):
          raise ApiError(grpc.StatusCode.ABORTED, f"planning failed: {error.details()}") from error
        log.info("LINEAR segment %d: replanning to the reached joints %s", reached[0],
                 [round(q, 4) for q in reached[1]])
        segment_.target.CopyFrom(_joint_target(reached[1]))
    if not response.discretized.state:
      raise ApiError(grpc.StatusCode.ABORTED, "planning failed: the planner returned an empty trajectory")
    return response

  def _call(self, method: str, request):
    try:
      return getattr(self.stub, method)(request, timeout=RPC_TIMEOUT_S)
    except grpc.RpcError as error:
      code = error.code() if error.code() == grpc.StatusCode.UNAVAILABLE else grpc.StatusCode.ABORTED
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

  def check_collisions(self, values) -> tuple[bool, str]:
    response = self._call(
        "CheckCollisions",
        planner_pb.CheckCollisionsRequest(
            world_id=WORLD, robot_reference=_robot(), waypoint=[joint_space_pb2.JointVec(joints=joints(values))]
        ),
    )
    return response.has_collision, response.collision_debug_msg
