"""PlanGrasp (phase_5c.md decision 6): Intrinsic's MoveIt grasp pipeline over its ROS 2 action.

robot_service applies `world`'s catalog objects to MoveIt's planning scene (/apply_planning_scene, in
`base_link`), asks
/grasp_planning/plan_grasps for box grasps of the target, keeps those the 0.075 m gripper can close on and
whose grasp and pregrasp joints are collision-free in Intrinsic's world (Addendum B.2), moves the grasp 2 cm deeper so the finger pads overlap the object, and returns the best one in the base frame
(= world; the Franka stands at the origin).

The helpers below need no ROS; GraspPlanner imports rclpy and the MoveIt messages when it is created.
"""

from collections.abc import Callable
import dataclasses
import logging
from pathlib import Path
import threading
import time

from google.protobuf import wrappers_pb2
import grpc
import numpy as np
from robot_service import robot_service_pb2 as api
import yaml

from robot import unknown_object
from robot.motion import pose_matrix
from robot.ops import ApiError

CATALOG_DIR = Path("/opt/app/assets/objects")
CATALOG_KEY = "catalog"  # world user_data written by scene_sync (decision 5)
GRIPPER_OPENING_M = 0.075  # ICON's release width
WIDTH_TOLERANCE_M = 1e-9  # rotation round-off; an extent of exactly the opening is kept
MAX_DEPTH_M = 0.02
DEPTH_MARGIN_M = 0.01
TOO_WIDE = f"object wider than the gripper opening ({GRIPPER_OPENING_M} m) in every horizontal direction"
NOT_COLLISION_FREE = "no grasp is collision-free in the robot's world model"
ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
PLANNING_FRAME = "base_link"  # MoveIt's model frame; the Franka base is at the world origin (Addendum A.1)
# Addendum A.2/B.1: static tops this close to z = 0 are lowered by this much in MoveIt. The base link mesh dips
# 0.03 mm below z = 0; MoveIt accepts poses up to this far inside the table.
BASE_PLANE_MARGIN_M = 0.0005
SCENE_TIMEOUT_S = 5.0
SERVER_TIMEOUT_S = 10.0
RESULT_TIMEOUT_S = 30.0
NOT_RUNNING = "grasp planner not running"
# The goal (decision 6.4): every candidate is returned (max_num_grasps) so the width filter sees them all.
GOAL = dict(group_name="panda_arm", end_effector_group="hand", tool_frame="right_gripper", planning_timeout_sec=10.0,
            gripper_motion_duration_sec=0.5, retract_dist_m=0.08, surfaces=[0, 1, 2, 3, 4], num_rotations=4,
            max_num_grasps=20)
log = logging.getLogger("robot_service")


@dataclasses.dataclass
class Grasp:
  """One planner grasp; poses are 4x4 transforms in the object frame (its origin is the bottom centre)."""
  quality: float
  object_t_tool: np.ndarray
  object_t_pregrasp: np.ndarray
  joints: dict[str, float]
  pregrasp_joints: dict[str, float]


def _catalog_entry(catalog: str, catalog_dir: Path) -> tuple[Path, dict]:
  path = catalog_dir / catalog / "object.yaml"
  if not path.is_file():
    raise ApiError(grpc.StatusCode.NOT_FOUND, f"no catalog entry {catalog!r} ({path})")
  return path, yaml.safe_load(path.read_text())


def catalog_entry(catalog: str, catalog_dir: Path = CATALOG_DIR) -> dict:
  """The catalog's object.yaml."""
  return _catalog_entry(catalog, catalog_dir)[1]


def catalog_dims(catalog: str, catalog_dir: Path = CATALOG_DIR) -> np.ndarray:
  """Bounding box [x, y, z] (m) from the catalog's object.yaml."""
  path, entry = _catalog_entry(catalog, catalog_dir)
  if entry.get("dims") is None:
    raise ApiError(grpc.StatusCode.FAILED_PRECONDITION, f"{path} has no dims")
  return np.array(entry["dims"], dtype=float)


def catalog_static(catalog: str, catalog_dir: Path = CATALOG_DIR) -> bool:
  return bool(_catalog_entry(catalog, catalog_dir)[1].get("static"))


def lowered_m(static: bool, bottom_z: float, dims: np.ndarray) -> float:
  """Addendum A.2: how far the MoveIt box of an upright object is lowered so the robot base does not touch it."""
  return BASE_PLANE_MARGIN_M if static and abs(bottom_z + dims[2]) <= BASE_PLANE_MARGIN_M else 0.0


def require_graspable(dims: np.ndarray) -> None:
  """Decision 6.2: the object stands upright, so its narrower horizontal extent must fit the gripper."""
  if min(dims[0], dims[1]) > GRIPPER_OPENING_M + WIDTH_TOLERANCE_M:
    raise ApiError(grpc.StatusCode.ABORTED, TOO_WIDE)


def extent_along(dims: np.ndarray, axis: np.ndarray) -> float:
  """Length of the box (object frame, axis-aligned) projected on a unit axis."""
  return float(np.abs(axis) @ dims)


def deepened(object_t_tool: np.ndarray, dims: np.ndarray) -> np.ndarray:
  """Decision 6.6: the tool moved along its own +z (the approach) so the finger pads overlap the object."""
  approach = object_t_tool[:3, 2]
  moved = object_t_tool.copy()
  moved[:3, 3] += approach * min(MAX_DEPTH_M, extent_along(dims, approach) - DEPTH_MARGIN_M)
  return moved


def arm_positions(joints: dict[str, float]) -> list[float]:
  """An IK solution's panda_joint1..7 positions."""
  missing = [n for n in ARM_JOINTS if n not in joints]
  if missing:
    raise ApiError(grpc.StatusCode.INTERNAL, f"the grasp planner's IK solution lacks {missing}")
  return [joints[n] for n in ARM_JOINTS]


def choose(grasps: list[Grasp], dims: np.ndarray, check: Callable[[list[float]], tuple[bool, str]],
           deep: Callable[[Grasp], list[float] | None] | None = None) -> Grasp:
  """Decision 6.5, Addendum B.2 and 6.7: the best grasp whose object extent along the finger axis (tool y) fits and
  whose grasp and pregrasp joints are collision-free by `check` (Intrinsic's CheckCollisions: in_collision, debug).
  `deep(g)`: joints at the deepened grasp Pick moves to (None: no IK solution); also checked when given."""
  kept = [g for g in grasps if extent_along(dims, g.object_t_tool[:3, 1]) <= GRIPPER_OPENING_M + WIDTH_TOLERANCE_M]
  if not kept:
    raise ApiError(grpc.StatusCode.ABORTED, TOO_WIDE)
  # Checked best first, stopping at the first free one: the same choice as filtering all, with fewer calls.
  for g in sorted(kept, key=lambda g: g.quality, reverse=True):
    candidates = [("grasp", arm_positions(g.joints)), ("pregrasp", arm_positions(g.pregrasp_joints))]
    if deep is not None:
      deep_joints = deep(g)
      if deep_joints is None:
        log.info("plan_grasp grasp quality=%.4f dropped: no IK solution at the deepened grasp", g.quality)
        continue
      candidates.append(("deepened grasp", deep_joints))
    for what, joints in candidates:
      in_collision, debug = check(joints)
      if in_collision:
        log.info("plan_grasp grasp quality=%.4f dropped: %s joints in collision in world: %s", g.quality, what, debug)
        break
    else:
      return g
  raise ApiError(grpc.StatusCode.ABORTED, NOT_COLLISION_FREE)


def matrix_pose(m: np.ndarray) -> api.Pose:
  """4x4 transform to a Pose (unit quaternion with w >= 0)."""
  r = m[:3, :3]
  # Shepperd's method: divide by the largest of 4w², 4x², 4y², 4z² (numerically stable for any rotation).
  squares = [1 + r[0, 0] + r[1, 1] + r[2, 2], 1 + r[0, 0] - r[1, 1] - r[2, 2],
             1 - r[0, 0] + r[1, 1] - r[2, 2], 1 - r[0, 0] - r[1, 1] + r[2, 2]]
  i = int(np.argmax(squares))
  s = 2 * np.sqrt(squares[i])
  if i == 0:
    q = (s / 4, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
  elif i == 1:
    q = ((r[2, 1] - r[1, 2]) / s, s / 4, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
  elif i == 2:
    q = ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, s / 4, (r[1, 2] + r[2, 1]) / s)
  else:
    q = ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, s / 4)
  w, x, y, z = q if q[0] >= 0 else tuple(-v for v in q)
  return api.Pose(x=m[0, 3], y=m[1, 3], z=m[2, 3], qx=x, qy=y, qz=z, qw=w)


def response(world_t_object: api.Pose, grasp: Grasp, dims: np.ndarray) -> api.PlanGraspResponse:
  """Decision 6.7: grasp and pregrasp TCP poses in the base frame, grasp joints panda_joint1..7."""
  world = pose_matrix(world_t_object)
  return api.PlanGraspResponse(
      grasp=matrix_pose(world @ deepened(grasp.object_t_tool, dims)),
      pregrasp=matrix_pose(world @ grasp.object_t_pregrasp),
      grasp_joints=api.Joints(positions=arm_positions(grasp.joints)),
      pregrasp_joints=api.Joints(positions=arm_positions(grasp.pregrasp_joints)),
  )


def object_catalog(o) -> str | None:
  """The catalog name in a world object's user data, or None."""
  if CATALOG_KEY not in o.object_component.user_data:
    return None
  value = wrappers_pb2.StringValue()
  o.object_component.user_data[CATALOG_KEY].Unpack(value)
  return value.value


def _wait(future, deadline: float) -> bool:
  """Waits for an rclpy future (the executor thread completes it) until the monotonic deadline."""
  done = threading.Event()
  future.add_done_callback(lambda _: done.set())
  return done.wait(max(0.0, deadline - time.monotonic()))


def _ros_pose(pose: api.Pose):
  from geometry_msgs.msg import Pose  # pylint: disable=g-import-not-at-top
  ros = Pose()
  ros.position.x, ros.position.y, ros.position.z = pose.x, pose.y, pose.z
  ros.orientation.x, ros.orientation.y, ros.orientation.z, ros.orientation.w = pose.qx, pose.qy, pose.qz, pose.qw
  return ros


def _matrix(ros_pose) -> np.ndarray:
  p, q = ros_pose.position, ros_pose.orientation
  return pose_matrix(api.Pose(x=p.x, y=p.y, z=p.z, qx=q.x, qy=q.y, qz=q.z, qw=q.w))


class GraspPlanner:
  """Client of MoveIt's /apply_planning_scene and the grasp planner's action (CycloneDDS, as the planner)."""

  def __init__(self):
    # pylint: disable=g-import-not-at-top
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from moveit_msgs.srv import ApplyPlanningScene
    from moveit_planning_interfaces.action import PlanGrasps
    self.node = rclpy.create_node("robot_service_grasp")
    self.scene_client = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    self.action = ActionClient(self.node, PlanGrasps, "/grasp_planning/plan_grasps")
    self.in_moveit: set[str] = set()  # objects this process added to MoveIt's scene
    self.lowered: set[str] = set()  # objects whose lowered box has been logged
    self.lock = threading.Lock()  # a scene update and its goal are not interleaved with another call's
    executor = SingleThreadedExecutor()
    executor.add_node(self.node)
    threading.Thread(target=executor.spin, daemon=True).start()

  def plan_grasp(self, world, name: str, check: Callable[[list[float]], tuple[bool, str]],
                 ik: Callable[[api.Pose, list[float]], list[float]] | None = None) -> api.PlanGraspResponse:
    """Decision 6 steps 1-7; `world` is robot_service's World, `name` an object ID, `check` Intrinsic's
    CheckCollisions (Addendum B.2), `ik(tcp_pose, seed)` Intrinsic's IK: the deepened grasp is checked too (the
    hand reaches 2 cm lower than the grasp MoveIt checked, e.g. into a taller neighbour of a small cube)."""
    catalog = object_catalog(world.get(name))
    if catalog is None:  # the robot or the camera: not a catalog object
      raise unknown_object(name)
    dims = catalog_dims(catalog)
    require_graspable(dims)
    with self.lock:
      self._apply_scene(world)
      grasps = self._plan(name, dims)
    world_t_object = world.pose(name)
    deep = None
    if ik is not None:
      def deep(g: Grasp) -> list[float] | None:
        try:
          return ik(matrix_pose(pose_matrix(world_t_object) @ deepened(g.object_t_tool, dims)), arm_positions(g.joints))
        except ApiError:
          return None
    best = choose(grasps, dims, check, deep)
    result = response(world_t_object, best, dims)
    log.info("plan_grasp object=%s grasps=%d quality=%.4f grasp=%s", name, len(grasps), best.quality,
             [round(v, 4) for v in (result.grasp.x, result.grasp.y, result.grasp.z)])
    return result

  def _apply_scene(self, world) -> None:
    """Decision 6.3: `world`'s catalog objects (not attached) as boxes; objects since gone are removed."""
    # pylint: disable=g-import-not-at-top
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    from shape_msgs.msg import SolidPrimitive
    deadline = time.monotonic() + SCENE_TIMEOUT_S
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    names = set()
    for o in world.objects():
      catalog = object_catalog(o)
      if catalog is None or o.parent.name != "root":
        continue
      dims = catalog_dims(catalog)
      pose = world.pose(o.name)
      lowered = lowered_m(catalog_static(catalog), pose.z, dims)
      if lowered and o.name not in self.lowered:
        log.info("plan_grasp MoveIt box of %s lowered by %.4f m (its top is at the robot base plane)", o.name, lowered)
        self.lowered.add(o.name)
      box = CollisionObject(id=o.name, operation=CollisionObject.ADD, pose=_ros_pose(pose))
      box.header.frame_id = PLANNING_FRAME
      box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=dims.tolist())]
      centre = Pose()
      centre.position.z, centre.orientation.w = dims[2] / 2 - lowered, 1.0
      box.primitive_poses = [centre]
      scene.world.collision_objects.append(box)
      names.add(o.name)
    for gone in sorted(self.in_moveit - names):
      remove = CollisionObject(id=gone, operation=CollisionObject.REMOVE)
      remove.header.frame_id = PLANNING_FRAME
      scene.world.collision_objects.append(remove)
    if not self.scene_client.wait_for_service(timeout_sec=SCENE_TIMEOUT_S):
      raise ApiError(grpc.StatusCode.UNAVAILABLE,
                     f"{NOT_RUNNING}: MoveIt's /apply_planning_scene is not available after {SCENE_TIMEOUT_S:.0f} s")
    future = self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
    if not _wait(future, deadline):
      self.scene_client.remove_pending_request(future)
      raise ApiError(grpc.StatusCode.UNAVAILABLE, f"MoveIt's /apply_planning_scene did not answer within {SCENE_TIMEOUT_S:.0f} s")
    if not future.result().success:
      raise ApiError(grpc.StatusCode.UNAVAILABLE, "MoveIt's /apply_planning_scene returned success=false")
    self.in_moveit = names

  def _plan(self, name: str, dims: np.ndarray) -> list[Grasp]:
    """Decision 6.4: one candidate, the object's box with its centre dims.z/2 above the origin."""
    # pylint: disable=g-import-not-at-top
    from action_msgs.msg import GoalStatus
    from moveit_msgs.msg import MoveItErrorCodes
    from moveit_planning_interfaces.action import PlanGrasps
    from moveit_planning_interfaces.msg import GraspCandidate
    candidate = GraspCandidate()
    candidate.target.id = name
    candidate.obj_dims_in_meters.x, candidate.obj_dims_in_meters.y, candidate.obj_dims_in_meters.z = dims.tolist()
    candidate.obj_t_obj_center.position.z, candidate.obj_t_obj_center.orientation.w = dims[2] / 2, 1.0
    goal = PlanGrasps.Goal(candidates=[candidate], **GOAL)
    if not self.action.wait_for_server(timeout_sec=SERVER_TIMEOUT_S):
      raise ApiError(grpc.StatusCode.UNAVAILABLE,
                     f"{NOT_RUNNING}: /grasp_planning/plan_grasps not available after {SERVER_TIMEOUT_S:.0f} s")
    deadline = time.monotonic() + RESULT_TIMEOUT_S
    sent = self.action.send_goal_async(goal)
    if not _wait(sent, deadline):
      raise ApiError(grpc.StatusCode.DEADLINE_EXCEEDED, f"the grasp planner did not answer the goal within {RESULT_TIMEOUT_S:.0f} s")
    handle = sent.result()
    if not handle.accepted:
      raise ApiError(grpc.StatusCode.FAILED_PRECONDITION, "the grasp planner rejected the goal (see grasp_planner.log)")
    result_future = handle.get_result_async()
    if not _wait(result_future, deadline):
      handle.cancel_goal_async()
      raise ApiError(grpc.StatusCode.DEADLINE_EXCEEDED,
                     f"the grasp planner returned no result within {RESULT_TIMEOUT_S:.0f} s; goal cancelled")
    wrapped = result_future.result()
    if wrapped.status != GoalStatus.STATUS_SUCCEEDED or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
      raise ApiError(grpc.StatusCode.ABORTED, f"no feasible grasp found (see grasp_planner.log): goal status "
                                              f"{wrapped.status}, MoveIt error code {wrapped.result.error_code.val}")
    return [Grasp(quality=r.grasp.grasp_quality, object_t_tool=_matrix(r.grasp.grasp_pose.pose),
                  object_t_pregrasp=_matrix(r.pre_grasp_pose.pose),
                  joints=dict(zip(r.grasp_ik_solution.name, r.grasp_ik_solution.position)),
                  pregrasp_joints=dict(zip(r.pregrasp_ik_solution.name, r.pregrasp_ik_solution.position)))
            for r in wrapped.result.ranked_grasps]
