"""The plan-tier stand-ins for ICON and Isaac.

There are no dynamics (slips, toppling, pushed neighbours or contact forces), force sequences, camera or
calibration. Joints jump instantly to trajectory ends; the clock still advances by planned motion time.
"""

import logging
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from typing import Callable

import grpc
from intrinsic.icon.actions import simple_gripper_pb2
from intrinsic.icon.actions import trajectory_tracking_action_pb2
from intrinsic.icon.proto.v1 import service_pb2 as icon_pb
from intrinsic.world.proto import object_world_refs_pb2 as refs_pb
from intrinsic.world.proto import object_world_service_pb2 as world_pb
from intrinsic.world.proto import object_world_updates_pb2 as updates_pb
import numpy as np
import yaml

from robot import grasp
from robot.motion import pose_matrix
from robot.ops import ApiError
from robot_model.franka_scene_object import HOME
from robot_service import robot_service_pb2 as api

WORLD = "world"
SIM_WORLD = "sim_world"
ROBOT = "franka"
ARM, GRIPPER, FT_SENSOR = "arm", "gripper", "ft_sensor"
JOG_MAX_VELOCITY_RAD_S = math.radians(30.0)
JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
TICK_S = 0.05
GRIPPER_SECONDS = 0.5
CONTAINS_TOLERANCE_M = 0.005
GRASP_CLEARANCE_M = 0.0005
RESULT_OK = 1
RESULT_NOT_FOUND = 2
NAME_NOT_UNIQUE = 101
CATALOG_DIR = Path("/opt/app/assets/objects")
if not CATALOG_DIR.is_dir():
  CATALOG_DIR = Path(__file__).resolve().parents[3] / "assets/objects"

log = logging.getLogger("robot_service")

Pose = tuple[float, float, float, float, float, float, float]
Support = tuple[str, object, object, float | None]


def _pose_values(pose) -> Pose:
  if hasattr(pose, "x"):
    return pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw
  return tuple(pose)


def _yaw(pose) -> float:
  _, _, _, qx, qy, qz, qw = _pose_values(pose)
  return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def drop(pose, dims, supports: list[Support]) -> tuple[float, str]:
  """Returns the dropped object's bottom z and the highest support under its footprint centre."""
  x, y, *_ = _pose_values(pose)
  best_z, best_name = 0.0, "table"
  for name, support_pose, support_dims, container_floor_m in supports:
    sx, sy, sz, *_ = _pose_values(support_pose)
    yaw = _yaw(support_pose)
    dx, dy = x - sx, y - sy
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    if abs(local_x) > support_dims[0] / 2 or abs(local_y) > support_dims[1] / 2:
      continue
    # A container's floor, unless the object is wider than its opening (a lid): then it rests on the walls.
    # shortcut: the wall is taken as the floor thickness and yaw is ignored (true of every catalog box today).
    lid = container_floor_m is not None and all(d >= s - 2 * container_floor_m for d, s in zip(dims[:2], support_dims[:2]))
    top = sz + (container_floor_m if container_floor_m is not None and not lid else support_dims[2])
    if top > best_z:
      best_z, best_name = top, name
  return best_z, best_name


def _by_name(name: str) -> refs_pb.ObjectReference:
  return refs_pb.ObjectReference(by_name=refs_pb.ObjectReferenceByName(object_name=name))


def _object_node(name: str) -> refs_pb.TransformNodeReference:
  return refs_pb.TransformNodeReference(by_name=refs_pb.TransformNodeReferenceByName(
      object=refs_pb.ObjectReferenceByName(object_name=name)))


def _catalog_entry(catalog: str) -> dict:
  return yaml.safe_load((CATALOG_DIR / catalog / "object.yaml").read_text())


def _catalog_dims(catalog: str) -> np.ndarray:
  return np.array(_catalog_entry(catalog)["dims"], dtype=float)


class PlanClock:
  """Wall time plus virtual action time, published as ROS simulation time."""

  def __init__(self):
    import rclpy  # pylint: disable=g-import-not-at-top
    from rclpy.executors import SingleThreadedExecutor  # pylint: disable=g-import-not-at-top
    from rosgraph_msgs.msg import Clock  # pylint: disable=g-import-not-at-top
    from sensor_msgs.msg import JointState  # pylint: disable=g-import-not-at-top

    self._clock_type = Clock
    self._joint_state_type = JointState
    self._start = time.monotonic()
    self._virtual = 0.0
    self._lock = threading.Lock()
    self._arm = list(HOME)
    self._width = grasp.GRIPPER_OPENING_M
    self.node = rclpy.create_node("robot_service_plan")
    self._clock_publisher = self.node.create_publisher(Clock, "/clock", 10)
    self._joint_publisher = self.node.create_publisher(JointState, "/joint_states", 10)
    self.node.create_timer(0.02, self._publish_clock)
    self.node.create_timer(TICK_S, self._publish_joints)
    executor = SingleThreadedExecutor()
    executor.add_node(self.node)
    threading.Thread(target=executor.spin, daemon=True).start()

  def now(self) -> float:
    with self._lock:
      virtual = self._virtual
    return time.monotonic() - self._start + virtual

  def advance(self, seconds: float) -> None:
    with self._lock:
      self._virtual += seconds

  def advancing(self, within_s: float) -> bool:
    return True

  def publish_joints(self, arm: list[float], width: float) -> None:
    with self._lock:
      self._arm, self._width = list(arm), width
    self._publish_joints()

  @staticmethod
  def _set_stamp(stamp, seconds: float) -> None:
    stamp.sec, stamp.nanosec = divmod(round(seconds * 1e9), 1_000_000_000)

  def _publish_clock(self) -> None:
    message = self._clock_type()
    self._set_stamp(message.clock, self.now())
    self._clock_publisher.publish(message)

  def _publish_joints(self) -> None:
    with self._lock:
      arm, width = list(self._arm), self._width
    message = self._joint_state_type()
    self._set_stamp(message.header.stamp, self.now())
    message.name = [*JOINT_NAMES, "panda_finger_joint1", "panda_finger_joint2"]
    message.position = [*arm, width / 2, width / 2]
    self._joint_publisher.publish(message)


class PlanIcon:
  """The subset of ICON used by robot_service, with instantaneous state changes."""

  def __init__(self, planner, world, clock):
    self.planner = planner
    self.world = world
    self.clock = clock
    self._joints = list(HOME)
    self._width = grasp.GRIPPER_OPENING_M
    self._speed = 1.0
    self._payload = (0.0, (0.0, 0.0, 0.0))
    self.clock.publish_joints(self._joints, self._width)

  def status(self) -> icon_pb.GetStatusResponse:
    response = icon_pb.GetStatusResponse()
    arm = response.part_status[ARM]
    for position in self._joints:
      arm.joint_states.add(position_sensed=position)
    response.part_status[GRIPPER].linear_gripper_state.sensed_width = self._width
    response.part_status[FT_SENSOR].wrench_at_ft.SetInParent()
    return response

  def operational(self) -> tuple[str, str]:
    return "ENABLED", ""

  def require_enabled(self) -> None:
    return None

  def wait_state(self, want: str, timeout_s: float) -> tuple[str, str]:
    return "ENABLED", ""

  def clear_faults(self, enabled_timeout_s: float = 10.0) -> None:
    return None

  def reset_arm(self) -> None:
    """What Isaac's scene reset does to the robot: arm at HOME, gripper open."""
    self._joints, self._width = list(HOME), grasp.GRIPPER_OPENING_M
    self._write_joints()
    self.clock.publish_joints(self._joints, self._width)
    log.info("plan_tier reset arm to HOME")

  def set_speed(self, factor: float) -> None:
    self._speed = factor

  def set_payload(self, mass_kg: float, com: tuple[float, float, float]) -> None:
    self._payload = (mass_kg, com)

  def arm_joints(self, status: icon_pb.GetStatusResponse | None = None) -> list[float]:
    status = status or self.status()
    return [joint.position_sensed for joint in status.part_status[ARM].joint_states]

  def joint_max_velocities(self) -> list[float]:
    return [JOG_MAX_VELOCITY_RAD_S] * 7

  def run_action(
      self,
      parts: list[str],
      action,
      stop_when: Callable[[bool], bool],
      cancel: threading.Event | None = None,
      condition=None,
      on_started: Callable[[], None] | None = None,
  ) -> None:
    action_type = action.proto.action_type_name
    if on_started is not None:
      on_started()
    if action_type == "intrinsic.trajectory_tracking":
      params = trajectory_tracking_action_pb2.TrajectoryTrackingActionFixedParams()
      action.proto.fixed_parameters.Unpack(params)
      trajectory = params.trajectory
      before = list(self._joints)
      self._joints = list(trajectory.state[-1].position)
      duration = trajectory.time_since_start[-1]
      planned = (duration.seconds + duration.nanos * 1e-9) / self._speed
      self._write_joints()
      self.clock.advance(planned)
      log.info("plan_tier move from=%s to=%s planned_s=%.3f", np.round(before, 4).tolist(),
               np.round(self._joints, 4).tolist(), planned)
    elif action_type == "intrinsic.simple_gripper":
      params = simple_gripper_pb2.SimpleGripperFixedParams()
      action.proto.fixed_parameters.Unpack(params)
      if params.command == simple_gripper_pb2.SimpleGripperFixedParams.GRASP:
        self._width, object_name = self._closing_width()
        command = "GRASP"
      elif params.command == simple_gripper_pb2.SimpleGripperFixedParams.RELEASE:
        self._width, object_name = grasp.GRIPPER_OPENING_M, "none"
        command = "RELEASE"
      else:
        raise ApiError(grpc.StatusCode.UNIMPLEMENTED, f"{action_type} is not available in the plan tier")
      self.clock.publish_joints(self._joints, self._width)
      self.clock.advance(GRIPPER_SECONDS)
      log.info("plan_tier gripper command=%s width_m=%.4f object=%s", command, self._width, object_name)
    elif action_type != "intrinsic.stop":
      raise ApiError(grpc.StatusCode.UNIMPLEMENTED, f"{action_type} is not available in the plan tier")
    stop_when(True)

  def run_sequence(
      self,
      parts: list[str],
      actions,
      transitions,
      aborts,
      success,
      stop_id: int,
      timeout_s: float,
      clock: Callable[[], float],
      cancel: Callable[[], bool],
      watch: Callable[[], str | None] = lambda: None,
  ):
    names = ", ".join(sorted({name for _, name, _, _ in actions}))
    raise ApiError(grpc.StatusCode.UNIMPLEMENTED, f"{names} is not available in the plan tier")

  def jog(self, joint_index: int, normalized_velocity: float, keep_going: Callable[[], bool],
          duration_s: float, period_s: float = 0.1) -> None:
    raise ApiError(grpc.StatusCode.UNIMPLEMENTED, "jogging is not available in the plan tier")

  def _write_joints(self) -> None:
    for world_id in (WORLD, SIM_WORLD):
      self.world._call("UpdateObjectJoints", updates_pb.UpdateObjectJointsRequest(
          world_id=world_id, object=_by_name(ROBOT), joint_names=JOINT_NAMES, joint_positions=self._joints))
    self.clock.publish_joints(self._joints, self._width)

  def _closing_width(self) -> tuple[float, str]:
    tcp = self.planner.fk(self._joints, api.TCP)
    world_t_tcp = pose_matrix(tcp)
    point = world_t_tcp[:3, 3]
    candidates = []
    for obj in self.world.objects():
      catalog = grasp.object_catalog(obj)
      if catalog is None:
        continue
      dims = _catalog_dims(catalog)
      world_t_object = pose_matrix(self.world.pose(obj.name))
      local = np.linalg.inv(world_t_object) @ np.append(point, 1.0)
      if (abs(local[0]) > dims[0] / 2 + CONTAINS_TOLERANCE_M
          or abs(local[1]) > dims[1] / 2 + CONTAINS_TOLERANCE_M
          or local[2] < -CONTAINS_TOLERANCE_M
          or local[2] > dims[2] + CONTAINS_TOLERANCE_M):
        continue
      closing_axis = world_t_object[:3, :3].T @ world_t_tcp[:3, 1]
      candidates.append((max(0.0, grasp.extent_along(dims, closing_axis) - GRASP_CLEARANCE_M), obj.name))
    if not candidates:
      return 0.0, "none"
    candidates.sort()
    if len(candidates) > 1:
      log.warning("plan_tier grasp TCP is inside multiple objects %s; using the smallest", candidates)
    return candidates[0]


class PlanIsaac:
  """An in-process truth table exposing the members scene_sync expects from its ROS adapter."""

  def __init__(self, world_stub, scene_yaml):
    self.world = world_stub
    self.lock = threading.Lock()
    self.poses: dict[str, Pose] = {}
    self.pending: dict[str, Pose] = {}
    self.tf_arrived = threading.Event()
    self.clock = 0.0
    self.epoch = 1
    self.sync = None
    self._start = time.monotonic()
    self._scene = scene_yaml
    self._catalogs = {name: spec["catalog"] for name, spec in scene_yaml["objects"].items()}
    self._defaults = self._scene_poses(scene_yaml)
    self._truth = dict(self._defaults)
    self._was_attached: set[str] = set()
    threading.Thread(target=self._run, daemon=True).start()

  @staticmethod
  def _scene_poses(scene_yaml) -> dict[str, Pose]:
    poses = {}
    for name, spec in scene_yaml["objects"].items():
      x, y, z = spec["position"]
      yaw = math.radians(spec.get("yaw", 0.0))
      poses[name] = (x, y, z, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return poses

  def bind(self, sync) -> None:
    self.sync = sync

  def take_pending(self) -> dict[str, Pose]:
    with self.lock:
      pending, self.pending = self.pending, {}
      self.tf_arrived.clear()
    return pending

  def call(self, context, service: str, request, request_renders: bool):
    if service == "/spawn_entity":
      prim = request.name.rsplit("/", 1)[-1]
      with self.lock:
        if prim in self._truth:
          return self._entity_result(NAME_NOT_UNIQUE, f"entity name {prim!r} is not unique")
        pose = request.initial_pose.pose
        self._truth[prim] = (pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z, pose.orientation.w)
      return self._entity_result(RESULT_OK, "")
    if service == "/delete_entity":
      prim = request.entity.rsplit("/", 1)[-1]
      with self.lock:
        if prim not in self._truth:
          return self._entity_result(RESULT_NOT_FOUND, f"entity {prim!r} was not found")
        del self._truth[prim]
      return self._entity_result(RESULT_OK, "")
    if service == "/sim/reset":
      with self.lock:
        self._truth = dict(self._defaults)
        self._was_attached = set()  # a cube held across the reset must not be "dropped" back into the new scene
        self.epoch += 1
      return SimpleNamespace(success=True, message="")
    if service in ("/sim/attach_board", "/sim/detach_board"):
      return SimpleNamespace(success=False, message="no calibration board in the plan tier")
    raise ValueError(f"unknown plan-tier service {service}")

  @staticmethod
  def _entity_result(result: int, message: str):
    return SimpleNamespace(result=SimpleNamespace(result=result, error_message=message))

  def _run(self) -> None:
    while True:
      try:
        self._tick()
      except Exception:  # pylint: disable=broad-except
        # One failed world read (an object mid-delete) must not end the thread that publishes every pose.
        log.exception("plan_tier tick failed; continuing")
      time.sleep(TICK_S)

  def _tick(self) -> None:
    self.clock = time.monotonic() - self._start
    sync = self.sync
    attached = set(sync.attached) if sync is not None else set()
    # World reads happen outside the lock; the truth table is changed under it, so a reset in between wins.
    with self.lock:
      held = [p for p in attached if p in self._truth]
      released = [p for p in self._was_attached - attached if p in self._truth]
      epoch = self.epoch
    updates = {prim: self._world_pose(prim) for prim in held}
    for prim in released:
      pose = self._world_pose(prim)
      catalog = self._catalogs.get(prim) or (sync.catalogs.get(prim) if sync is not None else None)
      updates[prim] = self._dropped_pose(prim, pose, catalog) if catalog is not None else pose
    with self.lock:
      if epoch == self.epoch:
        self._truth.update({p: pose for p, pose in updates.items() if p in self._truth})
        self._was_attached = attached
      self.poses.update(self._truth)
      self.pending.update(self._truth)
      present = set(self._truth)
      for prim in list(self.poses):
        if prim not in present:
          self.poses.pop(prim, None)
          self.pending.pop(prim, None)
    if self.pending:
      self.tf_arrived.set()

  def _world_pose(self, prim: str) -> Pose:
    name = self.sync.name(prim)
    response = self.world.GetTransform(world_pb.GetTransformRequest(
        world_id=WORLD, node_a=refs_pb.TransformNodeReference(id="root"), node_b=_object_node(name)), timeout=10.0)
    pose = response.a_t_b
    return (pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)

  def _dropped_pose(self, prim: str, pose: Pose, catalog: str) -> Pose:
    supports = []
    for other, other_pose in self._truth.items():
      if other == prim:
        continue
      other_catalog = self._catalogs.get(other) or self.sync.catalogs.get(other)
      if other_catalog is None:
        continue
      entry = _catalog_entry(other_catalog)
      container = entry.get("container")
      supports.append((other, other_pose, entry["dims"], container["floor"] if container else None))
    z, support = drop(pose, _catalog_dims(catalog), supports)
    x, y, old_z, *_ = pose
    yaw = _yaw(pose)
    result = (x, y, z, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    log.info("plan_tier drop object=%s from_z=%.4f to_z=%.4f support=%s", prim, old_z, z, support)
    return result
