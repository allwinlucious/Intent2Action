"""Unit tests for the plan-tier ICON, clock-independent state and Isaac stand-in."""

from pathlib import Path
from types import SimpleNamespace
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "container/stage/vendor"))
from robot_service import robot_service_pb2 as api  # noqa: E402
sys.path.append(str(ROOT / "container/services"))

from google.protobuf import any_pb2, duration_pb2, wrappers_pb2  # noqa: E402
import grpc  # noqa: E402
import pytest  # noqa: E402
from intrinsic.icon.actions import simple_gripper_pb2  # noqa: E402
from intrinsic.icon.actions import stop_utils  # noqa: E402
from intrinsic.icon.actions import trajectory_tracking_action_utils  # noqa: E402
from intrinsic.icon.proto import joint_space_pb2  # noqa: E402
from intrinsic.icon.python import actions  # noqa: E402
from intrinsic.math.proto import point_pb2, pose_pb2, quaternion_pb2  # noqa: E402

from robot import plan_tier  # noqa: E402
from robot.ops import ApiError  # noqa: E402


class FakeClock:

  def __init__(self):
    self.time = 0.0
    self.published = []

  def now(self):
    return self.time

  def advance(self, seconds):
    self.time += seconds

  def advancing(self, within_s):
    return True

  def publish_joints(self, arm, width):
    self.published.append((list(arm), width))


def _object(name: str, catalog: str):
  value = any_pb2.Any()
  value.Pack(wrappers_pb2.StringValue(value=catalog))
  return SimpleNamespace(name=name, object_component=SimpleNamespace(user_data={"catalog": value}))


class FakeWorld:

  def __init__(self):
    self.calls = []
    self.found = []
    self.poses = {}

  def _call(self, method, request):
    self.calls.append((method, request))
    return SimpleNamespace()

  def objects(self):
    return self.found

  def pose(self, name):
    return self.poses[name]


class FakePlanner:

  def __init__(self, tcp=None):
    self.tcp = tcp or api.Pose(x=0.0, y=0.0, z=0.025, qw=1.0)

  def fk(self, joints, tool):
    return self.tcp


def _trajectory_action(end, seconds=2.0):
  trajectory = joint_space_pb2.JointTrajectoryPVA()
  trajectory.state.add().position.extend([0.0] * 7)
  trajectory.state.add().position.extend(end)
  trajectory.time_since_start.add().CopyFrom(duration_pb2.Duration())
  trajectory.time_since_start.add().CopyFrom(duration_pb2.Duration(
      seconds=int(seconds), nanos=round((seconds % 1) * 1e9)))
  return trajectory_tracking_action_utils.create_trajectory_tracking_action(1, "arm", trajectory)


def _gripper_action(command):
  return actions.Action(1, "intrinsic.simple_gripper", "gripper",
                        simple_gripper_pb2.SimpleGripperFixedParams(command=command))


def _run(icon, action, events=None):
  events = events if events is not None else []
  icon.run_action([], action, lambda done: events.append(("stopped", done)) or True,
                  on_started=lambda: events.append(("started", True)))
  return events


def test_trajectory_jump_advances_clock_writes_both_worlds_and_orders_callbacks():
  world, clock = FakeWorld(), FakeClock()
  icon = plan_tier.PlanIcon(FakePlanner(), world, clock)
  icon.set_speed(0.5)
  end = [0.1 * i for i in range(7)]
  events = _run(icon, _trajectory_action(end, 2.5))

  assert icon.arm_joints() == end
  assert clock.time == 5.0
  assert events == [("started", True), ("stopped", True)]
  assert [request.world_id for _, request in world.calls] == ["world", "sim_world"]
  assert all(list(request.joint_positions) == end for _, request in world.calls)


def test_grasp_width_one_none_and_two_objects_and_release(caplog):
  world, clock = FakeWorld(), FakeClock()
  icon = plan_tier.PlanIcon(FakePlanner(), world, clock)
  red = _object("red", "red_cube")
  yellow = _object("yellow", "small_yellow_cube")
  world.poses = {
      "red": api.Pose(qw=1.0),
      "yellow": api.Pose(qw=1.0),
  }

  world.found = [red]
  events = _run(icon, _gripper_action(simple_gripper_pb2.SimpleGripperFixedParams.GRASP))
  assert icon.status().part_status["gripper"].linear_gripper_state.sensed_width == 0.0495
  assert events == [("started", True), ("stopped", True)]

  world.found = []
  _run(icon, _gripper_action(simple_gripper_pb2.SimpleGripperFixedParams.GRASP))
  assert icon.status().part_status["gripper"].linear_gripper_state.sensed_width == 0.0

  world.found = [red, yellow]
  _run(icon, _gripper_action(simple_gripper_pb2.SimpleGripperFixedParams.GRASP))
  assert icon.status().part_status["gripper"].linear_gripper_state.sensed_width == 0.0345
  assert "inside multiple objects" in caplog.text

  events = _run(icon, _gripper_action(simple_gripper_pb2.SimpleGripperFixedParams.RELEASE))
  assert icon.status().part_status["gripper"].linear_gripper_state.sensed_width == 0.075
  assert events == [("started", True), ("stopped", True)]
  assert clock.time == 2.0


def test_stop_orders_callbacks_and_unsupported_actions_are_unimplemented():
  icon = plan_tier.PlanIcon(FakePlanner(), FakeWorld(), FakeClock())
  assert _run(icon, stop_utils.create_stop_action(1, "arm")) == [
      ("started", True), ("stopped", True)]
  unsupported = actions.Action(2, "intrinsic.force_primitive", "arm", None)
  try:
    _run(icon, unsupported)
  except ApiError as error:
    assert error.code == grpc.StatusCode.UNIMPLEMENTED
  else:
    raise AssertionError("unsupported action returned OK")

  try:
    icon.run_sequence([], [(1, "intrinsic.force_primitive", None, "arm")], [], [], (1, None), 1, 1.0,
                      lambda: 0.0, lambda: False)
  except ApiError as error:
    assert error.code == grpc.StatusCode.UNIMPLEMENTED
  else:
    raise AssertionError("unsupported sequence returned OK")


def test_misc_icon_methods():
  clock = FakeClock()
  icon = plan_tier.PlanIcon(FakePlanner(), FakeWorld(), clock)
  icon.clear_faults()
  icon.clear_faults(3.0)
  assert clock.advancing(5.0)
  assert icon.joint_max_velocities() == [plan_tier.JOG_MAX_VELOCITY_RAD_S] * 7


def _pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
  return api.Pose(x=x, y=y, z=z, qz=__import__("math").sin(yaw / 2), qw=__import__("math").cos(yaw / 2))


def test_drop_table_cube_box_floor_and_highest_support():
  cube_dims = (0.05, 0.05, 0.05)
  assert plan_tier.drop(_pose(z=0.4), cube_dims, []) == (0.0, "table")
  cube = ("cube", _pose(), cube_dims, None)
  assert plan_tier.drop(_pose(z=0.4), cube_dims, [cube]) == (0.05, "cube")
  box = ("box", _pose(), (0.25, 0.21, 0.05), 0.01)
  assert plan_tier.drop(_pose(z=0.4), cube_dims, [box]) == (0.01, "box")
  heart = ("heart", _pose(), (0.056, 0.0547, 0.0547), 0.0013)
  assert plan_tier.drop(_pose(z=0.4), (0.0573, 0.0573, 0.0159), [heart]) == (0.0547, "heart")  # a lid on the rim
  high = ("high", _pose(z=0.1), cube_dims, None)
  z, support = plan_tier.drop(_pose(z=0.4), cube_dims, [cube, high])
  assert z == pytest.approx(0.15)
  assert support == "high"


class FakeWorldStub:

  def __init__(self):
    self.poses = {}

  def GetTransform(self, request, timeout):
    name = request.node_b.by_name.object.object_name
    return SimpleNamespace(a_t_b=self.poses[name])


class FakeSync:

  def __init__(self, catalogs):
    self.attached = set()
    self.catalogs = dict(catalogs)

  def name(self, prim):
    return prim


def _scene():
  return {"objects": {
      "red_cube": {"catalog": "red_cube", "position": [0.5, 0.0, 0.0], "yaw": 0},
      "table": {"catalog": "table", "position": [0.45, 0.0, -0.75], "yaw": 0},
  }}


def _entity_request(name, pose=None):
  pose = pose or pose_pb2.Pose(orientation=quaternion_pb2.Quaternion(w=1.0))
  return SimpleNamespace(name=f"/World/objects/{name}", initial_pose=SimpleNamespace(pose=pose))


def test_plan_isaac_spawn_delete_reset_and_republish():
  world = FakeWorldStub()
  isaac = plan_tier.PlanIsaac(world, _scene())
  isaac.bind(FakeSync({name: spec["catalog"] for name, spec in _scene()["objects"].items()}))
  spawned = isaac.call(None, "/spawn_entity", _entity_request("spawn_1"), True)
  assert spawned.result.result == plan_tier.RESULT_OK
  duplicate = isaac.call(None, "/spawn_entity", _entity_request("spawn_1"), True)
  assert duplicate.result.result == plan_tier.NAME_NOT_UNIQUE

  deleted = isaac.call(None, "/delete_entity", SimpleNamespace(entity="/World/objects/spawn_1"), True)
  assert deleted.result.result == plan_tier.RESULT_OK
  unknown = isaac.call(None, "/delete_entity", SimpleNamespace(entity="/World/objects/spawn_1"), True)
  assert unknown.result.result == plan_tier.RESULT_NOT_FOUND

  before = isaac.epoch
  assert isaac.call(None, "/sim/reset", SimpleNamespace(), False).success
  assert isaac.epoch == before + 1
  with isaac.lock:
    isaac.poses.clear()
    isaac.pending.clear()
  isaac._tick()
  assert set(isaac.pending) == {"red_cube", "table"}


def test_plan_isaac_attached_follows_world_and_detach_drops_synchronous_pose():
  world = FakeWorldStub()
  world.poses["red_cube"] = pose_pb2.Pose(
      position=point_pb2.Point(x=0.4, y=0.1, z=0.3), orientation=quaternion_pb2.Quaternion(w=1.0))
  isaac = plan_tier.PlanIsaac(world, _scene())
  sync = FakeSync({name: spec["catalog"] for name, spec in _scene()["objects"].items()})
  isaac.bind(sync)
  sync.attached.add("red_cube")
  isaac._tick()
  assert isaac.poses["red_cube"][:3] == (0.4, 0.1, 0.3)

  world.poses["red_cube"] = pose_pb2.Pose(
      position=point_pb2.Point(x=0.4, y=0.1, z=0.2), orientation=quaternion_pb2.Quaternion(w=1.0))
  sync.attached.clear()
  isaac._tick()
  assert isaac.poses["red_cube"][:3] == (0.4, 0.1, 0.0)


def test_reset_arm_returns_home_open_and_writes_both_worlds():
  world, clock = FakeWorld(), FakeClock()
  icon = plan_tier.PlanIcon(FakePlanner(), world, clock)
  _run(icon, _trajectory_action([0.3] * 7, 1.0))
  world.calls.clear()
  icon.reset_arm()
  assert icon.arm_joints() == list(plan_tier.HOME)
  assert icon.status().part_status["gripper"].linear_gripper_state.sensed_width == plan_tier.grasp.GRIPPER_OPENING_M
  assert [request.world_id for _, request in world.calls] == ["world", "sim_world"]
