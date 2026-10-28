"""Phase 3 Part 1 check: ICON drives the Isaac Franka through ros2_hwm.

Syncs the repository to Windows, starts sim/run_scene.py and the container
(ROBOT_BACKEND=isaac, container/run.sh), drives ICON with _session from WSL
and prints PASS/FAIL per check (phase_3.md, Part 1):
  .venv/bin/python tests/e2e/phase3_hwm_check.py
Needs the intrinsic-core:dev image built with ros2_hwm. Isaac's /joint_states
and /sim/reset are reached through tests/e2e/phase3_probe.py in the image.
Stops the container and Isaac at the end.
"""

import os
import subprocess
import sys
import time

import grpc
import numpy as np

import harness
import phase1_check as p1  # sets sys.path for gen/ and ICON's Python
from intrinsic.icon.actions import point_to_point_move_utils  # noqa: E402
from intrinsic.icon.actions import simple_gripper_pb2  # noqa: E402
from intrinsic.icon.actions import stop_utils  # noqa: E402
from intrinsic.icon.actions import tare_force_torque_sensor_utils  # noqa: E402
from intrinsic.icon.actions import trajectory_tracking_action_utils  # noqa: E402
from intrinsic.icon.proto.v1 import service_pb2 as icon_pb  # noqa: E402
from intrinsic.icon.proto.v1 import service_pb2_grpc as icon_grpc  # noqa: E402
from intrinsic.icon.proto.v1 import types_pb2  # noqa: E402
from intrinsic.icon.python import _session  # noqa: E402
from intrinsic.icon.python import actions  # noqa: E402

ISAAC_LOG = harness.LOG_DIR / "phase3_isaac.log"
ISAAC_RESTART_LOG = harness.LOG_DIR / "phase3_isaac_restart.log"
CONTAINER_LOG = harness.LOG_DIR / "phase3_container.log"
ENABLED_TIMEOUT_S = 240
CLEAR_FAULTS_TIMEOUT_S = 120
ACTION_TIMEOUT_S = 60
PROBE_TIMEOUT_S = 120
FAULT_AFTER_KILL_S = 25
FAULT_TIMEOUT_S = 10
JOINT_TOLERANCE_RAD = 0.005
MOVE_OFFSET_RAD = 0.3
GRASP_MAX_WIDTH_M = 0.005
RELEASE_MIN_WIDTH_M = 0.075
GRIPPER_SETTLE_S = 5
TARED_MAX_FORCE_N = 0.5
TARING_CYCLES = 25
# A 0.7 rad joint-1 move at 10 % speed lasts about 5 s of sim time.
STOP_SPEED_OVERRIDE = 0.1
STOP_MOVE_OFFSET_RAD = 0.7
STOP_AFTER_S = 2.0
STOPPED_VELOCITY_RAD_S = 0.01
STOP_MAX_SIM_S = 0.5
ARM_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
FINGER_NAME = "panda_finger_joint1"


def _icon(method: str, request, timeout: float = p1.DEADLINE_S):
  with p1._channel(p1.ICON_PORT) as channel:
    return getattr(icon_grpc.IconApiStub(channel), method)(
        request, timeout=timeout
    )


def _operational() -> tuple[str, str]:
  status = _icon(
      "GetOperationalStatus", icon_pb.GetOperationalStatusRequest()
  ).operational_status
  return types_pb2.OperationalState.Name(status.state), status.fault_reason


def _wait_state(want: str, timeout_s: float) -> tuple[str, str]:
  deadline = time.monotonic() + timeout_s
  while True:
    try:
      state = _operational()
    except grpc.RpcError as error:
      state = ("UNREACHABLE", p1._describe(error))
    if state[0] == want or time.monotonic() > deadline:
      return state
    time.sleep(0.5)


def _require_state(want: str, timeout_s: float) -> None:
  state, reason = _wait_state(want, timeout_s)
  if state != want:
    raise RuntimeError(f"ICON is {state} ({reason!r}), not {want}")


def _clear_faults() -> str:
  """ClearFaults (retried while ICON restarts), then waits for ENABLED."""
  deadline = time.monotonic() + CLEAR_FAULTS_TIMEOUT_S
  attempts = 0
  while True:
    attempts += 1
    try:
      _icon("ClearFaults", icon_pb.ClearFaultsRequest(), timeout=30)
      break
    except grpc.RpcError as error:
      if time.monotonic() > deadline:
        raise RuntimeError(f"ClearFaults: {p1._describe(error)}") from error
      time.sleep(1)
  _require_state("ENABLED", CLEAR_FAULTS_TIMEOUT_S)
  return f"ClearFaults OK after {attempts} attempt(s), ICON ENABLED"


def _part_status(part: str):
  return _icon("GetStatus", icon_pb.GetStatusRequest()).part_status[part]


def _run_action(part: str, action, timeout_s: float = ACTION_TIMEOUT_S) -> bool:
  """Runs action on part; True if it reported done within timeout_s."""
  with p1._channel(p1.ICON_PORT) as channel:
    with _session.Session(icon_grpc.IconApiStub(channel), [part]) as session:
      session.add_action(action)
      return session.start_action_and_wait(action, timeout_s=timeout_s)


def _move(goal, timeout_s: float = ACTION_TIMEOUT_S) -> None:
  action = point_to_point_move_utils.create_point_to_point_move_action(
      1, "arm", goal_position=goal
  )
  if not _run_action("arm", action, timeout_s):
    raise RuntimeError(f"move to {goal} did not finish within {timeout_s} s")


def _isaac_joints() -> dict[str, float]:
  result, output = harness.run_probe(
      "phase3_joint_states",
      "rmw_fastrtps_cpp",
      "phase3_probe.py",
      "joint_states",
      PROBE_TIMEOUT_S,
  )
  if result is None or "error" in result:
    raise RuntimeError(f"joint_states probe: {result or output[-300:]}")
  return dict(zip(result["names"], result["position"], strict=True))


def _isaac_arm_error(goal) -> float:
  joints = _isaac_joints()
  return max(abs(joints[n] - g) for n, g in zip(ARM_NAMES, goal, strict=True))


def _check_arm_at(goal, label: str) -> str:
  error = _isaac_arm_error(goal)
  if error >= JOINT_TOLERANCE_RAD:
    raise RuntimeError(f"Isaac {label} error {error:.5f} rad")
  return f"Isaac {label} error {error:.5f} rad"


def check_move() -> str:
  _move(p1.READY)
  goal = list(p1.READY)
  goal[0] += MOVE_OFFSET_RAD
  _move(goal)
  return _check_arm_at(goal, "joint")


def check_trajectory() -> str:
  _move(p1.READY)
  trajectory = p1._plan(p1.READY, p1.DEFAULT_Q, p1.MOTION_DEADLINE_S)
  action = trajectory_tracking_action_utils.create_trajectory_tracking_action(
      1, "arm", trajectory
  )
  if not _run_action("arm", action, p1.MOTION_DEADLINE_S):
    raise RuntimeError("trajectory did not finish")
  return _check_arm_at(p1.DEFAULT_Q, "final")


def _gripper(command) -> tuple[bool, float, float]:
  """(done, ICON width, Isaac width) after the command had time to settle."""
  action = actions.Action(
      1,
      "intrinsic.simple_gripper",
      "gripper",
      simple_gripper_pb2.SimpleGripperFixedParams(command=command),
  )
  done = _run_action("gripper", action, GRIPPER_SETTLE_S)
  icon_width = _part_status("gripper").linear_gripper_state.sensed_width
  return done, icon_width, 2 * _isaac_joints()[FINGER_NAME]


def check_gripper() -> str:
  params = simple_gripper_pb2.SimpleGripperFixedParams
  grasp = _gripper(params.GRASP)
  release = _gripper(params.RELEASE)
  detail = (
      f"GRASP done {grasp[0]} width ICON {grasp[1]:.4f} Isaac {grasp[2]:.4f} m;"
      f" RELEASE done {release[0]} width ICON {release[1]:.4f}"
      f" Isaac {release[2]:.4f} m"
  )
  if grasp[2] >= GRASP_MAX_WIDTH_M or release[2] < RELEASE_MIN_WIDTH_M:
    raise RuntimeError(detail)
  return detail


def _force_norm(status) -> float:
  w = status.wrench_at_ft
  return float(np.linalg.norm([w.x, w.y, w.z]))


def check_wrench() -> str:
  before = _part_status("ft_sensor")
  if not before.HasField("wrench_at_ft"):
    raise RuntimeError("ft_sensor part status has no wrench_at_ft")
  action = tare_force_torque_sensor_utils.create_tare_force_torque_sensor_action(
      1, "ft_sensor", num_taring_cycles=TARING_CYCLES
  )
  if not _run_action("ft_sensor", action):
    raise RuntimeError("tare_force_torque_sensor did not finish")
  after = _force_norm(_part_status("ft_sensor"))
  detail = f"force norm {_force_norm(before):.3f} N before, {after:.3f} N after taring"
  if after >= TARED_MAX_FORCE_N:
    raise RuntimeError(detail)
  return detail


def _arm_motion() -> tuple[int, float]:
  """(ICON timestamp ns, max |sensed joint velocity|) of the arm."""
  status = _part_status("arm")
  return status.timestamp_ns, max(
      abs(j.velocity_sensed) for j in status.joint_states
  )


def check_stop() -> str:
  _move(p1.READY)
  goal = list(p1.READY)
  goal[0] += STOP_MOVE_OFFSET_RAD
  _icon(
      "SetSpeedOverride",
      icon_pb.SetSpeedOverrideRequest(override_factor=STOP_SPEED_OVERRIDE),
  )
  try:
    with p1._channel(p1.ICON_PORT) as channel:
      with _session.Session(icon_grpc.IconApiStub(channel), ["arm"]) as session:
        move = point_to_point_move_utils.create_point_to_point_move_action(
            1, "arm", goal_position=goal
        )
        stop = stop_utils.create_stop_action(2, "arm")
        session.add_actions([move, stop])
        session.start_action(move.id)
        time.sleep(STOP_AFTER_S)
        stop_ns, moving_velocity = _arm_motion()
        session.start_action(stop.id)
        deadline = time.monotonic() + ACTION_TIMEOUT_S
        while True:
          now_ns, velocity = _arm_motion()
          if velocity < STOPPED_VELOCITY_RAD_S:
            break
          if time.monotonic() > deadline:
            raise RuntimeError(f"still moving at {velocity:.4f} rad/s")
          time.sleep(0.005)
  finally:
    _icon("SetSpeedOverride", icon_pb.SetSpeedOverrideRequest(override_factor=1.0))
  stop_s = (now_ns - stop_ns) * 1e-9
  detail = (
      f"moving at {moving_velocity:.3f} rad/s, stopped (< {STOPPED_VELOCITY_RAD_S}"
      f" rad/s) {stop_s:.3f} s sim time after intrinsic.stop"
  )
  if moving_velocity < 10 * STOPPED_VELOCITY_RAD_S or stop_s > STOP_MAX_SIM_S:
    raise RuntimeError(detail)
  return detail


class _Run:
  """Isaac and the container for the whole check."""

  def __init__(self):
    self.isaac = None
    self.pid = None
    self.container = None
    self._container_log = None

  def start_isaac(self, log_path) -> None:
    self.isaac = harness.Isaac(log_path)
    self.pid = self.isaac.wait_ready(harness.READY_TIMEOUT_S)
    if self.pid is None:
      raise RuntimeError(f"Isaac not READY within {harness.READY_TIMEOUT_S} s")
    print(f"isaac READY pid={self.pid}", flush=True)

  def stop_isaac(self) -> None:
    if self.isaac is not None:
      self.isaac.stop(self.pid)
      self.isaac = self.pid = None

  def start_container(self) -> None:
    self._container_log = open(CONTAINER_LOG, "w")
    self.container = subprocess.Popen(
        ["bash", str(harness.ROOT / "container/run.sh")],
        env={**os.environ, "ROBOT_BACKEND": "isaac"},
        stdin=subprocess.DEVNULL,
        stdout=self._container_log,
        stderr=subprocess.STDOUT,
    )

  def stop_container(self) -> None:
    subprocess.run(harness.docker("docker rm -f intrinsic"), capture_output=True)
    if self.container is not None:
      try:
        self.container.wait(timeout=30)
      except subprocess.TimeoutExpired:
        self.container.kill()
      self._container_log.close()
      self.container = None


def check_isaac_restart(run: _Run) -> str:
  run.stop_isaac()
  time.sleep(FAULT_AFTER_KILL_S)
  state, reason = _operational()
  if state != "FAULTED":
    raise RuntimeError(f"ICON is {state} {FAULT_AFTER_KILL_S} s after Isaac stopped")
  run.start_isaac(ISAAC_RESTART_LOG)
  cleared = _clear_faults()
  moved = check_move()
  return f"FAULTED ({reason!r}) {FAULT_AFTER_KILL_S} s after kill; {cleared}; move: {moved}"


def check_epoch() -> str:
  result, output = harness.run_probe(
      "phase3_reset", "rmw_fastrtps_cpp", "phase3_probe.py", "reset", PROBE_TIMEOUT_S
  )
  if result is None or not result.get("success"):
    raise RuntimeError(f"/sim/reset probe: {result or output[-300:]}")
  state, reason = _wait_state("FAULTED", FAULT_TIMEOUT_S)
  if state != "FAULTED" or "simulation reset" not in reason:
    raise RuntimeError(f"after /sim/reset ICON is {state} ({reason!r})")
  return f"/sim/reset {result['message']!r} -> FAULTED ({reason!r}); {_clear_faults()}"


def _run_check(name: str, check) -> bool:
  try:
    print(f"PASS {name}: {check()}", flush=True)
    return True
  except Exception as error:  # pylint: disable=broad-except
    print(f"FAIL {name}: {p1._describe(error)}", flush=True)
    return False


def main() -> int:
  start = time.monotonic()
  harness.sync_isaac()
  run = _Run()
  results = []
  try:
    run.start_isaac(ISAAC_LOG)
    run.start_container()
    p1._wait_for_ports()
    _require_state("ENABLED", ENABLED_TIMEOUT_S)
    print(f"ICON ENABLED after {time.monotonic() - start:.1f} s", flush=True)
    checks = (
        ("move", check_move),
        ("trajectory", check_trajectory),
        ("gripper", check_gripper),
        ("wrench", check_wrench),
        ("stop", check_stop),
        ("isaac_restart", lambda: check_isaac_restart(run)),
        ("epoch", check_epoch),
    )
    results = [_run_check(name, check) for name, check in checks]
  except Exception as error:  # pylint: disable=broad-except
    print(f"FAIL setup: {p1._describe(error)}", flush=True)
    results = [False]
  finally:
    run.stop_container()
    run.stop_isaac()
  print(f"elapsed {time.monotonic() - start:.1f}")
  return 0 if results and all(results) else 1


if __name__ == "__main__":
  sys.exit(main())
