"""Phase 5c check: PlanGrasp with Intrinsic's MoveIt grasp planner (phase_5c.md, done criteria).

Syncs the repository to Windows, starts sim/run_scene.py and the container
(ROBOT_BACKEND=isaac, container/run.sh), and prints PASS/FAIL per check, in order:
  .venv/bin/python tests/e2e/phase5c_check.py
`errors` restarts the container with GRASP_PLANNER=off. Stops the container and Isaac at the end.
"""

import os
import statistics
import subprocess
import sys
import time

import grpc
import numpy as np

import harness
import phase3_hwm_check as hwm  # sets sys.path for gen/ and ICON's Python
import phase1_check as p1
import phase3_cuj_check as cuj
from robot_service import robot_service_pb2 as api  # noqa: E402

ISAAC_LOG = harness.LOG_DIR / "phase5c_isaac.log"
CONTAINER_LOG = harness.LOG_DIR / "phase5c_container.log"
OFF_CONTAINER_LOG = harness.LOG_DIR / "phase5c_container_planner_off.log"
LOGS = "/var/lib/intrinsic/logs"
PLANNER_UP_S = 120
TRIALS = 10
MIN_SUCCESSES = 9
LIFT_M = 0.10
LIFT_MIN_M = 0.08
SETTLE_TOLERANCE_M = 0.005
SETTLE_TIMEOUT_S = 10.0
ACTION_PROBE = """
import sys, rclpy
from rclpy.action import ActionClient
from moveit_planning_interfaces.action import PlanGrasps
rclpy.init()
node = rclpy.create_node('phase5c_probe')
ok = ActionClient(node, PlanGrasps, '/grasp_planning/plan_grasps').wait_for_server(timeout_sec=float(sys.argv[1]))
print('server available' if ok else 'server not available')
sys.exit(0 if ok else 1)
"""
_plan_seconds: list[float] = []
_plan_calls = [0]
_goals = [0]
_log_start: dict[str, int] = {}


def _ros(command: str, timeout_s: float) -> subprocess.CompletedProcess:
  """`command` in the container with Humble, the image's ROS packages and robot_service's CycloneDDS."""
  script = ("set +u; source /opt/ros/humble/setup.bash; source /opt/ros_ext/setup.bash; set -u; "
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///opt/app/container/dds/cyclonedds.xml; "
            + command)
  return subprocess.run(harness.docker("docker exec -i intrinsic bash"), input=script, capture_output=True, text=True,
                        errors="replace", timeout=timeout_s)


def _log_lines(name: str) -> list[str]:
  out = subprocess.run(harness.docker(f"docker exec intrinsic cat {LOGS}/{name}.log"), capture_output=True, text=True,
                       errors="replace", check=True).stdout
  return out.splitlines()


def _plan(name: str) -> api.PlanGraspResponse:
  """PlanGrasp of a graspable object; counts the goals it sent to the grasp planner."""
  _plan_calls[0] += 1
  start = time.monotonic()
  try:
    response = cuj._call(cuj._robot, "PlanGrasp", api.PlanGraspRequest(object=name))
  except grpc.RpcError as error:
    # NOT_FOUND (step 1) and UNAVAILABLE (scene or server) end before a goal; every other error after it.
    _goals[0] += error.code() not in (grpc.StatusCode.NOT_FOUND, grpc.StatusCode.UNAVAILABLE)
    raise
  finally:
    _plan_seconds.append(time.monotonic() - start)
  _goals[0] += 1
  return response


def _plan_error(name: str) -> grpc.RpcError:
  _plan_calls[0] += 1
  try:
    response = cuj._call(cuj._robot, "PlanGrasp", api.PlanGraspRequest(object=name))
  except grpc.RpcError as error:
    return error
  raise RuntimeError(f"PlanGrasp({name!r}) returned OK: {response}")


def _world_pose(name: str) -> api.Pose:
  return {o.name: o for o in cuj._call(cuj._robot, "ListObjects").objects}[name].pose


def _settled(name: str) -> None:
  """After ResetScene: `world` (scene_sync) has caught up with Isaac's pose of `name`."""
  deadline = time.monotonic() + SETTLE_TIMEOUT_S
  while True:
    belief, truth = _world_pose(name), cuj._call(cuj._sim, "GetTruePose", api.GetTruePoseRequest(frame=name))
    gap = float(np.linalg.norm([belief.x - truth.x, belief.y - truth.y, belief.z - truth.z]))
    if gap <= SETTLE_TOLERANCE_M:
      return
    if time.monotonic() > deadline:
      raise RuntimeError(f"world's {name} {gap * 1e3:.1f} mm from Isaac's {SETTLE_TIMEOUT_S:.0f} s after ResetScene")
    time.sleep(0.2)


def _trial(name: str) -> str:
  cuj._call(cuj._sim, "ResetScene")
  _settled(name)
  plan = _plan(name)
  try:
    cuj._move("MoveToPose", pose=plan.pregrasp)
    cuj._move("MoveToPose", pose=plan.grasp, motion=api.LINEAR)
    grasped = cuj._gripper("Grasp")
    cuj._call(cuj._robot, "AttachObject", api.ObjectRequest(name=name))
    z0 = cuj._true_z(name)
    lifted = cuj._at(plan.grasp, plan.grasp.x, plan.grasp.y, plan.grasp.z + LIFT_M)
    cuj._move("MoveToPose", pose=lifted, motion=api.LINEAR)
    rose = cuj._true_z(name) - z0
    gripping = cuj._call(cuj._robot, "IsGripping")
  finally:
    for method, request in (("Release", None), ("DetachObject", api.ObjectRequest(name=name))):
      try:
        cuj._gripper(method) if request is None else cuj._call(cuj._robot, method, request)
      except (grpc.RpcError, RuntimeError):
        pass
  detail = (f"rose {rose * 1e3:.1f} mm, IsGripping {gripping.gripping} (width {gripping.width * 1e3:.1f} mm, "
            f"Grasp width {grasped.width * 1e3:.1f} mm), plan {_plan_seconds[-1]:.2f} s")
  if rose < LIFT_MIN_M or not gripping.gripping:
    raise RuntimeError(detail)
  return detail


def _trials(name: str) -> str:
  successes, first = 0, len(_plan_seconds)
  for i in range(TRIALS):
    try:
      detail = _trial(name)
      successes += 1
      print(f"  {name} trial {i + 1}: ok, {detail}", flush=True)
    except (grpc.RpcError, RuntimeError) as error:
      print(f"  {name} trial {i + 1}: FAILED {p1._describe(error)}", flush=True)
  seconds = _plan_seconds[first:]
  summary = (f"{successes}/{TRIALS} lifted; PlanGrasp median {statistics.median(seconds):.2f} s, "
             f"max {max(seconds):.2f} s")
  if successes < MIN_SUCCESSES:
    raise RuntimeError(f"{summary} (need {MIN_SUCCESSES})")
  return summary


def check_planner_up() -> str:
  start = time.monotonic()
  processes = {}
  for name, pattern in (("move_group", "lib/moveit_ros_move_group/move_group"),
                        ("grasp_planner", "lib/grasp_planner/grasp_planner")):
    while True:
      found = subprocess.run(harness.docker(f"docker exec intrinsic pgrep -f {pattern}"), capture_output=True, text=True)
      if found.returncode == 0:
        processes[name] = found.stdout.split()[0]
        break
      if time.monotonic() - start > PLANNER_UP_S:
        raise RuntimeError(f"{name} not running {PLANNER_UP_S} s after robot_service was ready")
      time.sleep(1)
  left = max(1.0, PLANNER_UP_S - (time.monotonic() - start))
  probe = _ros(f"python3 - {left:.0f} <<'EOF'\n{ACTION_PROBE}\nEOF", left + 30)
  if probe.returncode != 0:
    raise RuntimeError(f"/grasp_planning/plan_grasps: {(probe.stdout + probe.stderr).strip()[-400:]}")
  return (f"move_group pid {processes['move_group']}, grasp_planner pid {processes['grasp_planner']}; "
          f"/grasp_planning/plan_grasps available {time.monotonic() - start:.0f} s after robot_service was ready")


def check_cube() -> str:
  return _trials("red_cube")


def check_peg() -> str:
  return _trials("peg")


def check_too_wide() -> str:
  error = _plan_error("plate")
  if error.code() != grpc.StatusCode.ABORTED or "0.075" not in error.details():
    raise RuntimeError(f"PlanGrasp('plate') -> {cuj._code(error)}")
  return f"PlanGrasp('plate') -> {cuj._code(error)}"


def check_logs() -> str:
  rpc = [line for line in _log_lines("robot_service")[_log_start["robot_service"]:] if "rpc PlanGrasp " in line]
  # main.cpp's per-goal line ("goal <outcome> target=..."), not the accept handler's "Accepted ... goal ...".
  goals = [line for line in _log_lines("grasp_planner")[_log_start["grasp_planner"]:]
           if "]: goal " in line and " target=" in line]
  timed = [line for line in rpc if "status=" in line and "ms=" in line]
  if len(rpc) != _plan_calls[0] or len(timed) != len(rpc) or len(goals) != _goals[0]:
    raise RuntimeError(f"{len(rpc)} 'rpc PlanGrasp' lines ({len(timed)} with status and ms) for {_plan_calls[0]} calls;"
                       f" {len(goals)} 'goal ' lines in grasp_planner.log for {_goals[0]} goals")
  return (f"{len(rpc)} PlanGrasp lines for {_plan_calls[0]} calls, e.g. {rpc[-1][:200]!r}; {len(goals)} goal lines for"
          f" {_goals[0]} goals, e.g. {goals[-1][:200]!r}")


def _check_errors(run: hwm._Run) -> str:
  missing = _plan_error("nope")
  if missing.code() != grpc.StatusCode.NOT_FOUND:
    raise RuntimeError(f"PlanGrasp('nope') -> {cuj._code(missing)}")
  run.stop_container()
  hwm.CONTAINER_LOG = OFF_CONTAINER_LOG
  os.environ["GRASP_PLANNER"] = "off"
  try:
    run.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
  finally:
    del os.environ["GRASP_PLANNER"]
  off = _plan_error("red_cube")
  if off.code() != grpc.StatusCode.UNAVAILABLE or "grasp planner not running" not in off.details():
    raise RuntimeError(f"GRASP_PLANNER=off: PlanGrasp('red_cube') -> {cuj._code(off)}")
  return f"PlanGrasp('nope') -> {cuj._code(missing)}; GRASP_PLANNER=off: PlanGrasp('red_cube') -> {cuj._code(off)}"


def main() -> int:
  start = time.monotonic()
  harness.sync_isaac()
  hwm.CONTAINER_LOG = CONTAINER_LOG
  run = hwm._Run()
  results = []
  try:
    run.start_isaac(ISAAC_LOG)
    run.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    print(f"robot_service ready after {time.monotonic() - start:.1f} s", flush=True)
    results.append(hwm._run_check("planner_up", check_planner_up))
    _log_start.update({name: len(_log_lines(name)) for name in ("robot_service", "grasp_planner")})
    checks = (
        ("cube", check_cube),
        ("peg", check_peg),
        ("too_wide", check_too_wide),
        ("logs", check_logs),
        ("errors", lambda: _check_errors(run)),
    )
    results += [hwm._run_check(name, check) for name, check in checks]
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
