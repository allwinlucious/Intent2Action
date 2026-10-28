"""Phase 5b check: MoveToContact, MoveCompliant and CUJ 6 (phase_5b.md, done criteria).

Syncs the repository to Windows, starts sim/run_scene.py and the container
(ROBOT_BACKEND=isaac, container/run.sh), and prints PASS/FAIL per check, in order:
  .venv/bin/python tests/e2e/phase5b_check.py [--sweep]
--sweep runs only `jam` and 5 `insert` trials (need 4), one combination of the Addendum C.2 sweep.
ResetScene runs before every check and every insert trial. Stops the container and Isaac at the end.
"""

import re
import subprocess
import sys
import threading
import time

import grpc
from google.protobuf import duration_pb2
import numpy as np

import harness
import phase3_hwm_check as hwm  # sets sys.path for gen/ and ICON's Python
import phase1_check as p1
import phase3_cuj_check as cuj
import phase5c_check as p5c
from google.longrunning import operations_pb2  # noqa: E402
from robot_service import robot_service_pb2 as api  # noqa: E402

ISAAC_LOG = harness.LOG_DIR / "phase5b_isaac.log"
CONTAINER_LOG = harness.LOG_DIR / "phase5b_container.log"
DOWN = (0.0, 0.0, -1.0)
FORCE_N, SPEED = 15.0, 0.01
FORCE_RANGE_N = (12.0, 18.0)  # G8: +-20 %
TABLE_XY = (0.45, -0.05)
NO_CONTACT_Z = 0.30
FACE_OFF_AXIS_M = 0.02  # +x; contact_face and jam: the peg (radius 0.010) rests on the face beside the hole (radius 0.011)
JAM_CONTACT_FORCE_N = 10.0
JAM_DOWN_M = 0.03  # MoveCompliant target below the contact TCP pose
JAM_STIFFNESS, JAM_MAX_FORCE_N = 1000.0, 60.0  # spring 30 N plus overshoot (about 43 N) < 60
JAM_MAX_SIM_S = 10.0
# Sliding needs the lateral pull to beat friction (offset > mu * depth), so the target is only 2 mm down.
SEARCH_DOWN_M = 0.002  # MoveCompliant target below the contact TCP pose, on the hole axis
SEARCH_STIFFNESS, SEARCH_MAX_FORCE_N = 2000.0, 30.0  # about 4 N normal, 6 N lateral pull at 3 mm
OPERATION_TIMEOUT_S = 120.0  # wall; WaitOperation is repeated since the server caps one wait at 60 s
PEG, BLOCK = "peg", "hole_block"
BLOCK_TOP_Z = 0.05
LIFT_M = 0.10
ABOVE_M = 0.02  # peg bottom above the block top before MoveToContact
OFF_AXIS_M = 0.003  # +x
PLACE_MOVES = 3
PLACE_TOLERANCE_M = 0.0003
MIN_DEPTH_M = 0.025
TOP_FACE_TOLERANCE_M = 0.003
PAYLOAD_KG = 0.05
TRIALS, MIN_SUCCESSES = 10, 9
SWEEP_TRIALS, SWEEP_MIN_SUCCESSES = 5, 4  # Addendum C.2
FORCE_METHODS = ("MoveToContact", "MoveCompliant")
TIMESTAMP = re.compile(r"\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3} ")
TIMESTAMP_LEN = 23
_calls = {"rpc": 0, "op": 0}
_log_start = [""]  # timestamp of the last robot_service line before the checks
_id: dict[str, str] = {}  # prim -> object ID, refreshed by every _reset()


class _PeakForce:
  """GetWrench force norms while the block runs (polled every ~10 ms wall; may miss short spikes)."""

  def __enter__(self):
    self.norms, self._done = [], threading.Event()
    self._thread = threading.Thread(target=self._poll, daemon=True)
    self._thread.start()
    return self

  def _poll(self) -> None:
    while not self._done.wait(0.01):
      w = cuj._call(cuj._robot, "GetWrench")
      self.norms.append(float(np.linalg.norm([w.fx, w.fy, w.fz])))

  def __exit__(self, *_):
    self._done.set()
    self._thread.join()

  def summary(self) -> str:
    """Peak, and the 10th/90th percentiles of the second half (the two levels of an alternating wrench)."""
    if not self.norms:
      return "no GetWrench samples"
    low, high = np.percentile(self.norms[len(self.norms) // 2:], [10, 90])
    return f"peak GetWrench {max(self.norms):.1f} N, second half p10/p90 {low:.1f}/{high:.1f} N"


class _OperationError(grpc.RpcError):
  """A failed operation, with the code() and details() of the RpcError a wait=true call raises."""

  def __init__(self, code: grpc.StatusCode, message: str):
    super().__init__(message)
    self._code, self._message = code, message

  def code(self) -> grpc.StatusCode:
    return self._code

  def details(self) -> str:
    return self._message


def _wait(operation):
  """WaitOperation until `operation` is done (at most OPERATION_TIMEOUT_S wall); raises _OperationError if it failed."""
  deadline = time.monotonic() + OPERATION_TIMEOUT_S
  while not operation.done and (left := deadline - time.monotonic()) > 0:
    operation = cuj._call(cuj._robot, "WaitOperation", operations_pb2.WaitOperationRequest(
        name=operation.name, timeout=duration_pb2.Duration(seconds=int(left) + 1)))
  if operation.done and operation.HasField("error"):
    code = next(c for c in grpc.StatusCode if c.value[0] == operation.error.code)
    raise _OperationError(code, operation.error.message)
  return operation


def _force(method: str, **fields) -> str:
  """A force RPC (wait=false, then WaitOperation); prints status, message and peak force; returns the detail."""
  _calls["rpc"] += 1
  try:
    with _PeakForce() as peak:
      operation = _wait(cuj._call(cuj._robot, method, getattr(api, f"{method}Request")(wait=False, **fields)))
  except grpc.RpcError as error:
    # INVALID_ARGUMENT and FAILED_PRECONDITION are replies before any operation starts.
    _calls["op"] += error.code() not in (grpc.StatusCode.INVALID_ARGUMENT, grpc.StatusCode.FAILED_PRECONDITION)
    print(f"    {method} -> {cuj._code(error)}; {peak.summary()}", flush=True)
    raise
  _calls["op"] += 1
  cuj._done(operation, api.MotionResult)
  info = api.OperationInfo()
  operation.metadata.Unpack(info)
  print(f"    {method} -> OK {info.detail!r}; {peak.summary()}", flush=True)
  return info.detail


def _force_error(method: str, **fields) -> grpc.RpcError:
  try:
    detail = _force(method, **fields)
  except grpc.RpcError as error:
    return error
  raise RuntimeError(f"{method} returned OK: {detail}")


def _push_force(direction=DOWN) -> float:
  """|GetWrench force (flange frame) along `direction` (base frame)|."""
  w = cuj._call(cuj._robot, "GetWrench")
  flange = cuj._call(cuj._robot, "ComputeFk", api.ComputeFkRequest(joints=cuj._state().joints, tool=api.FLANGE)).pose
  return abs(float(np.dot([w.fx, w.fy, w.fz], cuj._matrix(flange)[:3, :3].T @ np.array(direction))))


def _true(prim: str) -> api.Pose:
  return cuj._call(cuj._sim, "GetTruePose", api.GetTruePoseRequest(frame=_id[prim]))


def _reset() -> None:
  cuj._call(cuj._sim, "ResetScene")
  _id.clear()
  _id.update(cuj.ids(cuj._sim))


def _sim_seconds(message: str) -> float:
  found = re.search(r"after ([0-9.]+) s of sim time", message)
  if not found:
    raise RuntimeError(f"no sim duration in {message!r}")
  return float(found.group(1))


def check_contact_face() -> str:
  """Addendum F.1: the grasped peg on the block face beside the hole (bare fingers on the table chatter in Isaac)."""
  _reset()
  hole, facts = _peg_above_block(FACE_OFF_AXIS_M)
  cuj._call(cuj._robot, "TareWrench")
  detail = _force("MoveToContact", dz=-1.0, max_force=FORCE_N, speed=SPEED)
  along, peg = _push_force(), _true(PEG)
  facts.append(f"MoveToContact(down, {FORCE_N:g} N, {SPEED:g} m/s) done; GetWrench along the push {along:.2f} N, "
               f"peg bottom z {peg.z:.4f} ({detail})")
  if not FORCE_RANGE_N[0] <= along <= FORCE_RANGE_N[1]:
    raise RuntimeError(f"force along the push outside {FORCE_RANGE_N}: {'; '.join(facts)}")
  return "; ".join(facts)


def check_no_contact() -> str:
  _reset()
  down = cuj._down()
  cuj._move("MoveToPose", pose=cuj._at(down, *TABLE_XY, NO_CONTACT_Z))
  error = _force_error("MoveToContact", dz=1.0, max_force=10.0, speed=0.02)
  if error.code() != grpc.StatusCode.ABORTED or "no contact within 0.15 m" not in error.details():
    raise RuntimeError(f"MoveToContact(up) -> {cuj._code(error)}")
  return f"MoveToContact(up, 10 N, 0.02 m/s) -> {cuj._code(error)}"


def check_bad_speed() -> str:
  _reset()
  error = _force_error("MoveToContact", dz=-1.0, max_force=FORCE_N, speed=0.2)
  if error.code() != grpc.StatusCode.INVALID_ARGUMENT or not re.search(r"between [0-9.]+ and [0-9.]+ m/s", error.details()):
    raise RuntimeError(f"MoveToContact(speed 0.2) -> {cuj._code(error)}")
  return f"MoveToContact(speed 0.2) -> {cuj._code(error)}"


def _peg_above_block(off_axis_m: float) -> tuple[api.Pose, list[str]]:
  """Grasps the peg and holds its bottom off_axis_m off the hole axis in +x, ABOVE_M above the top; returns (hole, facts)."""
  p5c._settled(_id[PEG])
  plan = p5c._plan(_id[PEG])
  cuj._move("MoveToPose", pose=plan.pregrasp)
  cuj._move("MoveToPose", pose=plan.grasp, motion=api.LINEAR)
  cuj._gripper("Grasp")
  cuj._call(cuj._robot, "AttachObject", api.ObjectRequest(id=_id[PEG]))
  cuj._call(cuj._robot, "SetPayload", api.SetPayloadRequest(mass=PAYLOAD_KG))
  cuj._move("MoveToPose", pose=cuj._at(plan.grasp, plan.grasp.x, plan.grasp.y, plan.grasp.z + LIFT_M), motion=api.LINEAR)
  hole = _true(BLOCK)
  # The peg's origin is its bottom centre. The arm lands within about 1 mm of a target, so the move is
  # repeated from the new true pose.
  for _ in range(PLACE_MOVES):
    peg, tcp = _true(PEG), cuj._state().tool_pose
    goal = np.array([hole.x + off_axis_m, hole.y, BLOCK_TOP_Z + ABOVE_M]) - [peg.x, peg.y, peg.z]
    if np.linalg.norm(goal) <= PLACE_TOLERANCE_M:
      break
    cuj._move("MoveToPose", pose=cuj._at(tcp, tcp.x + goal[0], tcp.y + goal[1], tcp.z + goal[2]), motion=api.LINEAR)
  peg = _true(PEG)
  return hole, [f"start off axis ({(peg.x - hole.x) * 1e3:+.2f}, {(peg.y - hole.y) * 1e3:+.2f}) mm, "
                f"{(peg.z - BLOCK_TOP_Z) * 1e3:.1f} mm above"]


def _insert_trial() -> str:
  _reset()
  hole, facts = _peg_above_block(OFF_AXIS_M)
  cuj._call(cuj._robot, "TareWrench")
  step = "MoveToContact (top face)"
  try:
    _force("MoveToContact", dz=-1.0, max_force=FORCE_N, speed=SPEED)
    along, contact, peg = _push_force(), cuj._state().tool_pose, _true(PEG)
    on_face = abs(peg.z - BLOCK_TOP_Z) <= TOP_FACE_TOLERANCE_M
    facts.append(f"contact {along:.2f} N, peg bottom z {peg.z:.4f}, x/y off axis "
                 f"({(peg.x - hole.x) * 1e3:+.2f}, {(peg.y - hole.y) * 1e3:+.2f}) mm")
    # Search: a stiff spring to the hole axis, slightly below the face, slides the peg over the hole.
    target = cuj._at(contact, contact.x + hole.x - peg.x, contact.y + hole.y - peg.y, contact.z - SEARCH_DOWN_M)
    step = "MoveCompliant (search)"
    _force("MoveCompliant", pose=target, stiffness=SEARCH_STIFFNESS, max_force=SEARCH_MAX_FORCE_N)
    peg = _true(PEG)
    facts.append(f"after MoveCompliant peg bottom z {peg.z:.4f}, x/y off axis "
                 f"({(peg.x - hole.x) * 1e3:+.2f}, {(peg.y - hole.y) * 1e3:+.2f}) mm")
    step = "MoveToContact (hole bottom)"
    _force("MoveToContact", dz=-1.0, max_force=FORCE_N, speed=SPEED)
    facts.append(f"bottom contact {_push_force():.2f} N")
  except grpc.RpcError as error:
    facts.append(f"{step} {cuj._code(error)}")
    failed = error
  else:
    failed = None
  depth, state = BLOCK_TOP_Z - _true(PEG).z, cuj._state()
  facts.append(f"depth {depth * 1e3:.1f} mm, faulted {state.faulted}")
  if failed is not None:
    raise RuntimeError("; ".join(facts)) from failed
  problems = []
  if not FORCE_RANGE_N[0] <= along <= FORCE_RANGE_N[1]:
    problems.append(f"contact force outside {FORCE_RANGE_N}")
  if not on_face:
    problems.append("contact not on the top face")
  if depth < MIN_DEPTH_M or state.faulted:
    problems.append(f"depth < {MIN_DEPTH_M * 1e3:.0f} mm or faulted")
  if problems:
    raise RuntimeError(f"{', '.join(problems)}: {'; '.join(facts)}")
  return "; ".join(facts)


def check_insert(trials: int = TRIALS, min_successes: int = MIN_SUCCESSES) -> str:
  successes = 0
  for i in range(trials):
    try:
      detail = _insert_trial()
      successes += 1
      print(f"  insert trial {i + 1}: ok, {detail}", flush=True)
    except (grpc.RpcError, RuntimeError) as error:
      print(f"  insert trial {i + 1}: FAILED {p1._describe(error)}", flush=True)
  if successes < min_successes:
    raise RuntimeError(f"{successes}/{trials} inserted (need {min_successes})")
  return f"{successes}/{trials} inserted >= {MIN_DEPTH_M * 1e3:.0f} mm"


def check_jam() -> str:
  _reset()
  hole, facts = _peg_above_block(FACE_OFF_AXIS_M)
  cuj._call(cuj._robot, "TareWrench")
  _force("MoveToContact", dz=-1.0, max_force=JAM_CONTACT_FORCE_N, speed=SPEED)
  contact, peg = cuj._state().tool_pose, _true(PEG)
  facts.append(f"contact {_push_force():.2f} N, peg bottom z {peg.z:.4f}, x/y off axis "
               f"({(peg.x - hole.x) * 1e3:+.2f}, {(peg.y - hole.y) * 1e3:+.2f}) mm")
  error = _force_error("MoveCompliant", pose=cuj._at(contact, contact.x, contact.y, contact.z - JAM_DOWN_M),
                       stiffness=JAM_STIFFNESS, max_force=JAM_MAX_FORCE_N)
  facts.append(f"MoveCompliant into the block face -> {cuj._code(error)}")
  took = _sim_seconds(error.details()) if error.code() == grpc.StatusCode.ABORTED else None
  if took is None or "settled before reaching the target" not in error.details() or took > JAM_MAX_SIM_S:
    raise RuntimeError("; ".join(facts))
  return "; ".join(facts)


def _log_lines() -> list[str]:
  """robot_service lines from the current and rotated files (the GetWrench sampler rotates it at 10 MB)."""
  out = subprocess.run(harness.docker(f"docker exec intrinsic sh -c 'cat {p5c.LOGS}/robot_service.log.* "
                                      f"{p5c.LOGS}/robot_service.log 2>/dev/null'"),
                       capture_output=True, text=True, errors="replace").stdout
  return sorted(line for line in out.splitlines() if TIMESTAMP.match(line))


def check_logs() -> str:
  lines = [line for line in _log_lines() if line[:TIMESTAMP_LEN] > _log_start[0]]
  rpc = [line for line in lines if any(f"rpc {m} " in line for m in FORCE_METHODS) and "status=" in line and "ms=" in line]
  forced = [line for line in lines if re.search(r"force Move(ToContact|Compliant) .* status=\w+ duration_s=[0-9.]+ "
                                                r"force_along_push_n=([0-9.]+|nan)", line)]
  if len(rpc) != _calls["rpc"] or len(forced) != _calls["op"]:
    raise RuntimeError(f"{len(rpc)} rpc lines for {_calls['rpc']} force calls; {len(forced)} force lines for "
                       f"{_calls['op']} force operations")
  return (f"{len(rpc)} rpc lines for {_calls['rpc']} calls; {len(forced)} force lines for {_calls['op']} operations, "
          f"e.g. {forced[-1][:220]!r}")


def main() -> int:
  sweep = "--sweep" in sys.argv[1:]
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
    _log_start[0] = _log_lines()[-1][:TIMESTAMP_LEN]
    checks = ((
        ("jam", check_jam),
        ("insert", lambda: check_insert(SWEEP_TRIALS, SWEEP_MIN_SUCCESSES)),
    ) if sweep else (
        ("contact_face", check_contact_face),
        ("no_contact", check_no_contact),
        ("bad_speed", check_bad_speed),
        ("insert", check_insert),
        ("jam", check_jam),
        ("logs", check_logs),
    ))
    results = [hwm._run_check(name, check) for name, check in checks]
  except Exception as error:  # pylint: disable=broad-except
    print(f"FAIL setup: {p1._describe(error)}", flush=True)
    results = [False]
  finally:
    subprocess.run(harness.docker(f"docker cp intrinsic:/var/lib/intrinsic/logs/robot_service.log "
                                  f"{harness.LOG_DIR}/phase5b_robot_service.log"), capture_output=True)
    run.stop_container()
    run.stop_isaac()
  print(f"elapsed {time.monotonic() - start:.1f}")
  return 0 if results and all(results) else 1


if __name__ == "__main__":
  sys.exit(main())
