"""Physics check for pushing: a carton wider than the gripper opens is pushed into a painted zone, closed loop.

  .venv/bin/python tests/e2e/push_check.py [--gui]

Each round: read the carton's and the zone's true poses, put the open fingers (spread across the push face) just
behind the carton, push it with a force-limited compliant move by the remaining distance, back off, re-read. Up
to ROUNDS rounds; passes when the carton's centre is within TOLERANCE_M of the zone's centre and it rests on it.
"""

import math
import sys
import time

import harness
import phase1_check as p1
import phase3_cuj_check as cuj
import phase3_hwm_check as hwm  # sets sys.path for gen/

sys.path.insert(0, str(harness.ROOT / "learn"))
import families  # noqa: E402
from robot_service import robot_service_pb2 as api  # noqa: E402

ISAAC_LOG = harness.LOG_DIR / "push_check_isaac.log"
CONTAINER_LOG = harness.LOG_DIR / "push_check_container.log"
CARTON = families.Placement("big_carton", 0.50, 0.22, 0.0, 0.0)
ZONE = families.Placement("target_zone", 0.50, -0.15, -0.0019, 0.0)  # flush with the table top
HALF_CARTON_M = 0.06
GAP_M = 0.025  # fingers behind the carton face before a push
PUSH_Z = 0.035  # TCP (fingertip centre) height: about the carton's lower half
ABOVE_Z = 0.15
# A compliant move is a spring to its target: steps of STEP_M at STIFFNESS start at about 15 N (sliding the carton
# needs about 1.2 N), so the 25 N limit only trips on a real jam.
STIFFNESS, MAX_FORCE_N, STEP_M = 300.0, 25.0, 0.05
ROUNDS = 4
TOLERANCE_M = 0.02


def _yawed(base: api.Pose, yaw: float) -> api.Pose:
  """`base` rotated by `yaw` about the world z axis (q_z * q_base)."""
  cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
  w, x, y, z = base.qw, base.qx, base.qy, base.qz
  return api.Pose(x=base.x, y=base.y, z=base.z, qw=cz * w - sz * z, qx=cz * x - sz * y, qy=cz * y + sz * x,
                  qz=cz * z + sz * w)


def _true(catalog: str) -> api.Pose:
  for o in cuj._call(cuj._sim, "ListTrueObjects").objects:
    if o.catalog == catalog:
      return o.pose
  raise RuntimeError(f"{catalog} not in the scene")


def _yaw(p: api.Pose) -> float:
  return math.degrees(math.atan2(2 * (p.qw * p.qz + p.qx * p.qy), 1 - 2 * (p.qy ** 2 + p.qz ** 2)))


def push_round(down: api.Pose) -> float:
  carton, zone = _true("big_carton"), _true("target_zone")
  ex, ey = zone.x - carton.x, zone.y - carton.y
  distance = math.hypot(ex, ey)
  print(f"  carton ({carton.x:.3f}, {carton.y:.3f}, z {carton.z:.3f}, yaw {_yaw(carton):.0f} deg) "
        f"-> zone ({zone.x:.3f}, {zone.y:.3f}): {distance * 1000:.0f} mm", flush=True)
  if distance <= TOLERANCE_M:
    return distance
  dx, dy = ex / distance, ey / distance
  hand = _yawed(down, math.atan2(dy, dx))  # fingers (home: along y) turned across the push direction
  back = HALF_CARTON_M + GAP_M
  start_x, start_y = carton.x - dx * back, carton.y - dy * back
  cuj._gripper("Release")
  cuj._move("MoveToPose", pose=cuj._at(hand, start_x, start_y, ABOVE_Z))
  cuj._move("MoveToPose", pose=cuj._at(hand, start_x, start_y, PUSH_Z), motion=api.LINEAR)
  goal_x, goal_y = zone.x - dx * back, zone.y - dy * back  # fingers here leave the carton centred on the zone
  t0, steps, outcome = time.monotonic(), 0, "reached"
  while True:
    tcp = cuj._state().tool_pose
    rx, ry = goal_x - tcp.x, goal_y - tcp.y
    left = rx * dx + ry * dy
    if left <= 0.005:
      break
    step = min(STEP_M, left)
    steps += 1
    if not _compliant(cuj._at(hand, tcp.x + dx * step, tcp.y + dy * step, PUSH_Z)):
      outcome = "force limit (jam)"
      break
  print(f"  pushed in {steps} steps of <= {STEP_M * 1000:.0f} mm, {MAX_FORCE_N:.0f} N limit: {outcome} "
        f"({time.monotonic() - t0:.1f} s)", flush=True)
  # Back off 5 cm (two gentle steps) so the planner's start state is clear of the carton, then lift.
  for _ in range(2):
    tcp = cuj._state().tool_pose
    _compliant(cuj._at(hand, tcp.x - dx * 0.025, tcp.y - dy * 0.025, tcp.z + 0.01))
  tcp = cuj._state().tool_pose
  carton_now = _true("big_carton")
  seen = {o.id: (o.category, round(o.pose.x, 3), round(o.pose.y, 3), round(o.pose.z, 3)) for o in cuj._call(cuj._robot, "ListObjects").objects}
  print(f"  after back-off: tcp ({tcp.x:.3f}, {tcp.y:.3f}, {tcp.z:.3f}); carton true ({carton_now.x:.3f}, "
        f"{carton_now.y:.3f}); world model {seen}", flush=True)
  # Straight up in gentle steps (free space; no planner start-state check). A spring settles short of its target,
  # so aim each step 4 cm above the current height and stop when high enough or no longer rising.
  for _ in range(5):
    if tcp.z >= ABOVE_Z - 0.02:
      break
    _compliant(cuj._at(hand, tcp.x, tcp.y, tcp.z + 0.04))
    before, tcp = tcp.z, cuj._state().tool_pose
    if tcp.z - before < 0.005:
      break
  time.sleep(1.0)
  return distance


def _compliant(pose: api.Pose) -> bool:
  """One compliant step; False only when the force limit stopped it (settling a few mm short is normal)."""
  try:
    cuj._move("MoveCompliant", pose=pose, stiffness=STIFFNESS, max_force=MAX_FORCE_N)
  except Exception as error:  # pylint: disable=broad-except
    if "settled before reaching" in str(error):
      return True
    if "force limit" in str(error):
      return False
    raise
  return True


def main() -> int:
  if "--gui" in sys.argv:
    harness.ISAAC_COMMAND[-1] += " --gui"
    harness.READY_TIMEOUT_S = 900
  harness.sync_isaac()
  hwm.CONTAINER_LOG = CONTAINER_LOG
  stack = hwm._Run()
  ok = False
  try:
    stack.start_isaac(ISAAC_LOG)
    stack.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    families.load_placements([CARTON, ZONE], cuj._sim, cuj._robot)
    down = cuj._down()
    for number in range(1, ROUNDS + 1):
      print(f"round {number}", flush=True)
      if push_round(down) <= TOLERANCE_M:
        ok = True
        break
    else:
      carton, zone = _true("big_carton"), _true("target_zone")
      ok = math.hypot(zone.x - carton.x, zone.y - carton.y) <= TOLERANCE_M
    carton = _true("big_carton")
    print(f"{'PASS' if ok else 'FAIL'} push: carton at ({carton.x:.3f}, {carton.y:.3f}, z {carton.z:.3f}, "
          f"yaw {_yaw(carton):.0f} deg)", flush=True)
    if "--keep" in sys.argv:
      input("stack kept running; press Enter to stop ")
  except Exception as error:  # pylint: disable=broad-except
    print(f"FAIL setup: {p1._describe(error)}", flush=True)
  finally:
    stack.stop_container()
    stack.stop_isaac()
  return 0 if ok else 1


if __name__ == "__main__":
  sys.exit(main())
