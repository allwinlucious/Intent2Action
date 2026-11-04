"""Records visually different takes for the video's grid (the stack must be up: tests/e2e/demo_stack.py).

  .venv/bin/python tools/record_scripted.py [take ...]        (default: all, in TAKES order)

Takes (C:\\isaac-sim-work\\videos\\grid\\<take>.mp4, 30 fps real speed; robot events are not logged for the
scripted ones):
  peg_in_hole  the force-control check of tests/e2e/phase5b_check.py: guarded contact, compliant search, insert
  push         tests/e2e/push_check.py: an over-wide carton pushed into the painted zone with force-limited moves
  gift_tower   the owner's three gift boxes stacked: green, then red, then white (a scripted program)
  sort_by_color, cubes_in_box, yellow_on_red   the fine-tuned model's programs from demo_runs/programs/
Scripted takes show the robot stack's capabilities; only the demo_runs programs were written by a model.
"""

import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen"), str(ROOT / "learn"), str(ROOT / "tests/e2e"), str(ROOT / "tools")]

import record_take  # noqa: E402

GRID = record_take.VIDEOS / "grid"
FAIL_GRID = record_take.VIDEOS / "grid_fail"
GIFTS = (("green_gift", 0.42, 0.12, 0.2), ("red_gift", 0.58, 0.16, -0.3), ("white_gift", 0.52, -0.12, 0.4))
GIFT_TOWER = ('base = find(category="gift", color="green")\nred = find(category="gift", color="red")\n'
              'white = find(category="gift", color="white")\npick(red)\nplace(red, base)\npick(white)\n'
              'place(white, red)\nverify(lambda: on_top(red, base) and on_top(white, red), "the tower is not built")\n')
MODEL_PROGRAMS = {"sort_by_color": "sort_by_color:0", "cubes_in_box": "cubes_in_box:0", "yellow_on_red": "yellow_on_red:0"}


def _program(placements, program: str) -> str:
  import families
  import phase3_cuj_check as cuj
  from program_runtime.api import GrpcBackend, RobotAPI
  from program_runtime.runtime import run
  families.load_placements(placements, cuj._sim, cuj._robot)
  cuj._move("Home")
  time.sleep(record_take.SETTLE_S)
  first = record_take._newest_index() + 1
  result = run(program, RobotAPI(GrpcBackend("127.0.0.1:18100")))
  return first, "OK" if result.ok else f"FAILED: {str(result.error)[:200]}"


def peg_in_hole():
  import phase5b_check as p5b
  import phase3_cuj_check as cuj
  cuj._move("Home")
  first = record_take._newest_index() + 1
  try:
    result = f"OK: {p5b._insert_trial()[:200]}"
  except Exception as error:  # pylint: disable=broad-except
    result = f"FAILED: {str(error)[:200]}"
  _release_peg()  # a held, attached object blocks the next scene load
  return first, result


def push():
  import families
  import phase3_cuj_check as cuj
  import push_check as pc
  families.load_placements([pc.CARTON, pc.ZONE], cuj._sim, cuj._robot)
  cuj._move("Home")
  time.sleep(record_take.SETTLE_S)
  first = record_take._newest_index() + 1
  down = cuj._down()
  for _ in range(pc.ROUNDS):
    if pc.push_round(down) <= pc.TOLERANCE_M:
      return first, "OK"
  carton, zone = pc._true("big_carton"), pc._true("target_zone")
  ok = math.hypot(zone.x - carton.x, zone.y - carton.y) <= pc.TOLERANCE_M
  return first, "OK" if ok else "FAILED: carton not on the zone"


def gift_tower():
  import families
  return _program([families.Placement(c, x, y, 0.0, yaw) for c, x, y, yaw in GIFTS], GIFT_TOWER)


def _release_peg():
  import phase3_cuj_check as cuj
  import phase5b_check as p5b
  from robot_service import robot_service_pb2 as api
  cuj._gripper("Release")
  cuj._call(cuj._robot, "DetachObject", api.ObjectRequest(id=p5b._id[p5b.PEG]))
  cuj._call(cuj._robot, "SetPayload", api.SetPayloadRequest(mass=0.0))
  # After a jam the peg still touches the block and the fingers, so no lift can be planned: take it away first.
  cuj._call(cuj._sim, "RemoveObject", api.ObjectRequest(id=p5b._id[p5b.PEG]))
  time.sleep(1.0)
  cuj._move("Home")


def fail_peg_jam():
  """The peg 2 cm off the hole: it lands on the block face, and the force limit stops the push (a safe failure)."""
  import phase5b_check as p5b
  import phase3_cuj_check as cuj
  cuj._move("Home")
  first = record_take._newest_index() + 1
  try:
    result = f"FAILED as scripted: {p5b.check_jam()[:200]}"
  except Exception as error:  # pylint: disable=broad-except
    result = f"FAILED as scripted (unexpected detail): {str(error)[:200]}"
  _release_peg()
  return first, result


def fail_box_full():
  """Eight cubes for a box that takes six: it packs six, then finds no free spot."""
  import families
  spots = [(0.34 + 0.10 * (i % 4), 0.06 + 0.11 * (i // 4)) for i in range(8)]
  cubes = [families.Placement(("red_cube", "blue_cube", "green_cube")[i % 3], x, y, 0.0, 0.1 * i)
           for i, (x, y) in enumerate(spots)]
  first, result = _program([*cubes, families.Placement("red_box", 0.42, -0.22, 0.0, 0.0)],
                           'box = find(category="box", color="red")\nfor cube in find_all(category="cube"):\n'
                           '  pick(cube)\n  place(cube, box)\n'
                           'verify(lambda: all(inside(c, box) for c in find_all(category="cube")), "a cube is not in the box")\n')
  return first, f"FAILED as scripted: {result}" if result != "OK" else "OK (did not fail)"


def _model_take(scene: str):
  def take():
    placements, _ = record_take._task(scene)
    program = (ROOT / f"demo_runs/programs/{scene.replace(':', '_')}.G1d.py").read_text()
    return _program(placements, program)
  return take


TAKES = {"peg_in_hole": peg_in_hole, "push": push, "gift_tower": gift_tower,
         **{name: _model_take(scene) for name, scene in MODEL_PROGRAMS.items()}}
FAILURES = {"fail_peg_jam": fail_peg_jam, "fail_box_full": fail_box_full}  # recorded into grid_fail/


def main(names) -> None:
  for name in names:
    failure = name in FAILURES
    grid = FAIL_GRID if failure else GRID
    grid.mkdir(parents=True, exist_ok=True)
    index_path = grid / "index.json"
    first, result = (FAILURES if failure else TAKES)[name]()
    time.sleep(record_take.SETTLE_S)
    frames = record_take.encode(first, record_take._newest_index(), grid / f"{name}.mp4")
    index = json.loads(index_path.read_text()) if index_path.exists() else {}  # shared with record_successes.py
    # A failure take belongs in its grid when it failed (ok here means "usable for its grid").
    index[name] = {"ok": result.startswith("FAILED as scripted") if failure else result.startswith("OK"),
                   "result": result, "scripted": name not in MODEL_PROGRAMS}
    index_path.write_text(json.dumps(index, indent=1))
    print(f"{name}: {result[:160]} ({frames / record_take.FPS:.1f} s)", flush=True)


if __name__ == "__main__":
  main(sys.argv[1:] or list(TAKES))
