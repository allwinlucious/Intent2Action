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
  # Let go of the peg: a held, attached object blocks the next scene load.
  from robot_service import robot_service_pb2 as api
  cuj._gripper("Release")
  cuj._call(cuj._robot, "DetachObject", api.ObjectRequest(id=p5b._id[p5b.PEG]))
  cuj._call(cuj._robot, "SetPayload", api.SetPayloadRequest(mass=0.0))
  tcp = cuj._state().tool_pose
  cuj._move("MoveToPose", pose=cuj._at(tcp, tcp.x, tcp.y, tcp.z + 0.1), motion=api.LINEAR)
  cuj._move("Home")
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


def _model_take(scene: str):
  def take():
    placements, _ = record_take._task(scene)
    program = (ROOT / f"demo_runs/programs/{scene.replace(':', '_')}.G1d.py").read_text()
    return _program(placements, program)
  return take


TAKES = {"peg_in_hole": peg_in_hole, "push": push, "gift_tower": gift_tower,
         **{name: _model_take(scene) for name, scene in MODEL_PROGRAMS.items()}}


def main(names) -> None:
  GRID.mkdir(parents=True, exist_ok=True)
  index_path = GRID / "index.json"
  for name in names:
    first, result = TAKES[name]()
    time.sleep(record_take.SETTLE_S)
    frames = record_take.encode(first, record_take._newest_index(), GRID / f"{name}.mp4")
    index = json.loads(index_path.read_text()) if index_path.exists() else {}  # shared with record_successes.py
    index[name] = {"ok": result.startswith("OK"), "result": result, "scripted": name not in MODEL_PROGRAMS}
    index_path.write_text(json.dumps(index, indent=1))
    print(f"{name}: {result[:160]} ({frames / record_take.FPS:.1f} s)", flush=True)


if __name__ == "__main__":
  main(sys.argv[1:] or list(TAKES))
