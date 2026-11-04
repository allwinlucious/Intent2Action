"""Records a scripted push in Isaac: the carton wider than the gripper onto the painted zone, with the robot's own
force-limited moves (the stack must be up: tests/e2e/demo_stack.py). Not a model's program: the push-experience
expert's sequence (learn/push_experience.py push_turns) run through the same robot tools a model would call.

  .venv/bin/python tools/record_push.py --name finale_push_scripted

Writes C:\\isaac-sim-work\\videos\\<name>.mp4 and demo_runs/<name>.events.jsonl (every call and its result).
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen"), str(ROOT / "learn"), str(ROOT / "tests/e2e"), str(ROOT / "tools")]

import record_take  # noqa: E402

PUSH_Z, ABOVE_Z, GAP_M, FINGER_M = 0.035, 0.15, 0.025, 0.01
CONTACT_N, CONTACT_SPEED, PUSH_LIMIT_N, TARGET_N = 10.0, 0.01, 25.0, 15.0


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--name", default="finale_push_scripted")
  args = parser.parse_args()
  import families
  import phase3_cuj_check as cuj
  from mcp_server import server
  from mcp_server.server import DESCRIPTOR_SET, Tools
  from push_experience import _yawed
  placements, _ = record_take._task("carton:0")
  families.load_placements(placements, cuj._sim, cuj._robot)
  cuj._move("Home")
  time.sleep(record_take.SETTLE_S)
  tools, events = Tools(DESCRIPTOR_SET), (record_take.PROOF / f"{args.name}.events.jsonl").open("w")

  def call(name: str, arguments: dict) -> dict:
    result, status = server.invoke(tools, cuj._robot, name, arguments)
    text = "\n".join(item.text for item in result.content if hasattr(item, "text"))
    events.write(json.dumps({"tool": name, "arguments": arguments, "status": status, "result": text[:600]}) + "\n")
    events.flush()
    print(f"{name}: {status} {text[:140]}", flush=True)
    value = json.loads(text) if text.startswith("{") else {}
    while value.get("name", "").startswith("operations/") and not value.get("done", True):
      value = call("wait_operation", {"name": value["name"], "timeout": "30s"})  # a long move outlasts one call
    return value

  def true(category: str):
    return next(o.pose for o in cuj._call(cuj._sim, "ListTrueObjects").objects
                if o.catalog == {"carton": "big_carton", "zone": "target_zone"}[category])

  first = record_take._newest_index() + 1
  carton, zone = true("carton"), true("zone")
  ex, ey = zone.x - carton.x, zone.y - carton.y
  distance = math.hypot(ex, ey)
  dx, dy = ex / distance, ey / distance
  down = call("get_state", {})["tool_pose"]
  hand = _yawed(down, math.atan2(dy, dx))  # the fingers across the push direction
  back = 0.06 + FINGER_M + GAP_M

  def pose(x, y, z):
    return {"x": x, "y": y, "z": z, **hand}

  start = (carton.x - dx * back, carton.y - dy * back)
  call("release", {})
  call("move_to_pose", {"pose": pose(*start, ABOVE_Z), "motion": "ANY", "tool": "TCP"})
  call("move_to_pose", {"pose": pose(*start, PUSH_Z), "motion": "LINEAR", "tool": "TCP"})
  call("move_to_contact", {"dx": dx, "dy": dy, "dz": 0.0, "max_force": CONTACT_N, "speed": CONTACT_SPEED})
  for _ in range(4):
    carton = true("carton")
    left = math.hypot(zone.x - carton.x, zone.y - carton.y)
    print(f"carton {left * 1000:.0f} mm from the zone centre", flush=True)
    if left <= 0.01:
      break
    tcp = call("get_state", {})["tool_pose"]
    stiffness = max(50.0, min(300.0, round(TARGET_N / left / 10) * 10))
    call("move_compliant", {"pose": pose(tcp["x"] + dx * left, tcp["y"] + dy * left, PUSH_Z),
                            "stiffness": stiffness, "max_force": PUSH_LIMIT_N})
  tcp = call("get_state", {})["tool_pose"]
  call("move_compliant", {"pose": pose(tcp["x"] - dx * 0.03, tcp["y"] - dy * 0.03, PUSH_Z + 0.01), "stiffness": 300.0,
                          "max_force": PUSH_LIMIT_N})
  tcp = call("get_state", {})["tool_pose"]
  call("move_compliant", {"pose": pose(tcp["x"], tcp["y"], tcp["z"] + 0.04), "stiffness": 300.0, "max_force": PUSH_LIMIT_N})
  carton = true("carton")
  off = math.hypot(zone.x - carton.x, zone.y - carton.y)
  events.write(json.dumps({"result": f"carton {off * 1000:.1f} mm from the zone centre"}) + "\n")
  events.close()
  time.sleep(record_take.SETTLE_S)
  out = record_take.VIDEOS / f"{args.name}.mp4"
  frames = record_take.encode(first, record_take._newest_index(), out)
  print(f"carton {off * 1000:.1f} mm from the zone centre; {out}: {frames / record_take.FPS:.1f} s", flush=True)


if __name__ == "__main__":
  main()
