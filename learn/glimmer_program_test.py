"""Zero-shot program writing by an OpenAI-compatible model (Glimmer on Modal), on fake scenes; no sim, no GPU.

  set -a; source .env; set +a; .venv/bin/python learn/glimmer_program_test.py [--out logs/glimmer/zero_shot.jsonl]

For held-out intents (not among the prompt's examples) x a few random fake scenes: valid on the first try (check +
dry run), valid after the one retry, and whether running it on the fake robot achieves the intent (rule checker).
"""

import argparse
import json
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from program_runtime.agent import ChatClient, ProgramWriter  # noqa: E402
from program_runtime.api import FakeBackend, Obj, RobotAPI  # noqa: E402
from program_runtime.rules import TrueObj, check_intent, validate_intent  # noqa: E402
from program_runtime.runtime import run  # noqa: E402

CUBES = {"red": 0.05, "blue": 0.05, "green": 0.05, "yellow": 0.035}
BOXES = {"red": (0.38, -0.24), "blue": (0.66, -0.20)}
INTENTS = [
    {"description": "only the blue cube goes into the red box",
     "rules": [{"select": {"category": "cube", "color": "blue"}, "relation": "in", "target": {"category": "box", "color": "red"}}],
     "others": "stay"},
    {"description": "small cubes into the blue box, everything else stays",
     "rules": [{"select": {"category": "cube", "size": "small"}, "relation": "in", "target": {"category": "box", "color": "blue"}}],
     "others": "stay"},
    {"description": "the green cube goes on top of the blue cube",
     "rules": [{"select": {"category": "cube", "color": "green"}, "relation": "on", "target": {"category": "cube", "color": "blue"}}],
     "others": "stay"},
    {"description": "sort by size: small cubes into the blue box, medium cubes into the red box",
     "rules": [{"select": {"category": "cube", "size": "small"}, "relation": "in", "target": {"category": "box", "color": "blue"}},
               {"select": {"category": "cube", "size": "medium"}, "relation": "in", "target": {"category": "box", "color": "red"}}],
     "others": "stay"},
    {"description": "every cube that is not red goes into the blue box",
     "rules": [{"select": {"category": "cube", "color": c}, "relation": "in", "target": {"category": "box", "color": "blue"}}
               for c in ("blue", "green", "yellow")],
     "others": "stay"},
]


def scene(seed: int) -> list[Obj]:
  rng = random.Random(seed)
  objects, placed = [], []
  for i, (color, size) in enumerate(CUBES.items()):
    while True:
      x, y = rng.uniform(0.35, 0.60), rng.uniform(0.0, 0.2)
      if all((x - a) ** 2 + (y - b) ** 2 >= 0.08 ** 2 for a, b in placed):
        placed.append((x, y))
        break
    objects.append(Obj(id=f"obj_{rng.randint(10, 99)}{i}", category="cube", color=color,
                       size="small" if size < 0.04 else "medium", x=x, y=y, z=0.0, yaw=0.0,
                       sx=size, sy=size, sz=size, attached=False))
  for color, (x, y) in BOXES.items():
    objects.append(Obj(id=f"obj_{rng.randint(10, 99)}b", category="box", color=color, size="large", x=x, y=y, z=0.0,
                       yaw=0.0, sx=0.25, sy=0.21, sz=0.05, attached=False))
  return objects


def truth(objects: list[Obj]) -> dict[str, TrueObj]:
  return {o.id: TrueObj(id=o.id, category=o.category, color=o.color, size=o.size, x=o.x, y=o.y, z=o.z, yaw=o.yaw,
                        sx=o.sx, sy=o.sy, sz=o.sz) for o in objects}


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--scenes", type=int, default=3)
  parser.add_argument("--out", default=str(ROOT / "logs/glimmer/zero_shot.jsonl"))
  args = parser.parse_args()
  Path(args.out).parent.mkdir(parents=True, exist_ok=True)
  writer = ProgramWriter(ChatClient())
  totals = {"runs": 0, "first_try_valid": 0, "valid": 0, "intent_achieved": 0}
  with open(args.out, "w") as out:
    for n, intent in enumerate(INTENTS):
      assert not validate_intent(intent), validate_intent(intent)
      for seed in range(args.scenes):
        objects = scene(100 * n + seed)
        start = time.monotonic()
        program = writer.write(intent, objects)
        seconds = time.monotonic() - start
        backend = FakeBackend(objects)
        result = run(program, RobotAPI(backend))
        rules = check_intent(intent, truth(objects), truth(backend.list_objects()))
        valid = not str(result.error or "").startswith(("check", "dry run"))
        totals["runs"] += 1
        totals["first_try_valid"] += valid and writer.last_attempts == 1
        totals["valid"] += valid
        totals["intent_achieved"] += result.ok and rules.ok
        record = {"intent": intent["description"], "seed": seed, "attempts": writer.last_attempts, "seconds": round(seconds, 1),
                  "ok": result.ok, "error": result.error, "rules": rules.summary, "program": program}
        out.write(json.dumps(record) + "\n")
        out.flush()
        print(f"{'OK  ' if result.ok and rules.ok else 'FAIL'} {intent['description'][:55]:55s} seed {seed} "
              f"attempts {writer.last_attempts} {seconds:4.0f}s  {rules.summary if result.ok else result.error}", flush=True)
  print(json.dumps(totals))


if __name__ == "__main__":
  main()
