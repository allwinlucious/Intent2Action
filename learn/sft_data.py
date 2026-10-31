"""Training rows for Glimmer SFT in the cookbook's Axolotl format: one JSON object per line with `messages`.

  .venv/bin/python learn/sft_data.py smoke --out logs/sft/smoke
      program-writing rows from the example programs on random fake scenes: exercises the training path only

Prompts come from program_runtime.agent.program_messages, the writer's own prompt, so training and use match.
"""

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "learn")]

from glimmer_program_test import scene  # noqa: E402
from program_runtime.agent import program_messages  # noqa: E402

EXAMPLES = ROOT / "program_runtime" / "examples"


def program_row(intent: dict, objects, program: str) -> dict:
  return {"messages": [*program_messages(intent, objects), {"role": "assistant", "content": program}]}


def smoke(out: Path, train: int = 24, validation: int = 4) -> None:
  names = sorted(p.stem for p in EXAMPLES.glob("*.py"))
  rows = []
  for seed in range(train + validation):
    name = names[seed % len(names)]
    intent = json.loads((EXAMPLES / f"{name}.json").read_text())
    rows.append(program_row(intent, scene(1000 + seed), (EXAMPLES / f"{name}.py").read_text()))
  out.mkdir(parents=True, exist_ok=True)
  for split, part in (("train", rows[:train]), ("validation", rows[train:])):
    (out / f"{split}.jsonl").write_text("".join(json.dumps(r) + "\n" for r in part))
  print(f"{out}: {train} train, {validation} validation rows")


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("kind", choices=["smoke"])
  parser.add_argument("--out", default=str(ROOT / "logs/sft/smoke"))
  args = parser.parse_args()
  smoke(Path(args.out))
