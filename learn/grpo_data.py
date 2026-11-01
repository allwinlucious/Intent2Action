"""GRPO prompts and verified rewards from the expert's all-strategies runs.

  .venv/bin/python learn/grpo_data.py --solve logs/expert/train_all_sorting.jsonl --out logs/grpo/data
  .venv-ml/bin/python learn/grpo_data.py render --data logs/grpo/data

Prompts: train tasks where the strategies disagree (some succeed, some fail), so a group of samples carries a
signal. Reward table per task: normalized strategy program -> 1.0 if it succeeded on that task in the plan tier,
else 0.0. G1b writes exact strategy programs (250/250 held-out), so a sample's reward is a looked-up, verified
outcome; a program matching no strategy gets 0 and is counted (learn/grpo_train.py logs the share).
"""

import argparse
import json
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen"), str(ROOT / "learn")]


def normalize(program: str) -> str:
  return "\n".join(line.rstrip() for line in program.strip().splitlines())


def build(solve_path, out_dir: Path) -> None:
  from program_runtime.agent import program_messages
  from program_runtime.api import Obj
  rows = []
  for line in Path(solve_path).read_text().splitlines():
    value = json.loads(line)
    if value.get("split") != "train" or "objects" not in value:
      continue
    # Strategies can produce the same program (e.g. on a tower), with different sampled outcomes: their mean.
    runs = {}
    for e in value["episodes"]:
      runs.setdefault(normalize(e["program"]), []).append(float(e["ok"]))
    outcomes = {program: sum(oks) / len(oks) for program, oks in runs.items()}
    if len(set(outcomes.values())) < 2:
      continue
    rows.append({"key": f"{value['family']}:{value['seed']}", "family": value["family"],
                 "messages": program_messages(value["intent"], [Obj(**o) for o in value["objects"]]),
                 "rewards": outcomes})
  random.Random(0).shuffle(rows)
  out_dir.mkdir(parents=True, exist_ok=True)
  (out_dir / "prompts.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
  by_family = {}
  for row in rows:
    by_family[row["family"]] = by_family.get(row["family"], 0) + 1
  print(f"{out_dir}: {len(rows)} prompts where strategies disagree {by_family}")


def render(data: Path) -> None:
  import transformers
  tokenizer = transformers.AutoTokenizer.from_pretrained("meta-models/Muse-Glimmer-30B")
  rows = []
  for line in (data / "prompts.jsonl").read_text().splitlines():
    row = json.loads(line)
    prompt = tokenizer.apply_chat_template(row["messages"], add_generation_prompt=True, tokenize=False,
                                           reasoning_strength="low")
    rows.append({"key": row["key"], "family": row["family"], "prompt": prompt, "rewards": row["rewards"]})
  (data / "rendered.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
  print(f"{data / 'rendered.jsonl'}: {len(rows)} prompts")


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("command", nargs="?", default="build", choices=("build", "render"))
  parser.add_argument("--solve", default=str(ROOT / "logs/expert/train_all_sorting.jsonl"))
  parser.add_argument("--out", default=str(ROOT / "logs/grpo/data"))
  parser.add_argument("--data", default=str(ROOT / "logs/grpo/data"))
  args = parser.parse_args()
  build(args.solve, Path(args.out)) if args.command == "build" else render(Path(args.data))
