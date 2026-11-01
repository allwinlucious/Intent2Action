"""DPO pairs from the expert's all-strategies runs: on the same task and scene, the label program (chosen) against a
strategy program that failed there (rejected; the docs-level listed order when it failed).

  .venv/bin/python learn/dpo_data.py --solve logs/expert/train_all_sorting.jsonl --out logs/dpo/pairs
  .venv-ml/bin/python learn/dpo_data.py render --data logs/dpo/pairs     # Glimmer's chat template -> strings

`build` writes {train,validation}.jsonl with {"family", "seed", "messages", "chosen", "rejected"} (programs);
`render` writes rendered/{train,validation}.jsonl with TRL's standard preference format: {"prompt", "chosen",
"rejected"} as text, the prompt ending at the assistant turn and each completion the full reply up to <|eot|>.
"""

import argparse
import json
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen"), str(ROOT / "learn")]


def pairs(solve: list[dict]) -> list[dict]:
  from learn import sft_data
  from program_runtime.agent import program_messages
  from program_runtime.api import Obj
  rates = sft_data.strategy_rates(solve)
  out = []
  for value in solve:
    if value.get("split") != "train" or "objects" not in value:
      continue
    chosen = sft_data.best_program(value, rates)
    failed = [e for e in value["episodes"] if not e["ok"] and e["program"] != chosen]
    if not chosen or not failed:
      continue
    rejected = next((e for e in failed if e["strategy"] == "listed"), failed[0])
    out.append({"family": value["family"], "seed": value["seed"], "rejected_strategy": rejected["strategy"],
                "messages": program_messages(value["intent"], [Obj(**o) for o in value["objects"]]),
                "chosen": chosen, "rejected": rejected["program"]})
  return out


def build(solve_path, out_dir: Path) -> None:
  rows = pairs([json.loads(line) for line in Path(solve_path).read_text().splitlines()])
  random.Random(0).shuffle(rows)
  n_val = max(1, len(rows) // 20)
  out_dir.mkdir(parents=True, exist_ok=True)
  for name, part in (("validation", rows[:n_val]), ("train", rows[n_val:])):
    (out_dir / f"{name}.jsonl").write_text("".join(json.dumps(r) + "\n" for r in part))
  by_family = {}
  for row in rows:
    by_family[row["family"]] = by_family.get(row["family"], 0) + 1
  print(f"{out_dir}: {len(rows)} pairs ({len(rows) - n_val} train, {n_val} validation) {by_family}")


def fenced(program: str) -> str:
  return "```python\n" + program + "```"


def render(data: Path) -> None:
  import transformers
  tokenizer = transformers.AutoTokenizer.from_pretrained("meta-models/Muse-Glimmer-30B")
  out_dir = data / "rendered"
  out_dir.mkdir(exist_ok=True)
  for name in ("train", "validation"):
    rows = []
    for line in (data / f"{name}.jsonl").read_text().splitlines():
      row = json.loads(line)
      prompt = tokenizer.apply_chat_template(row["messages"], add_generation_prompt=True, tokenize=False,
                                             reasoning_strength="low")
      completions = {}
      for key in ("chosen", "rejected"):
        full = tokenizer.apply_chat_template([*row["messages"], {"role": "assistant", "content": fenced(row[key])}],
                                             tokenize=False, reasoning_strength="low")
        assert full.startswith(prompt)
        completions[key] = full[len(prompt):]
      rows.append({"prompt": prompt, **completions})
    (out_dir / f"{name}.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    print(f"{out_dir / name}.jsonl: {len(rows)} pairs")


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("command", nargs="?", default="build", choices=("build", "render"))
  parser.add_argument("--solve", default=str(ROOT / "logs/expert/train_all_sorting.jsonl"))
  parser.add_argument("--out", default=str(ROOT / "logs/dpo/pairs"))
  parser.add_argument("--data", default=str(ROOT / "logs/dpo/pairs"))
  args = parser.parse_args()
  build(args.solve, Path(args.out)) if args.command == "build" else render(Path(args.data))
