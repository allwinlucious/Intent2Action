"""Training rows for Glimmer SFT in the cookbook's Axolotl format: one JSON object per line with `messages`.

  .venv/bin/python learn/sft_data.py smoke --out logs/sft/smoke
      program-writing rows from the example programs on random fake scenes: exercises the training path only
  .venv/bin/python learn/sft_data.py build --version {G1,G2,G3} --out DIR [--solve PATH] [--samples PATH]
      [--recovery PATH]
      G1: program rows of every solved train task; G2: G1 + rows of Glimmer's successful samples (at most 2 per
      task); G3: G2 + confirmed recovery rows (G1 + recovery rows when the samples file does not exist), recovery
      rows at most 30 % of the dataset. Writes DIR/{train,validation}.jsonl and DIR/manifest.json; then
      learn/render_rows.py renders them with Glimmer's chat template.

Prompts come from program_runtime.agent.program_messages and recovery_messages, the product's own prompts, so
training and use match. Recovery rows also carry the `tools` the recovery agent sends.
"""

import argparse
import collections
import json
import math
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen"), str(ROOT / "learn")]

from glimmer_program_test import scene  # noqa: E402
from learn.episodes import read_jsonl  # noqa: E402
from mcp_server.server import DESCRIPTOR_SET, Tools  # noqa: E402
from program_runtime.agent import program_messages, recovery_messages, tool_specs  # noqa: E402
from program_runtime.api import Obj  # noqa: E402

EXAMPLES = ROOT / "program_runtime" / "examples"
KINDS = ("program", "self", "recovery")
SELF_PER_TASK = 2


def program_row(intent: dict, objects, program: str) -> dict:
  """The fenced form is what the writer's parser accepts and what the zero-shot model writes."""
  return {"messages": [*program_messages(intent, objects), {"role": "assistant",
                                                            "content": "```python\n" + program + "```"}]}


def recovery_row(example: dict, specs: list[dict]) -> dict:
  return {"messages": [*recovery_messages(example["context"]),
                       {"role": "assistant", "content": json.dumps(example["decision"])}], "tools": specs}


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


def _objects(value: dict) -> list[Obj]:
  return [Obj(**obj) for obj in value["objects"]]


def strategy_rates(solve: list[dict]) -> dict[tuple[str, str], float]:
  """(family, strategy) -> success rate over the tasks that ran it."""
  runs = collections.Counter()
  wins = collections.Counter()
  for value in solve:
    for episode in value["episodes"]:
      key = (value["family"], episode["strategy"])
      runs[key] += 1
      wins[key] += bool(episode["ok"])
  return {key: wins[key] / runs[key] for key in runs}


def best_program(value: dict, rates: dict) -> str | None:
  """The label: among the strategies that succeeded on this task, the one most reliable on its family (a strategy
  that worked by luck here, e.g. the listed order on a tower, loses to one that always works). Records that ran
  only up to the first success have one success: that program."""
  ok = [episode for episode in value["episodes"] if episode["ok"]]
  if not ok:
    return value.get("expert_program")
  order = {episode["strategy"]: i for i, episode in enumerate(value["episodes"])}
  best = max(ok, key=lambda e: (rates.get((value["family"], e["strategy"]), 0.0), -order[e["strategy"]]))
  return best["program"]


def program_rows(solve: list[dict]) -> list[tuple[str, str, dict]]:
  rates = strategy_rates(solve)
  rows = []
  for value in solve:
    program = best_program(value, rates)
    if program:
      rows.append(("program", value["family"], program_row(value["intent"], _objects(value), program)))
  return rows


def self_rows(solve: list[dict], samples: list[dict]) -> list[tuple[str, str, dict]]:
  """Per task the two lowest sample indexes among the successful, distinct programs, on the scene they were
  written for (the solve record's)."""
  scenes = {(value["family"], value["seed"]): value for value in solve}
  chosen = collections.defaultdict(list)
  for value in sorted((value for value in samples if value["ok"]), key=lambda value: value["sample"]):
    key = (value["family"], value["seed"])
    if key in scenes and value["program"] not in chosen[key] and len(chosen[key]) < SELF_PER_TASK:
      chosen[key].append(value["program"])
  return [("self", key[0], program_row(scenes[key]["intent"], _objects(scenes[key]), program))
          for key, programs in chosen.items() for program in programs]


def recovery_rows(examples: list[dict], specs: list[dict], others: int) -> list[tuple[str, str, dict]]:
  """Confirmed examples, capped at 30 % of the dataset: r <= 0.3 (others + r) is r <= 3 * others / 7."""
  rows = [("recovery", value["family"], recovery_row(value, specs)) for value in examples if value.get("confirmed")]
  limit = 3 * others // 7
  return random.Random(0).sample(rows, limit) if len(rows) > limit else rows


def validation_size(n: int) -> int:
  if n < 2:
    return 0
  share = math.ceil(n / 20)
  return share if share >= 16 else max(1, min(16, n // 5))


def split(rows: list) -> tuple[list, list]:
  """(train, validation) after a seed-0 shuffle."""
  rows = list(rows)
  random.Random(0).shuffle(rows)
  n_val = validation_size(len(rows))
  return rows[n_val:], rows[:n_val]


def _train(path) -> list[dict]:
  if not Path(path).exists():
    sys.exit(f"{path} does not exist")
  return [value for value in read_jsonl(path) if value["split"] == "train"]


def build(version: str, solve_path, samples_path, recovery_path, out: Path) -> dict:
  solve = [value for value in _train(solve_path) if "objects" in value]
  rows = program_rows(solve)
  sources = [str(solve_path)]
  fallback = None
  if version in ("G2", "G3"):
    if Path(samples_path).exists():
      rows += self_rows(solve, _train(samples_path))
      sources.append(str(samples_path))
    elif version == "G2":
      sys.exit(f"G2 needs the samples file {samples_path}, which does not exist")
    else:
      fallback = "the samples file does not exist: G1 + recovery rows"
  if version == "G3":
    rows += recovery_rows(_train(recovery_path), tool_specs(Tools(DESCRIPTOR_SET)), len(rows))
    sources.append(str(recovery_path))
  train, validation = split(rows)
  out.mkdir(parents=True, exist_ok=True)
  for name, part in (("train", train), ("validation", validation)):
    (out / f"{name}.jsonl").write_text("".join(json.dumps(row) + "\n" for _, _, row in part))
  kinds = collections.Counter(kind for kind, _, _ in rows)
  per_family = collections.defaultdict(collections.Counter)
  for kind, family, _ in rows:
    per_family[family][kind] += 1
  manifest = {
    "version": version, "rows": {kind: kinds[kind] for kind in KINDS},
    "families": {family: {kind: counts[kind] for kind in KINDS} for family, counts in sorted(per_family.items())},
    "train": len(train), "validation": len(validation), "fallback": fallback, "sources": sources,
  }
  (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
  print(f"{version} -> {out}: train={len(train)} validation={len(validation)} "
        + " ".join(f"{kind}={kinds[kind]}" for kind in KINDS) + (f" ({fallback})" if fallback else ""))
  for family, counts in manifest["families"].items():
    print(f"  {family}: " + " ".join(f"{kind}={counts[kind]}" for kind in KINDS))
  return manifest


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  subparsers = parser.add_subparsers(dest="kind", required=True)
  subparsers.add_parser("smoke").add_argument("--out", default=str(ROOT / "logs/sft/smoke"))
  sub = subparsers.add_parser("build")
  sub.add_argument("--version", choices=("G1", "G2", "G3"), required=True)
  sub.add_argument("--solve", default=str(ROOT / "logs/expert/train.jsonl"))
  sub.add_argument("--samples", default=str(ROOT / "logs/samples/train_episodes.jsonl"))
  sub.add_argument("--recovery", default=str(ROOT / "logs/expert/recovery_train.jsonl"))
  sub.add_argument("--out", required=True)
  args = parser.parse_args()
  if args.kind == "smoke":
    smoke(Path(args.out))
  else:
    build(args.version, args.solve, args.samples, args.recovery, Path(args.out))
