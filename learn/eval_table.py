"""Held-out results table: success per family and version, with 95 % intervals, plus the references.

  .venv/bin/python learn/eval_table.py [--repeat 3] DOCS G0 G1b ...
      reads logs/eval/<V>_episodes[_r<k>].jsonl and logs/expert/heldout_all.jsonl

A task's success is the mean over its runs (the plan tier samples, so one program's outcome is a probability).
Intervals: percentile bootstrap over tasks (2000 resamples of the family's tasks, each carrying its run mean), the
unit that was sampled. References, both single runs from the expert's held-out pass:
  fixed best strategy — per family, the strategy most reliable on the TRAINING tasks, run on each held-out task;
  oracle — a task counts if any of the 4-5 strategies succeeded on it (a search ceiling, not a model).
A version must cover every reference task with all `k` runs; otherwise the table is refused (a partial file must
not look like a result).
"""

import collections
import json
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "learn")]


def load(path) -> list[dict]:
  return [json.loads(line) for line in Path(path).read_text().splitlines()]


def bootstrap(values: list[float], resamples: int = 2000, seed: int = 0) -> tuple[float, float]:
  if not values:
    return 0.0, 0.0
  rng = random.Random(seed)
  means = sorted(sum(rng.choices(values, k=len(values))) / len(values) for _ in range(resamples))
  return 100 * means[int(0.025 * resamples)], 100 * means[int(0.975 * resamples) - 1]


def task_means(version: str, suffix: str, runs: int, keys: set) -> dict:
  per_task = collections.defaultdict(list)
  for e in load(ROOT / f"logs/eval/{version}_episodes{suffix}.jsonl"):
    per_task[(e["family"], e["seed"])].append(e)
  missing = sorted(keys - set(per_task))
  short = sorted(k for k, es in per_task.items() if k in keys and len({e.get("sample", 0) for e in es}) < runs)
  if missing or short:
    sys.exit(f"{version}: incomplete: {len(missing)} tasks missing, {len(short)} with fewer than {runs} runs "
             f"(e.g. {(missing or short)[:3]})")
  return {k: {"mean": sum(bool(e["ok"]) for e in es) / len(es), "runs": es} for k, es in per_task.items() if k in keys}


def references() -> tuple[dict, dict, dict]:
  """(docs-level, fixed best strategy, oracle) success per held-out task, single runs."""
  from sft_data import strategy_rates
  rates = strategy_rates(load(ROOT / "logs/expert/train_all_sorting.jsonl"))
  best = {}
  for (family, strategy), rate in rates.items():
    if family not in best or rate > rates[(family, best[family])]:
      best[family] = strategy
  best["mixed"] = best["sorting"]  # held-out family: the choice of the most similar trained family
  import families
  import strategies
  norm = lambda text: "\n".join(line.rstrip() for line in text.strip().splitlines())  # noqa: E731
  docs, fixed, oracle = {}, {}, {}
  for r in load(ROOT / "logs/expert/heldout_all.jsonl"):
    key, by_strategy = (r["family"], r["seed"]), {e["strategy"]: e["ok"] for e in r["episodes"]}
    docs[key] = float(by_strategy.get("listed", False))
    # By program text: a strategy that was not run on a task (top-first without a tower) writes the same program
    # as one that was (towers are built in the intent's order by every strategy).
    program = norm(strategies.program(families.generate(*key).intent, best[r["family"]]))
    fixed[key] = float(next((e["ok"] for e in r["episodes"] if norm(e["program"]) == program), False))
    oracle[key] = float(any(by_strategy.values()))
  return docs, fixed, oracle


def cell(values: list[float]) -> str:
  lo, hi = bootstrap(values)
  return f"{100 * sum(values) / len(values):.0f} % [{lo:.0f}–{hi:.0f}]"


def main(versions: list[str], runs: int) -> None:
  suffix = f"_r{runs}" if runs > 1 else ""
  docs, fixed, oracle = references()
  keys = set(oracle)
  results = {v: task_means(v, suffix, runs, keys) for v in versions}
  families = sorted({f for f, _ in keys})
  print("| family | " + " | ".join(versions) + " | docs-level (1 run) | fixed best strategy (1 run) | oracle |")
  print("|---|" + "---|" * (len(versions) + 3))
  for family in families + ["all"]:
    ks = [k for k in sorted(keys) if family == "all" or k[0] == family]
    cells = [cell([results[v][k]["mean"] for k in ks]) for v in versions]
    refs = [f"{100 * sum(ref[k] for k in ks) / len(ks):.0f} %" for ref in (docs, fixed, oracle)]
    print(f"| {family} | " + " | ".join(cells + refs) + " |")
  for v in versions:
    kinds = collections.Counter()
    for value in results[v].values():
      for e in value["runs"]:
        if not e["ok"]:
          error = e.get("error") or ""
          kind = (e.get("failure") or {}).get("kind")
          kinds[kind or ("check/dry run" if error.startswith(("check", "dry run")) else
                         "rules" if e.get("runtime_ok") else "other")] += 1
    print(f"{v} failures: {dict(kinds)}")


if __name__ == "__main__":
  args = sys.argv[1:]
  runs = 1
  if "--repeat" in args:
    i = args.index("--repeat")
    runs = int(args[i + 1])
    args = args[:i] + args[i + 2:]
  main(args, runs)
