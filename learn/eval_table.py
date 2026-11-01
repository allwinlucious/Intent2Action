"""Held-out results table: success per family and version with 95 % Wilson intervals, plus the references.

  .venv/bin/python learn/eval_table.py G0 G1 G2 ...   (reads logs/eval/<V>_episodes.jsonl, logs/expert/heldout_all.jsonl)
"""

import collections
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
  if n == 0:
    return 0.0, 0.0
  p = k / n
  d = 1 + z * z / n
  c = (p + z * z / (2 * n)) / d
  h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
  return 100 * (c - h), 100 * (c + h)


def load(path) -> list[dict]:
  return [json.loads(line) for line in Path(path).read_text().splitlines()]


def main(versions: list[str]) -> None:
  ref = {}
  for r in load(ROOT / "logs/expert/heldout_all.jsonl"):
    listed = next((e["ok"] for e in r["episodes"] if e["strategy"] == "listed"), False)
    ref[(r["family"], r["seed"])] = (listed, any(e["ok"] for e in r["episodes"]))
  # With repeated runs (<V>_episodes_r<k>.jsonl, flag --repeat k) a task's success is its mean over the runs.
  results = {}
  for v in versions:
    path = ROOT / f"logs/eval/{v}_episodes{SUFFIX}.jsonl"
    runs = collections.defaultdict(list)
    for e in load(path):
      runs[(e["family"], e["seed"])].append(e)
    results[v] = {k: {**es[0], "ok": sum(bool(e["ok"]) for e in es) / len(es), "runs": es} for k, es in runs.items()}
  families = sorted({f for f, _ in ref})
  print("| family | docs-level | " + " | ".join(versions) + " | best strategy |")
  print("|---|---|" + "---|" * len(versions) + "---|")
  for family in families + ["all"]:
    keys = [k for k in ref if family == "all" or k[0] == family]
    n = len(keys)
    cells = []
    for v in versions:
      ok = sum(float(results[v].get(k, {}).get("ok") or 0) for k in keys)
      lo, hi = wilson(round(ok), n)
      cells.append(f"{100 * ok / n:.0f} % [{lo:.0f}–{hi:.0f}]")
    docs = 100 * sum(ref[k][0] for k in keys) / n
    best = 100 * sum(ref[k][1] for k in keys) / n
    print(f"| {family} | {docs:.0f} % | " + " | ".join(cells) + f" | {best:.0f} % |")
  for v in versions:
    kinds = collections.Counter()
    for e in (r for value in results[v].values() for r in value["runs"]):
      if not e["ok"]:
        error = e.get("error") or ""
        kind = (e.get("failure") or {}).get("kind")
        kinds[kind or ("check/dry run" if error.startswith(("check", "dry run")) else "rules" if e.get("runtime_ok") else "other")] += 1
    print(f"{v} failures: {dict(kinds)}")


SUFFIX = ""

if __name__ == "__main__":
  args = sys.argv[1:]
  if "--repeat" in args:
    i = args.index("--repeat")
    SUFFIX = f"_r{args[i + 1]}"
    args = args[:i] + args[i + 2:]
  main(args)
