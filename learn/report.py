"""Metrics of episode runs (logs/learn/<run>.jsonl) and stage-1 benchmarks (<bench>_summary.json), as markdown.

  .venv-ml/bin/python learn/report.py --runs qwen_eval b1_eval ... --benches bench_qwen bench_b1 ...
"""

import argparse
import collections
import json

import env

INVALID = {"INVALID_ARGUMENT", "INVALID_ARGUMENTS", "NOT_FOUND", "UNKNOWN_TOOL"}


def episode_metrics(records: list[dict]) -> dict:
  calls = [c for r in records for c in r["calls"]]
  n = len(records)
  ok = sum(r["success"] for r in records)
  errors = [c["status"] != "OK" for c in calls]
  recover, after_error = 0, 0
  for r in records:
    for a, b in zip(r["calls"], r["calls"][1:]):
      if a["status"] != "OK":
        after_error += 1
        recover += b["status"] == "OK"
  gen = [c.get("gen_seconds", 0) for c in calls]
  return {
      "success": f"{ok}/{n} ({ok / n:.0%})" if n else "-",
      "error-call rate": f"{sum(errors) / max(len(calls), 1):.0%}",
      "invalid-arg rate": f"{sum(c['status'] in INVALID for c in calls) / max(len(calls), 1):.0%}",
      "calls/episode": f"{len(calls) / max(n, 1):.1f}",
      "calls/success": f"{len(calls) / ok:.1f}" if ok else "-",
      "recovery": f"{recover}/{after_error}" if after_error else "-",
      "hit 40-call limit": sum(r.get("hit_call_limit", False) for r in records),
      "gen s/call": f"{sum(gen) / max(len(gen), 1):.1f}",
  }


def family_table(runs: dict[str, list[dict]]) -> str:
  families = sorted({r["task"]["family"] for rs in runs.values() for r in rs})
  lines = ["| run | " + " | ".join(families) + " |", "|---|" + "---|" * len(families)]
  for name, rs in runs.items():
    by = collections.defaultdict(lambda: [0, 0])
    for r in rs:
      by[r["task"]["family"]][0] += r["success"]
      by[r["task"]["family"]][1] += 1
    lines.append(f"| {name} | " + " | ".join(f"{by[f][0]}/{by[f][1]}" for f in families) + " |")
  return "\n".join(lines)


def error_table(runs: dict[str, list[dict]]) -> str:
  lines = ["| run | top error statuses |", "|---|---|"]
  for name, rs in runs.items():
    count = collections.Counter(c["status"] for r in rs for c in r["calls"] if c["status"] != "OK")
    lines.append(f"| {name} | " + ", ".join(f"{k} {v}" for k, v in count.most_common(5)) + " |")
  return "\n".join(lines)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--runs", nargs="*", default=[])
  parser.add_argument("--benches", nargs="*", default=[])
  args = parser.parse_args()
  runs = {}
  for name in args.runs:
    path = env.LOG_DIR / f"{name}.jsonl"
    if path.exists():
      runs[name] = [json.loads(l) for l in path.read_text().splitlines()]
  if runs:
    metrics = {name: episode_metrics(rs) for name, rs in runs.items()}
    keys = list(next(iter(metrics.values())))
    print("| run | " + " | ".join(keys) + " |\n|---|" + "---|" * len(keys))
    for name, m in metrics.items():
      print(f"| {name} | " + " | ".join(str(m[k]) for k in keys) + " |")
    print("\nSuccess per family:\n\n" + family_table(runs))
    print("\nErrors:\n\n" + error_table(runs))
  if args.benches:
    print("\nStage-1 benchmark (call matches the accepted call / stops when infeasible):\n")
    summaries = {b: json.loads((env.LOG_DIR / f"{b}_summary.json").read_text()) for b in args.benches
                 if (env.LOG_DIR / f"{b}_summary.json").exists()}
    keys = sorted({k for s in summaries.values() for k in s if "/" not in k})
    print("| bench | " + " | ".join(keys) + " |\n|---|" + "---|" * len(keys))
    for b, s in summaries.items():
      print(f"| {b} | " + " | ".join(f"{s[k][0]}/{s[k][1]} ({s[k][0] / s[k][1]:.0%})" if k in s else "-" for k in keys) + " |")


if __name__ == "__main__":
  main()
