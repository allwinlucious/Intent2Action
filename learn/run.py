"""Runs a policy over a task split in the sim; one JSONL line per episode in logs/learn/<out>.jsonl.

  .venv-ml/bin/python learn/run.py --policy expert|qwen|lora:<adapter dir>|head:<head dir> \
      --split eval|train --count 30 [--start 0] [--repeat 1] --out <name>
"""

import argparse
import collections
import json
import time

import agent
import env
import tasks as tasks_lib


def make_policy(spec: str):
  if spec == "expert":
    import expert
    return expert.Policy()
  if spec in ("qwen", "qwen-think"):
    import qwen
    return qwen.Policy(spec, thinking=spec == "qwen-think")
  if spec.startswith("lora:"):
    import qwen
    return qwen.Policy(spec, adapter=spec.removeprefix("lora:"))
  if spec.startswith("head:"):
    import head
    return head.Policy(spec, spec.removeprefix("head:"))
  raise ValueError(spec)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--policy", required=True)
  parser.add_argument("--split", default="eval")
  parser.add_argument("--count", type=int, default=30)
  parser.add_argument("--start", type=int, default=0)
  parser.add_argument("--repeat", type=int, default=1)
  parser.add_argument("--families", default="")
  parser.add_argument("--out", required=True)
  args = parser.parse_args()
  suite = tasks_lib.suite(args.split, args.start + args.count, 0)[args.start:]
  if args.families:
    suite = [t for t in suite if t.family in args.families.split(",")]
  path = env.LOG_DIR / f"{args.out}.jsonl"
  finished = collections.Counter(json.loads(l)["task"]["id"] for l in open(path)) if path.exists() else {}
  todo = [t for r in range(args.repeat) for t in suite if finished.get(t.id, 0) <= r]  # resume after a stop
  if not todo:
    print(f"DONE {args.out}: nothing left", flush=True)
    return
  policy = make_policy(args.policy)
  print(f"policy {args.policy} loaded; GPU {env.gpu_used_mib()} MiB", flush=True)
  start, passed = time.monotonic(), 0
  with env.Stack(args.out), open(env.LOG_DIR / f"{args.out}.jsonl", "a") as out:
    print(f"stack up; GPU {env.gpu_used_mib()} MiB", flush=True)
    for task in todo:
      passed += agent.episode(policy, task, out)["success"]
  total = len(todo)
  print(f"DONE {args.out}: {passed}/{total} passed in {(time.monotonic() - start) / 60:.0f} min", flush=True)


if __name__ == "__main__":
  main()
