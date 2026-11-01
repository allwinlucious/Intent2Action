"""The sim tier: a version's held-out programs on Isaac physics, and how often physics agrees with the plan tier.

  .venv/bin/python learn/isaac_eval.py run --version G1b --per-family 10      # starts Isaac (headless) + the stack
  .venv/bin/python learn/isaac_eval.py agreement --version G1b

`run` takes the first `per-family` held-out programs of each family from logs/eval/<V>_programs.jsonl and writes
logs/eval/<V>_isaac_episodes.jsonl (resumes). `agreement` compares each with the plan-tier episode of the same
program: both ok, plan ok / physics failed (what planning misses), plan failed / physics ok.
"""

import argparse
import collections
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen"), str(ROOT / "tests" / "e2e")]

from learn.episodes import read_jsonl  # noqa: E402

LOGS = ROOT / "logs" / "eval"


def run(version: str, per_family: int) -> None:
  import harness
  import phase1_check as p1
  import phase3_cuj_check as cuj
  import phase3_hwm_check as hwm
  from learn import glimmer_sample
  from learn.episodes import connect

  programs = read_jsonl(LOGS / f"{version}_programs.jsonl")
  chosen, count = [], collections.Counter()
  for value in sorted(programs, key=lambda v: (v["family"], v["seed"])):
    if count[value["family"]] < per_family:
      chosen.append(value)
      count[value["family"]] += 1
  harness.ISAAC_COMMAND[-1] += " --physics-only"
  harness.sync_isaac()
  hwm.CONTAINER_LOG = LOGS / f"{version}_isaac_container.log"
  stack = hwm._Run()
  try:
    stack.start_isaac(LOGS / f"{version}_isaac.log")
    stack.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    sim_stub, robot_stub = connect()
    glimmer_sample.run(chosen, LOGS / f"{version}_isaac_episodes.jsonl", sim_stub, robot_stub)
  finally:
    stack.stop_container()
    stack.stop_isaac()


def agreement(version: str) -> None:
  plan = {(v["family"], v["seed"]): v["ok"] for v in read_jsonl(LOGS / f"{version}_episodes.jsonl")}
  physics = {(v["family"], v["seed"]): v for v in read_jsonl(LOGS / f"{version}_isaac_episodes.jsonl")}
  table = collections.Counter()
  for key, value in physics.items():
    table[(plan[key], value["ok"])] += 1
  n = sum(table.values())
  print(f"{version}: {n} programs on both tiers")
  print(f"  both ok {table[(True, True)]}, both failed {table[(False, False)]}, "
        f"plan ok / physics failed {table[(True, False)]}, plan failed / physics ok {table[(False, True)]}")
  print(f"  agreement {100 * (table[(True, True)] + table[(False, False)]) / n:.0f} %; "
        f"success plan {100 * sum(plan[k] for k in physics) / n:.0f} %, physics "
        f"{100 * sum(v['ok'] for v in physics.values()) / n:.0f} %")
  for key, value in sorted(physics.items()):
    if plan[key] and not value["ok"]:
      print(f"  physics-only failure {key}: {(value.get('error') or '')[:160]}")


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  sub = parser.add_subparsers(dest="command", required=True)
  p = sub.add_parser("run")
  p.add_argument("--version", required=True)
  p.add_argument("--per-family", type=int, default=10)
  p = sub.add_parser("agreement")
  p.add_argument("--version", required=True)
  args = parser.parse_args()
  run(args.version, args.per_family) if args.command == "run" else agreement(args.version)
