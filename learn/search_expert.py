"""Search expert: runs strategy programs on a task in the running stack; the first that succeeds is the expert's program.

  .venv/bin/python learn/search_expert.py solve --split train --n 20 [--families a,b] [--out PATH] [--start-stack plan]
  .venv/bin/python learn/search_expert.py recovery --split train --n 20 [--in PATH] [--out PATH] [--start-stack plan]

`solve` writes one JSON line per task; `recovery` re-runs the failed `listed` episodes of solved tasks with an agent
that answers the first handoff with the expert program as a patch, and writes the confirmed examples.
Both append to their output and skip tasks already in it, so a stopped run continues where it stopped; solve
records without `objects` (written before phase L4) count as absent.
"""

import argparse
import collections
import dataclasses
import json
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]

from learn import families, strategies  # noqa: E402
from learn import sft_data  # noqa: E402
from learn.episodes import connect, in_shard, read_jsonl, record, run_episode, stack  # noqa: E402

STACK_TOLERANCE_M = 0.01


def stacked(task) -> bool:
  """A cube placed at another cube's x, y and higher."""
  cubes = [placement for placement in task.placements if placement.catalog in families.CUBE_CATALOGS]
  return any(abs(upper.x - lower.x) <= STACK_TOLERANCE_M and abs(upper.y - lower.y) <= STACK_TOLERANCE_M
             and upper.z > lower.z for upper in cubes for lower in cubes if upper is not lower)


def solve(task, sim_stub, robot_stub, all_strategies: bool = False) -> list:
  """Episodes of the strategies in order up to the first success (`listed` always runs first), or of every
  applicable strategy with `all_strategies` (the training label is then chosen by sft_data.best_strategy)."""
  episodes = []
  for strategy in strategies.STRATEGIES:
    if strategy == "top_first" and not stacked(task):
      continue
    episode = run_episode(task, strategies.program(task.intent, strategy), sim_stub, robot_stub, strategy=strategy)
    episodes.append(episode)
    if episode.ok and not all_strategies:
      break
  return episodes


def task_record(task, episodes: list) -> dict:
  expert = next((episode for episode in episodes if episode.ok), None)
  return {
    "family": task.family, "seed": task.seed, "split": task.split, "intent": task.intent,
    "placements": [dataclasses.asdict(placement) for placement in task.placements],
    "expert_strategy": expert.strategy if expert else None,
    "expert_program": expert.program if expert else None,
    "objects": episodes[0].objects,
    "episodes": [record(episode) for episode in episodes],
  }


class CapturingAgent:
  """Keeps the runtime's first handoff context and answers every handoff with the expert program as a patch."""

  def __init__(self, strategy: str, program: str):
    self.strategy = strategy
    self.program = program
    self.context = None
    self.decision = None

  def recover(self, context: dict) -> dict:
    if self.context is None:
      self.context = {key: value for key, value in context.items() if key != "_backend"}
      self.decision = {
        "rung": "patch",
        "reason": f"{context['kind']} failure at {context['call']}: switch to the {self.strategy} order",
        "patch": self.program,
      }
    return dict(self.decision)


def _task(value: dict):
  placements = tuple(families.Placement(**placement) for placement in value["placements"])
  return families.Task(value["family"], value["seed"], value["split"], value["intent"], placements, 0)


def recover(value: dict, sim_stub, robot_stub) -> tuple[str, dict | None, object]:
  """(outcome, example or None, episode) for one solve record with an expert program and a failed `listed` run."""
  listed = value["episodes"][0]
  strategy, program = value.get("label_strategy") or value["expert_strategy"], value.get("label_program") or value["expert_program"]
  agent = CapturingAgent(strategy, program)
  episode = run_episode(_task(value), listed["program"], sim_stub, robot_stub, agent=agent, strategy="listed")
  if agent.context is None:
    return "no_handoff", None, episode
  if not episode.ok:
    return "not_confirmed", None, episode
  example = {
    "family": value["family"], "seed": value["seed"], "split": value["split"], "context": agent.context,
    "decision": agent.decision, "confirmed": True, "episode": record(episode),
  }
  return "confirmed", example, episode


def _eligible(value: dict) -> bool:
  listed = value["episodes"][0] if value["episodes"] else None
  return bool(value["expert_program"]) and listed is not None and listed["strategy"] == "listed" and not listed["ok"]


def _mean(values) -> float:
  return statistics.mean(values) if values else 0.0


def _solve_summary(family: str, records: list[dict]) -> str:
  episodes = [episode for value in records for episode in value["episodes"]]
  solved = sum(value["expert_strategy"] is not None for value in records)
  listed = sum(value["episodes"][0]["ok"] for value in records)
  wins = collections.Counter(value["expert_strategy"] for value in records if value["expert_strategy"])
  win_text = " ".join(f"{strategy}={wins[strategy]}" for strategy in strategies.STRATEGIES)
  n = max(len(records), 1)
  return (f"{family}: tasks={len(records)} solved={100 * solved / n:.0f}% listed={100 * listed / n:.0f}% "
          f"wins {win_text} mean_s={_mean([e['duration_s'] for e in episodes]):.1f} "
          f"mean_calls={_mean([e['calls'] for e in episodes]):.1f}")


def _solve_records(path) -> list[dict]:
  return [value for value in read_jsonl(path) if "objects" in value]


def _run_solve(args, names: list[str], sim_stub, robot_stub) -> None:
  done = {(value["family"], value["seed"]) for value in _solve_records(args.out)}
  print(f"resume: {len(done)} tasks already in {args.out}", flush=True)
  with open(args.out, "a") as out:
    for family in names:
      for task in families.tasks(family, args.split, args.n):
        if (task.family, task.seed) in done or not in_shard(task.family, task.seed, getattr(args, "shard", None)):
          continue
        value = task_record(task, solve(task, sim_stub, robot_stub, getattr(args, "all_strategies", False)))
        out.write(json.dumps(value) + "\n")
        out.flush()
  print(f"wrote {args.out}")
  records = _solve_records(args.out)
  for family in names:
    print(_solve_summary(family, [value for value in records if value["family"] == family]))


def _run_recovery(args, names: list[str], sim_stub, robot_stub) -> None:
  start = 100000 if args.split == "heldout" else 0
  values = [value for value in _solve_records(args.input) if value["split"] == args.split
            and value["family"] in names and value["seed"] < start + args.n]
  # The patch is the same label the program rows use (sft_data.best_program): the most reliable successful strategy
  # on the family; with first-success records, two teachers disagreed (G1c, results 2026-11-01).
  rates = sft_data.strategy_rates(values)
  for value in values:
    best = sft_data.best_program(value, rates)
    value["label_program"] = best
    value["label_strategy"] = next((e["strategy"] for e in value["episodes"] if e["program"] == best), None)
  done = {(value["family"], value["seed"]) for value in read_jsonl(args.out)}
  print(f"resume: {len(done)} tasks already in {args.out}", flush=True)
  counts = {family: collections.Counter() for family in names}
  timing = {family: [] for family in names}
  with open(args.out, "a") as out:
    for value in values:
      if (not _eligible(value) or (value["family"], value["seed"]) in done
          or not in_shard(value["family"], value["seed"], getattr(args, "shard", None))):
        continue
      outcome, example, episode = recover(value, sim_stub, robot_stub)
      counts[value["family"]][outcome] += 1
      timing[value["family"]].append(episode)
      if example is not None:
        out.write(json.dumps(example) + "\n")
        out.flush()
  print(f"wrote {args.out}")
  outcomes = ("confirmed", "not_confirmed", "no_handoff")
  in_file = collections.Counter(value["family"] for value in read_jsonl(args.out))
  for family in names:
    episodes = timing[family]
    print(f"{family}: tasks={len(episodes)} " + " ".join(f"{name}={counts[family][name]}" for name in outcomes)
          + f" mean_s={_mean([e.duration_s for e in episodes]):.1f} mean_calls={_mean([e.calls for e in episodes]):.1f}"
          + f"; file: confirmed={in_file[family]}")
  total = sum(counts.values(), collections.Counter())
  print("total: " + " ".join(f"{name}={total[name]}" for name in outcomes)
        + f"; {total['confirmed']} recovery examples confirmed; file: {sum(in_file.values())} examples")


def main() -> None:
  parser = argparse.ArgumentParser()
  subparsers = parser.add_subparsers(dest="command", required=True)
  for name in ("solve", "recovery"):
    sub = subparsers.add_parser(name)
    sub.add_argument("--split", choices=("train", "heldout"), required=True)
    sub.add_argument("--n", type=int, required=True)
    sub.add_argument("--families", help="comma-separated; default: all families valid for the split")
    sub.add_argument("--out")
    sub.add_argument("--start-stack", choices=("none", "plan"), default="none")
    sub.add_argument("--shard", help="k/N: only this worker's tasks (learn/fleet.py)")
    if name == "solve":
      sub.add_argument("--all-strategies", action="store_true", help="run every strategy, not up to the first success")
    if name == "recovery":
      sub.add_argument("--in", dest="input")
  args = parser.parse_args()
  valid = [family for family in families.FAMILIES if not (family == "mixed" and args.split == "train")]
  names = args.families.split(",") if args.families else valid
  unknown = sorted(set(names) - set(valid))
  if unknown:
    parser.error(f"families not valid for {args.split}: {', '.join(unknown)}")
  logs = ROOT / "logs" / "expert"
  if args.command == "solve":
    args.out = args.out or str(logs / f"{args.split}.jsonl")
  else:
    args.input = args.input or str(logs / f"{args.split}.jsonl")
    args.out = args.out or str(logs / f"recovery_{args.split}.jsonl")
  Path(args.out).parent.mkdir(parents=True, exist_ok=True)
  with stack(args.start_stack):
    sim_stub, robot_stub = connect()
    if args.command == "solve":
      _run_solve(args, names, sim_stub, robot_stub)
    else:
      _run_recovery(args, names, sim_stub, robot_stub)


if __name__ == "__main__":
  main()
