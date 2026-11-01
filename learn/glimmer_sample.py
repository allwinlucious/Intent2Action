"""Glimmer's own attempts: k programs per train task from the endpoint, then each one run in the running stack.

  set -a; source .env; set +a
  .venv/bin/python learn/glimmer_sample.py generate --split train --n 150 --k 4 --out logs/samples/train_programs.jsonl
      [--solve PATH] [--families a,b]
  .venv/bin/python learn/glimmer_sample.py run --in logs/samples/train_programs.jsonl
      --out logs/samples/train_episodes.jsonl [--start-stack plan]

`generate` asks the endpoint in LLM_BASE_URL (temperature 0.7, up to 8 requests at once) through the product's
ProgramWriter, on the scene of every solve record with `objects` (solved by the expert or not). `run` runs every
sampled program through the episode runner. They are separate so the paid endpoint is up only while generating.
Both skip work already in their output; a failed request is written as an `error` record and retried next time.
"""

import argparse
import collections
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import threading
import time
import urllib.error

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]

from learn import families  # noqa: E402
from learn.episodes import connect, in_shard, read_jsonl, record, run_episode, stack  # noqa: E402
from program_runtime.agent import ChatClient, ProgramWriter  # noqa: E402
from program_runtime.api import Obj  # noqa: E402

TEMPERATURE = 0.7
WORKERS = 8


def solve_records(path, split: str, names: list[str], n: int) -> list[dict]:
  """The solve records with `objects` among the first `n` seeds of each family, as `solve --n` selects them."""
  start = 100000 if split == "heldout" else 0
  return [value for value in read_jsonl(path) if "objects" in value and value["split"] == split
          and value["family"] in names and value["seed"] < start + n]


def _sample(client, value: dict, index: int) -> dict:
  key = {"family": value["family"], "seed": value["seed"], "split": value["split"], "sample": index}
  writer = ProgramWriter(client)
  started = time.monotonic()
  try:
    program = writer.write(value["intent"], [Obj(**obj) for obj in value["objects"]])
  except (urllib.error.URLError, TimeoutError) as error:
    return key | {"error": f"{type(error).__name__}: {error}"}
  return key | {"program": program, "attempts": writer.last_attempts, "seconds": round(time.monotonic() - started, 1)}


def generate(records: list[dict], k: int, out_path, client) -> None:
  done = {(value["family"], value["seed"], value["sample"]) for value in read_jsonl(out_path) if "program" in value}
  jobs = [(value, index) for value in records for index in range(k)
          if (value["family"], value["seed"], index) not in done]
  print(f"resume: {len(done)} samples already in {out_path}; {len(jobs)} to generate", flush=True)
  lock = threading.Lock()
  counts = collections.Counter()

  def work(job) -> None:
    result = _sample(client, *job)
    with lock:
      out.write(json.dumps(result) + "\n")
      out.flush()
      counts["error" if "error" in result else "program"] += 1
      detail = result.get("error") or f"attempts {result['attempts']} {result['seconds']:.1f}s"
      print(f"{'ERROR' if 'error' in result else 'OK'} {result['family']} {result['seed']} sample {result['sample']} "
            f"{detail}", flush=True)

  with open(out_path, "a") as out, ThreadPoolExecutor(max_workers=WORKERS) as pool:
    list(pool.map(work, jobs))
  written = [value for value in read_jsonl(out_path) if "program" in value]
  print(f"wrote {out_path}: programs={counts['program']} errors={counts['error']} (retried on the next run); "
        f"file: {len(written)} programs, first try {sum(value['attempts'] == 1 for value in written)}")


def run(samples: list[dict], out_path, sim_stub, robot_stub, shard: str | None = None, agent=None) -> None:
  """`agent`: the recovery agent for handoffs (evaluation of a model version with recovery), or None."""
  stored = {(value["family"], value["seed"], value["sample"]): value.get("program") for value in read_jsonl(out_path)}
  stale = []
  for value in samples:
    key = (value["family"], value["seed"], value["sample"])
    if "program" in value and stored.get(key) not in (None, value["program"]):
      stale.append(key)
  if stale:  # regenerated programs: an old episode must not be kept under the same key
    sys.exit(f"{out_path} has episodes of other programs for {len(stale)} samples (e.g. {stale[:2]}); move it away")
  done = set(stored)
  todo = [value for value in samples
          if "program" in value and (value["family"], value["seed"], value["sample"]) not in done
          and in_shard(value["family"], value["seed"], shard)]
  print(f"resume: {len(done)} samples already in {out_path}; {len(todo)} to run", flush=True)
  with open(out_path, "a") as out:
    for value in todo:
      task = families.generate(value["family"], value["seed"])
      episode = run_episode(task, value["program"], sim_stub, robot_stub, agent=agent)
      out.write(json.dumps(record(episode) | {"sample": value["sample"], "intent": task.intent}) + "\n")
      out.flush()
  print(f"wrote {out_path}")
  by_family = collections.defaultdict(list)
  for value in read_jsonl(out_path):
    by_family[value["family"]].append(value["ok"])
  for family, oks in by_family.items():
    print(f"{family}: samples={len(oks)} ok={100 * sum(oks) / len(oks):.0f}%")


def main() -> None:
  parser = argparse.ArgumentParser()
  subparsers = parser.add_subparsers(dest="command", required=True)
  sub = subparsers.add_parser("generate")
  sub.add_argument("--split", choices=("train", "heldout"), required=True)
  sub.add_argument("--n", type=int, required=True)
  sub.add_argument("--families", help="comma-separated; default: all families valid for the split")
  sub.add_argument("--k", type=int, default=4)
  sub.add_argument("--solve", help="default: logs/expert/<split>.jsonl")
  sub.add_argument("--out", required=True)
  sub.add_argument("--model", help="model name at the endpoint (a version: G0 is the base name, G1... adapters)")
  sub.add_argument("--temperature", type=float, default=TEMPERATURE, help="0 for evaluation")
  sub = subparsers.add_parser("run")
  sub.add_argument("--shard", help="k/N: only this worker's samples (learn/fleet.py)")
  sub.add_argument("--recover", metavar="MODEL", help="the model version as live recovery agent on failures")
  sub.add_argument("--in", dest="input", required=True)
  sub.add_argument("--out", required=True)
  sub.add_argument("--start-stack", choices=("none", "plan"), default="none")
  args = parser.parse_args()
  Path(args.out).parent.mkdir(parents=True, exist_ok=True)
  if args.command == "generate":
    valid = [family for family in families.FAMILIES if not (family == "mixed" and args.split == "train")]
    names = args.families.split(",") if args.families else valid
    unknown = sorted(set(names) - set(valid))
    if unknown:
      parser.error(f"families not valid for {args.split}: {', '.join(unknown)}")
    solve = args.solve or str(ROOT / "logs" / "expert" / f"{args.split}.jsonl")
    generate(solve_records(solve, args.split, names, args.n), args.k, args.out,
             ChatClient(model=args.model, temperature=args.temperature))
  else:
    with stack(args.start_stack):
      sim_stub, robot_stub = connect()
      agent = None
      if args.recover:
        from mcp_server.server import DESCRIPTOR_SET, Tools
        from program_runtime.agent import RecoveryAgent
        agent = RecoveryAgent(ChatClient(model=args.recover), robot_stub, Tools(DESCRIPTOR_SET))
      run(read_jsonl(args.input), args.out, sim_stub, robot_stub, args.shard, agent)


if __name__ == "__main__":
  main()
