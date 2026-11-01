"""Evaluation of a model version through offline generation on Modal (learn/modal_train.py generate).

  .venv/bin/python learn/offline_eval.py requests --solve logs/expert/heldout_all.jsonl --out logs/eval/requests.jsonl
  .venv/bin/python learn/offline_eval.py spawn --version G1                 # uploads requests, spawns generation
  .venv/bin/python learn/offline_eval.py collect --version G1               # waits, downloads, writes programs
  .venv/bin/python learn/fleet.py --workers 8 -- learn/glimmer_sample.py run --in logs/eval/G1_programs.jsonl \
      --out logs/eval/G1_episodes.jsonl

Every version (G0 = the base model) goes through the same path: the product's writer prompt on the held-out
scene the expert recorded, greedy decoding, one attempt (no corrective retry), the program extracted as the
product does (`_program_text`).
"""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]

from learn.episodes import read_jsonl  # noqa: E402
from program_runtime.agent import _program_text, program_messages  # noqa: E402
from program_runtime.api import Obj  # noqa: E402

LOGS = ROOT / "logs" / "eval"
MODAL = [str(ROOT / "tools" / "modal.sh")]


def requests(solve_path, out_path) -> int:
  rows = []
  for value in read_jsonl(solve_path):
    if "objects" not in value:
      continue
    objects = [Obj(**obj) for obj in value["objects"]]
    rows.append({"key": f"{value['family']}:{value['seed']}", "family": value["family"], "seed": value["seed"],
                 "split": value["split"], "messages": program_messages(value["intent"], objects)})
  Path(out_path).parent.mkdir(parents=True, exist_ok=True)
  Path(out_path).write_text("".join(json.dumps(row) + "\n" for row in rows))
  return len(rows)


def spawn(version: str, requests_path) -> None:
  subprocess.run([*MODAL, "volume", "put", "--force", "show-once-train", str(requests_path), "eval/requests.jsonl"],
                 check=True)
  code = ("import modal; f = modal.Function.from_name('show-once-train', 'generate'); "
          f"c = f.spawn('eval/requests.jsonl', 'eval/{version}_replies.jsonl', '{version}'); print(c.object_id)")
  subprocess.run([str(ROOT / ".venv-ml/bin/python"), "-c", code], check=True,
                 env=_modal_env())


def _modal_env() -> dict:
  import os
  env = dict(os.environ)
  for line in (ROOT / ".env").read_text().splitlines():
    if line.startswith(("MODAL_TOKEN_ID=", "MODAL_TOKEN_SECRET=")):
      key, value = line.split("=", 1)
      env[key] = value.strip().strip("'\"")
  return env


def collect(version: str, requests_path, expected: int, timeout_s: float = 4 * 3600) -> Path:
  """Waits until the replies file on the volume has every request, then writes <version>_programs.jsonl."""
  local = LOGS / f"{version}_replies.jsonl"
  deadline = time.monotonic() + timeout_s
  while True:
    subprocess.run([*MODAL, "volume", "get", "--force", "show-once-train", f"eval/{version}_replies.jsonl",
                    str(local)], capture_output=True)
    have = len(read_jsonl(local)) if local.exists() else 0
    print(f"{version}: {have}/{expected} replies", flush=True)
    if have >= expected:
      break
    if time.monotonic() > deadline:
      raise TimeoutError(f"{version}: {have}/{expected} replies after {timeout_s:.0f} s")
    time.sleep(60)
  return programs(version, requests_path, local)


def programs(version: str, requests_path, replies_path) -> Path:
  """<version>_programs.jsonl from a replies file (Modal volume download or a pod's generate output)."""
  replies = {value["key"]: value["text"] for value in read_jsonl(replies_path)}
  out = LOGS / f"{version}_programs.jsonl"
  with out.open("w") as handle:
    for row in read_jsonl(requests_path):
      handle.write(json.dumps({"family": row["family"], "seed": row["seed"], "split": row["split"], "sample": 0,
                               "version": version, "program": _program_text(replies[row["key"]])}) + "\n")
  print(f"wrote {out}")
  return out


def main() -> None:
  parser = argparse.ArgumentParser()
  sub = parser.add_subparsers(dest="command", required=True)
  p = sub.add_parser("requests")
  p.add_argument("--solve", default=str(ROOT / "logs/expert/heldout_all.jsonl"))
  p.add_argument("--out", default=str(LOGS / "requests.jsonl"))
  for name in ("spawn", "collect", "programs"):
    p = sub.add_parser(name)
    p.add_argument("--version", required=True)
    p.add_argument("--requests", default=str(LOGS / "requests.jsonl"))
    if name == "programs":
      p.add_argument("--replies", required=True, help="a local replies file (e.g. copied from a pod)")
  args = parser.parse_args()
  if args.command == "requests":
    print(f"{requests(args.solve, args.out)} requests -> {args.out}")
  elif args.command == "spawn":
    spawn(args.version, args.requests)
  elif args.command == "programs":
    programs(args.version, args.requests, args.replies)
  else:
    collect(args.version, args.requests, len(read_jsonl(args.requests)))


if __name__ == "__main__":
  main()
