"""Runs a resumable plan-tier command on several plan-tier containers side by side, one shard of the tasks each.

  .venv/bin/python learn/fleet.py --workers 8 -- learn/search_expert.py solve --split train --n 150 --out logs/expert/train.jsonl
  .venv/bin/python learn/fleet.py --workers 8 -- learn/search_expert.py recovery --split train --n 150 --out logs/expert/recovery_train.jsonl

Container k is `plan_k` on its own Docker network with robot_service published on 127.0.0.1:<base-port + k>
(container/run.sh PUBLISH_PORT); its worker gets ROBOT_ADDRESS for that port and `--shard k/N --out <out>.shard<k>`.
Records already in <out> are split into the shard files first (the shard of a task is stable), so every worker
resumes; afterwards the shard files are merged back into <out> (one record per task) and the containers removed.
One plan-tier container uses about one core and 0.6 GB.
"""

import argparse
import contextlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]

import grpc  # noqa: E402
from google.protobuf import empty_pb2  # noqa: E402

from learn.episodes import in_shard, read_jsonl  # noqa: E402
from robot_service import robot_service_pb2 as api  # noqa: E402
from robot_service import robot_service_pb2_grpc as api_grpc  # noqa: E402

READY_TIMEOUT_S = 300
WORKER_RESTARTS = 2  # a failed worker resumes its shard
LOGS = ROOT / "logs" / "fleet"


def _key(value: dict) -> tuple:
  return (value["family"], value["seed"], value.get("sample"))


def _docker(command: str) -> None:
  subprocess.run(["sg", "docker", "-c", command], capture_output=True)


@contextlib.contextmanager
def containers(n: int, base_port: int):
  """Starts plan_0..plan_{n-1}; yields their addresses once each robot_service lists the default scene."""
  LOGS.mkdir(parents=True, exist_ok=True)
  names = [f"plan_{k}" for k in range(n)]
  try:
    for k, name in enumerate(names):
      env = {**os.environ, "CONTAINER_NAME": name, "PUBLISH_PORT": str(base_port + k), "ROBOT_BACKEND": "plan"}
      subprocess.Popen(["bash", str(ROOT / "container/run.sh")], env=env, stdin=subprocess.DEVNULL,
                       stdout=open(LOGS / f"{name}.log", "w"), stderr=subprocess.STDOUT)
    addresses = [f"127.0.0.1:{base_port + k}" for k in range(n)]
    deadline = time.monotonic() + READY_TIMEOUT_S
    for name, address in zip(names, addresses):
      stub = api_grpc.RobotServiceStub(grpc.insecure_channel(address))
      while True:
        try:
          # The scene and the motion planner (it starts last and slowest when many containers boot together).
          if stub.ListObjects(empty_pb2.Empty(), timeout=5).objects:
            stub.ComputeFk(api.ComputeFkRequest(joints=api.Joints(positions=[0.0] * 7), tool=api.TCP), timeout=10)
            print(f"{name} ready on {address}", flush=True)
            break
        except grpc.RpcError:
          pass
        if time.monotonic() > deadline:
          raise RuntimeError(f"{name} not ready after {READY_TIMEOUT_S} s (see {LOGS / (name + '.log')})")
        time.sleep(2)
    yield addresses
  finally:
    for name in names:
      _docker(f"docker rm -f {name}")
    print(f"removed {', '.join(names)}", flush=True)


def split(out: Path, n: int) -> list[Path]:
  """Moves the records of `out` into the shard files they belong to (keeping what the shards already hold)."""
  shards = [out.with_name(f"{out.name}.shard{k}") for k in range(n)]
  existing = read_jsonl(out)
  for k, path in enumerate(shards):
    have = {_key(v) for v in read_jsonl(path)}
    mine = [v for v in existing if in_shard(v["family"], v["seed"], f"{k}/{n}") and _key(v) not in have]
    with path.open("a") as handle:
      handle.writelines(json.dumps(v) + "\n" for v in mine)
  return shards


def merge(out: Path, shards: list[Path]) -> int:
  """Writes every distinct record of `out` and the shards to `out` (shards win), removes the shards."""
  records = {_key(v): v for v in read_jsonl(out)}
  for path in shards:
    records.update({_key(v): v for v in read_jsonl(path)})
  tmp = out.with_name(out.name + ".tmp")
  tmp.write_text("".join(json.dumps(v) + "\n" for v in records.values()))
  tmp.replace(out)
  for path in shards:
    path.unlink(missing_ok=True)
  return len(records)


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--workers", type=int, default=8)
  parser.add_argument("--base-port", type=int, default=18200)
  parser.add_argument("command", nargs=argparse.REMAINDER, help="-- script args... (must include --out PATH)")
  args = parser.parse_args()
  command = args.command[1:] if args.command[:1] == ["--"] else args.command
  if "--out" not in command:
    parser.error("the command needs --out PATH (the merged output)")
  out = ROOT / command[command.index("--out") + 1] if not Path(command[command.index("--out") + 1]).is_absolute() \
      else Path(command[command.index("--out") + 1])
  out.parent.mkdir(parents=True, exist_ok=True)
  shards = split(out, args.workers)
  failed = 0
  with containers(args.workers, args.base_port) as addresses:
    def start(k: int, address: str) -> subprocess.Popen:
      argv = list(command)
      argv[argv.index("--out") + 1] = str(shards[k])
      argv += ["--shard", f"{k}/{args.workers}"]
      log = open(LOGS / f"worker_{k}.log", "a")  # noqa: SIM115 (kept open for the child)
      print(f"worker {k}: {address} -> {shards[k]} (log {LOGS / f'worker_{k}.log'})", flush=True)
      return subprocess.Popen([sys.executable, *argv], cwd=ROOT, env={**os.environ, "ROBOT_ADDRESS": address},
                              stdout=log, stderr=subprocess.STDOUT)

    procs = {k: start(k, address) for k, address in enumerate(addresses)}
    restarts = dict.fromkeys(procs, 0)
    while procs:
      time.sleep(5)
      for k, proc in list(procs.items()):
        code = proc.poll()
        if code is None:
          continue
        print(f"worker {k} exited {code}", flush=True)
        if code != 0 and restarts[k] < WORKER_RESTARTS:  # e.g. a service in its container was not up yet
          restarts[k] += 1
          time.sleep(30)
          procs[k] = start(k, addresses[k])
          continue
        failed += code != 0
        del procs[k]
  print(f"merged {merge(out, shards)} records into {out}", flush=True)
  return 1 if failed else 0


if __name__ == "__main__":
  sys.exit(main())
