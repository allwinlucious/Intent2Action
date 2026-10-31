"""One episode: load a task into the running stack, run a program on it, grade the result with the rule checker."""

import contextlib
import dataclasses
import json
import os
from pathlib import Path
import socket
import sys
import time
import zlib

import grpc
from google.protobuf import empty_pb2

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]

from learn import families  # noqa: E402
from program_runtime.api import GrpcBackend, RobotAPI  # noqa: E402
from program_runtime.rules import check_intent, truth_snapshot  # noqa: E402
from program_runtime.runtime import run  # noqa: E402
from robot_service import robot_service_pb2_grpc as api_grpc  # noqa: E402

ADDRESS = os.environ.get("ROBOT_ADDRESS", "127.0.0.1:18100")  # learn/fleet.py sets one per container


@dataclasses.dataclass
class Episode:
  family: str
  seed: int
  split: str
  strategy: str | None  # strategy name, or None for a model-written program
  program: str
  ok: bool  # runtime ok and rules ok
  runtime_ok: bool
  rules_ok: bool
  rules_summary: str
  per_object: dict
  error: str | None  # runtime error text
  failure: dict | None  # first failure: kind, message, call, line
  handoffs: list
  calls: int  # pick + place calls made
  duration_s: float
  events: list[dict]
  objects: list[dict] = dataclasses.field(default_factory=list)  # the scene right after loading, as Obj dicts


def connect():
  """(sim_stub, robot_stub) on one channel: robot_service serves both."""
  channel = grpc.insecure_channel(ADDRESS)
  return api_grpc.SimServiceStub(channel), api_grpc.RobotServiceStub(channel)


def _require_port() -> None:
  host, port = ADDRESS.split(":")
  try:
    socket.create_connection((host, int(port)), timeout=2).close()
  except OSError:
    sys.exit(f"robot_service is not reachable on {ADDRESS}: start the stack (plan tier: "
             f"ROBOT_BACKEND=plan container/run.sh) or pass --start-stack plan")


@contextlib.contextmanager
def stack(start: str):
  """Starts the plan-tier container when `start` is "plan" (stopped on exit), then requires robot_service."""
  run = None
  try:
    if start == "plan":
      sys.path.insert(0, str(ROOT / "tests" / "e2e"))
      import phase3_hwm_check as hwm  # noqa: E402  (imports ICON's Python; only needed to start the stack)
      import plan_tier_check  # noqa: E402

      hwm.CONTAINER_LOG = plan_tier_check.CONTAINER_LOG
      run = hwm._Run()
      run.start_container(backend="plan")
      plan_tier_check._wait_ready()
    _require_port()
    yield
  finally:
    if run is not None:
      run.stop_container()


def in_shard(family: str, seed: int, shard: str | None) -> bool:
  """`shard` "k/N": whether task (family, seed) belongs to worker k of N (stable across runs: resume works)."""
  if not shard:
    return True
  k, n = (int(v) for v in shard.split("/"))
  return zlib.crc32(f"{family}:{seed}".encode()) % n == k


def read_jsonl(path) -> list[dict]:
  """The records of a JSON-lines file; none when it does not exist."""
  path = Path(path)
  return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def record(episode: Episode) -> dict:
  return {key: value for key, value in dataclasses.asdict(episode).items() if key != "events"}


def run_episode(task, program, sim_stub, robot_stub, agent=None, strategy=None) -> Episode:
  """Program and robot failures are the episode's result; only an unreachable stack or a loader timeout raises."""
  try:
    robot_stub.GetState(empty_pb2.Empty(), timeout=10)
  except grpc.RpcError as error:
    raise RuntimeError(f"stack not reachable: {error.code().name}: {error.details()}") from error
  started = time.monotonic()
  families.load(task, sim_stub, robot_stub)
  objects = [dataclasses.asdict(obj) for obj in GrpcBackend(stub=robot_stub).list_objects()]
  before = truth_snapshot(sim_stub)
  events = []
  result = run(program, RobotAPI(GrpcBackend(stub=robot_stub)), agent=agent, log=events.append)
  rules = check_intent(task.intent, before, truth_snapshot(sim_stub))
  episode = Episode(
    family=task.family, seed=task.seed, split=task.split, strategy=strategy, program=program,
    ok=result.ok and rules.ok, runtime_ok=result.ok, rules_ok=rules.ok, rules_summary=rules.summary,
    per_object=rules.per_object, error=result.error, failure=result.failures[0] if result.failures else None,
    handoffs=result.handoffs,
    calls=sum(event["kind"] == "result" and event["fn"] in {"pick", "place"} for event in events),
    duration_s=round(time.monotonic() - started, 2), events=events, objects=objects,
  )
  print(f"{'OK' if episode.ok else 'FAIL'} {task.family} {task.seed} {strategy or 'model'} {episode.duration_s:.1f} "
        f"{episode.calls} {episode.error or ('' if episode.ok else episode.rules_summary)}", flush=True)
  return episode
