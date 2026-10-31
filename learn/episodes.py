"""One episode: load a task into the running stack, run a program on it, grade the result with the rule checker."""

import dataclasses
from pathlib import Path
import sys
import time

import grpc
from google.protobuf import empty_pb2

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]

from learn import families  # noqa: E402
from program_runtime.api import GrpcBackend, RobotAPI  # noqa: E402
from program_runtime.rules import check_intent, truth_snapshot  # noqa: E402
from program_runtime.runtime import run  # noqa: E402
from robot_service import robot_service_pb2_grpc as api_grpc  # noqa: E402

ADDRESS = "127.0.0.1:18100"


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


def connect():
  """(sim_stub, robot_stub) on one channel: robot_service serves both."""
  channel = grpc.insecure_channel(ADDRESS)
  return api_grpc.SimServiceStub(channel), api_grpc.RobotServiceStub(channel)


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
    duration_s=round(time.monotonic() - started, 2), events=events,
  )
  print(f"{'OK' if episode.ok else 'FAIL'} {task.family} {task.seed} {strategy or 'model'} {episode.duration_s:.1f} "
        f"{episode.calls} {episode.error or ('' if episode.ok else episode.rules_summary)}", flush=True)
  return episode
