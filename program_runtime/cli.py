"""Command-line entry point for writing and running robot programs."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import sys

import grpc

from mcp_server.server import DESCRIPTOR_SET, Tools

from .agent import ChatClient, ProgramWriter, RecoveryAgent
from .api import FakeBackend, GrpcBackend, Obj, RobotAPI
from .reader import IntentReader, ReadError
from .rules import validate_intent
from .runtime import run


def _frame_count(value: str) -> int:
  count = int(value)
  if count < 2:
    raise argparse.ArgumentTypeError("must be at least 2")
  return count


def _parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(prog="python -m program_runtime.cli")
  subparsers = parser.add_subparsers(dest="command", required=True)
  command = subparsers.add_parser("run", help="run a checked robot program")
  command.add_argument("--intent", required=True, type=Path)
  source = command.add_mutually_exclusive_group(required=True)
  source.add_argument("--program", type=Path)
  source.add_argument("--write", action="store_true")
  command.add_argument("--recover", action="store_true")
  command.add_argument("--writer-model")
  command.add_argument("--recovery-model")
  command.add_argument("--address", default="127.0.0.1:18100")
  command.add_argument("--log", type=Path)
  command.add_argument("--fake-scene", type=Path)
  command = subparsers.add_parser("read", help="read a task intent from a clip or sentence")
  source = command.add_mutually_exclusive_group(required=True)
  source.add_argument("--clip", type=Path)
  source.add_argument("--text")
  command.add_argument("--note")
  command.add_argument("--frames", type=_frame_count, default=6)
  command.add_argument("--model", default="G0")
  command.add_argument("--out", type=Path)
  return parser


def _default_log() -> Path:
  stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
  return Path("logs") / "runs" / f"{stamp}.jsonl"


def _read(args) -> int:
  reader = IntentReader(ChatClient(), model=args.model)
  try:
    reading = reader.read_clip(args.clip, count=args.frames, note=args.note) \
        if args.clip else reader.read_text(args.text)
  except ReadError as error:
    print(f"FAILED: {error}", file=sys.stderr)
    for problem in error.problems:
      print(f"- {problem}", file=sys.stderr)
    return 1
  print(f"SEEN: {reading.seen}")
  output = json.dumps(reading.intent, indent=2)
  print(output)
  if args.out:
    args.out.write_text(output + "\n")
  return 0


def _run(args) -> int:
  intent = json.loads(args.intent.read_text())
  problems = validate_intent(intent)
  if problems:
    print("FAILED: intent: " + "; ".join(problems))
    return 1
  if args.fake_scene:
    objects = [Obj(**value) for value in json.loads(args.fake_scene.read_text())]
    backend = FakeBackend(objects)
  else:
    backend = GrpcBackend(args.address)
  api = RobotAPI(backend)
  client = ChatClient() if args.write or args.recover else None
  if args.write:
    source = ProgramWriter(client, model=args.writer_model).write(intent, backend.list_objects())
  else:
    source = args.program.read_text()
  print(source.rstrip())

  agent = None
  if args.recover:
    if isinstance(backend, GrpcBackend):
      stub = backend.stub
    else:
      from robot_service import robot_service_pb2_grpc
      stub = robot_service_pb2_grpc.RobotServiceStub(grpc.insecure_channel(args.address))
    agent = RecoveryAgent(client, stub, Tools(DESCRIPTOR_SET), model=args.recovery_model)

  log_path = args.log or _default_log()
  log_path.parent.mkdir(parents=True, exist_ok=True)
  with log_path.open("w") as stream:
    def write_log(event: dict) -> None:
      stream.write(json.dumps(event, separators=(",", ":")) + "\n")
      stream.flush()

    result = run(source, api, agent=agent, log=write_log)
  for line in result.report:
    print(line)
  if result.ok:
    print("OK")
    return 0
  print(f"FAILED: {result.error}")
  return 1


def main(argv: list[str] | None = None) -> int:
  parser = _parser()
  args = parser.parse_args(argv)
  if args.command == "read" and args.note is not None and args.clip is None:
    parser.error("--note requires --clip")
  return _read(args) if args.command == "read" else _run(args)


signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))


if __name__ == "__main__":
  sys.exit(main())
