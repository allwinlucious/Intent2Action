"""Records one demo take in Isaac as a real-speed 30 fps mp4 (the stack must be up: tests/e2e/demo_stack.py).

  .venv/bin/python tools/record_take.py mixed:100002 --from-eval WONDER --name small_beats_big_wonder
  .venv/bin/python tools/record_take.py mixed:100002 --from-eval G1d --name small_beats_big_ours
  .venv/bin/python tools/record_take.py cubes_in_box:0 --program my_program.py --name cubes

The scene is a demo preset id or any task `family:seed`. The program is a file, or a version's evaluated program
for that task (logs/eval/<VERSION>_programs.jsonl), so a take shows exactly what was measured. The take resets the
scene, homes the arm, runs the program through the runtime (check, dry run, execution; no recovery agent), and
encodes the frames Isaac saved meanwhile (one per 1/30 s of sim time in --pretty mode) to
C:\\isaac-sim-work\\videos\\<name>.mp4, and the run's events to demo_runs/<name>.events.jsonl (committed).
"""

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "gen"), str(ROOT / "learn"), str(ROOT / "tests/e2e")]

FRAMES = Path("/mnt/c/isaac-sim-work/viewer_frames")
VIDEOS = Path("/mnt/c/isaac-sim-work/videos")
PROOF = ROOT / "demo_runs"
FPS = 30
SETTLE_S = 2.0  # still frames before and after the run


def _task(scene: str):
  import families
  from demo_ui.scenes import BY_ID
  if scene in BY_ID:
    preset = BY_ID[scene]
    return preset.placements, preset.intent
  if scene.endswith(".json"):  # a scene saved as {"placements": [...], "intent": {...}}
    value = json.loads(Path(scene).read_text())
    return [families.Placement(**p) for p in value["placements"]], value["intent"]
  family, seed = scene.split(":")
  task = families.generate(family, int(seed))
  return task.placements, task.intent


def _program(args) -> str:
  if args.program:
    return Path(args.program).read_text()
  family, seed = args.scene.split(":")
  for line in (ROOT / f"logs/eval/{args.from_eval}_programs.jsonl").read_text().splitlines():
    value = json.loads(line)
    if value["family"] == family and value["seed"] == int(seed):
      return value["program"]
  raise SystemExit(f"no {args.from_eval} program for {args.scene}")


def _newest_index() -> int:
  names = [p.name for p in FRAMES.glob("frame_*.png")]
  return max((int(n[6:11]) for n in names), default=-1)


def encode(first: int, last: int, out: Path) -> int:
  import imageio.v2 as imageio
  written = 0
  with imageio.get_writer(out, fps=FPS, codec="libx264", quality=9, macro_block_size=8) as writer:
    for index in range(first, last + 1):
      try:
        writer.append_data(imageio.imread(FRAMES / f"frame_{index:05d}.png")[..., :3])
        written += 1
      except (OSError, ValueError, FileNotFoundError):  # a frame still being written, or skipped by Isaac
        continue
  return written


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("scene")
  source = parser.add_mutually_exclusive_group(required=True)
  source.add_argument("--program")
  source.add_argument("--from-eval", metavar="VERSION")
  parser.add_argument("--name", required=True)
  parser.add_argument("--address", default="127.0.0.1:18100")
  parser.add_argument("--recover", metavar="MODEL", help="a model version as the recovery agent (LLM_BASE_URL)")
  parser.add_argument("--answer", help="the operator's reply if the recovery agent asks (logged as such)")
  args = parser.parse_args()

  import families
  import phase3_cuj_check as cuj
  from program_runtime.api import GrpcBackend, RobotAPI
  from program_runtime.runtime import run

  placements, intent = _task(args.scene)
  program = _program(args)
  families.load_placements(placements, cuj._sim, cuj._robot)
  cuj._move("Home")
  time.sleep(SETTLE_S)
  first = _newest_index() + 1
  VIDEOS.mkdir(parents=True, exist_ok=True)
  PROOF.mkdir(parents=True, exist_ok=True)
  events = (PROOF / f"{args.name}.events.jsonl").open("w")  # in the repo: what the robot was told and did

  def log(event: dict) -> None:
    events.write(json.dumps(event, separators=(",", ":")) + "\n")
    events.flush()

  hooks = {}
  if args.recover:
    # The operator's part is scripted for a recorded take: the reply to a question, and approval of every patch
    # and every robot move the agent asks for; each is logged with what was approved.
    import grpc
    from mcp_server.server import DESCRIPTOR_SET, Tools
    from program_runtime.agent import ChatClient, RecoveryAgent
    from robot_service import robot_service_pb2_grpc as api_grpc
    agent = RecoveryAgent(ChatClient(model=args.recover), api_grpc.RobotServiceStub(grpc.insecure_channel(args.address)),
                          Tools(DESCRIPTOR_SET), model=args.recover)

    def approve_tool(name, arguments):
      log({"type": "operator", "approved_tool": name, "arguments": arguments})
      return True, ""

    def ask(question):
      log({"type": "operator", "question": question, "answer": args.answer})
      return args.answer or ""

    def approve(patch, reason):
      log({"type": "operator", "approved_patch": patch, "reason": reason})
      return True, ""

    agent.approve_tool = approve_tool
    hooks = {"agent": agent, "ask": ask, "approve": approve}
  result = run(program, RobotAPI(GrpcBackend(args.address)), log=log, **hooks)
  events.close()
  time.sleep(SETTLE_S)
  last = _newest_index()
  out = VIDEOS / f"{args.name}.mp4"
  frames = encode(first, last, out)
  print(f"{'OK' if result.ok else 'FAILED: ' + str(result.error)[:200]}")
  (PROOF / f"{args.name}.program.py").write_text(program)
  print(f"{out}: {frames} frames, {frames / FPS:.1f} s at {FPS} fps (intent: {intent.get('description', '')})")
  return 0


if __name__ == "__main__":
  sys.exit(main())
