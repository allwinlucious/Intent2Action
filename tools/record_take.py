"""Records one demo take in Isaac as a real-speed 30 fps mp4 (the stack must be up: tests/e2e/demo_stack.py).

  .venv/bin/python tools/record_take.py mixed:100002 --from-eval WONDER --name small_beats_big_wonder
  .venv/bin/python tools/record_take.py mixed:100002 --from-eval G1d --name small_beats_big_ours
  .venv/bin/python tools/record_take.py cubes_in_box:0 --program my_program.py --name cubes

The scene is a demo preset id or any task `family:seed`. The program is a file, or a version's evaluated program
for that task (logs/eval/<VERSION>_programs.jsonl), so a take shows exactly what was measured. The take resets the
scene, homes the arm, runs the program through the runtime (check, dry run, execution; no recovery agent), and
encodes the frames Isaac saved meanwhile (one per 1/30 s of sim time in --pretty mode) to
C:\\isaac-sim-work\\videos\\<name>.mp4, plus <name>.jsonl with the run's events.
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
FPS = 30
SETTLE_S = 2.0  # still frames before and after the run


def _task(scene: str):
  import families
  from demo_ui.scenes import BY_ID
  if scene in BY_ID:
    preset = BY_ID[scene]
    return preset.placements, preset.intent
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
  events = (VIDEOS / f"{args.name}.jsonl").open("w")
  result = run(program, RobotAPI(GrpcBackend(args.address)),
               log=lambda event: events.write(json.dumps(event, separators=(",", ":")) + "\n"))
  events.close()
  time.sleep(SETTLE_S)
  last = _newest_index()
  out = VIDEOS / f"{args.name}.mp4"
  frames = encode(first, last, out)
  print(f"{'OK' if result.ok else 'FAILED: ' + str(result.error)[:200]}")
  print(f"{out}: {frames} frames, {frames / FPS:.1f} s at {FPS} fps (intent: {intent.get('description', '')})")
  return 0


if __name__ == "__main__":
  sys.exit(main())
