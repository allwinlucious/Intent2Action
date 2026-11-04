"""Demo: the viewer open with presentation lighting (--gui --pretty); the robot runs the sort-by-colour program.

  .venv/bin/python tests/e2e/demo_program.py [--keep-running]

Starts run_scene.py with the viewer and the container, lays out three cubes and two boxes, runs
program_runtime/examples/sort_by_color.py through the program runtime (no model), checks the result against
ground truth, and writes logs/demo/program_demo.mp4 (viewer frames at real speed), a still, and the run log.
"""

import shutil
import sys

import imageio.v2 as imageio
import harness
import phase1_check as p1
import phase3_cuj_check as cuj
import phase3_hwm_check as hwm
import program_check as pc

from program_runtime.api import GrpcBackend, RobotAPI
from program_runtime.rules import check_intent, truth_snapshot
from program_runtime.runtime import run

OUT = harness.LOG_DIR / "demo"
FPS = 15  # frames are 1/15 s apart in sim time (run_scene --pretty): real speed
LAYOUT = 1


def _video() -> harness.Path | None:
  frames = sorted(harness.VIEWER_FRAMES.glob("frame_*.png"))
  if not frames:
    return None
  path = OUT / "program_demo.mp4"
  with imageio.get_writer(path, fps=FPS, codec="libx264", quality=8, macro_block_size=8) as writer:
    for frame in frames:
      try:
        writer.append_data(imageio.imread(frame)[..., :3])
      except (OSError, ValueError):  # a frame still being written when Isaac stopped
        continue
  shutil.copy(frames[-1], OUT / "program_demo_end.png")
  return path


def main() -> int:
  keep = "--keep-running" in sys.argv
  OUT.mkdir(parents=True, exist_ok=True)
  harness.ISAAC_COMMAND[-1] += " --gui --pretty"
  harness.READY_TIMEOUT_S = 900  # the first start downloads the warehouse environment
  harness.sync_isaac()
  hwm.CONTAINER_LOG = OUT / "program_demo_container.log"
  stack = hwm._Run()
  ok = False
  try:
    stack.start_isaac(OUT / "program_demo_isaac.log")
    stack.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    intent, source = pc._source("sort_by_color")
    pc._scene("sort_by_color", LAYOUT)
    before = truth_snapshot(cuj._sim)
    print("scene ready; running the program", flush=True)
    stream, logger = pc._log("demo")
    try:
      result = run(source, RobotAPI(GrpcBackend()), log=logger)
    finally:
      stream.close()
    rules = check_intent(intent, before, truth_snapshot(cuj._sim))
    ok = result.ok and rules.ok
    print(f"runtime ok={result.ok} error={result.error}; rules: {rules.summary}", flush=True)
    for line in result.report:
      print(f"report: {line}", flush=True)
  finally:
    if not keep:
      stack.stop_container()
      stack.stop_isaac()
  video = _video()
  print(f"{'PASS' if ok else 'FAIL'} demo; video {video}", flush=True)
  if keep:
    print(f"still running: Isaac pid {stack.pid}; stop with taskkill.exe /PID {stack.pid} /T /F "
          "and sg docker -c 'docker rm -f intrinsic'", flush=True)
  return 0 if ok else 1


if __name__ == "__main__":
  sys.exit(main())
