"""Starts run_scene.py --gui --pretty alone (no container), saves one viewer frame to logs/demo/preview.png, stops.

  .venv/bin/python tests/e2e/preview_pretty.py   (for tuning sim/pretty.py)
"""

import shutil
import time
import harness

harness.ISAAC_COMMAND[-1] += " --gui --pretty"
harness.sync_isaac()
isaac = harness.Isaac(harness.LOG_DIR / "demo" / "preview_isaac.log")
pid = isaac.wait_ready(900)
try:
  time.sleep(20)  # a few renders after the scene has loaded
  frames = sorted(harness.VIEWER_FRAMES.glob("frame_*.png"))
  shutil.copy(frames[-2], harness.LOG_DIR / "demo" / "preview.png")
  print(f"preview from {len(frames)} frames, pid {pid}", flush=True)
finally:
  isaac.stop(pid)
