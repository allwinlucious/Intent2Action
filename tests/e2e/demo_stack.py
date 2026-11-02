"""Brings up the stack for the demo UI and keeps it running: the Isaac viewer window (real render, native camera
controls) and the robot container, then waits until stopped (SIGTERM / Ctrl-C), and shuts both down.

  .venv/bin/python tests/e2e/demo_stack.py [--no-pretty] [scene_id]

Then, in another shell: python -m demo_ui.server --port 8765, and open http://127.0.0.1:8765/?panel beside the
Isaac window. scene_id (e.g. cubes_in_box:0) is loaded through the demo UI, not here.
"""

import signal
import sys
import threading
import time

import harness
import phase1_check as p1
import phase3_cuj_check as cuj
import phase3_hwm_check as hwm  # sets sys.path for gen/

ISAAC_LOG = harness.LOG_DIR / "demo_stack_isaac.log"
CONTAINER_LOG = harness.LOG_DIR / "demo_stack_container.log"


def main() -> int:
  harness.ISAAC_COMMAND[-1] += " --gui" + ("" if "--no-pretty" in sys.argv else " --pretty")
  harness.READY_TIMEOUT_S = 900
  harness.sync_isaac()
  hwm.CONTAINER_LOG = CONTAINER_LOG
  stack = hwm._Run()
  stop = threading.Event()
  signal.signal(signal.SIGTERM, lambda *_: stop.set())
  start = time.monotonic()
  try:
    stack.start_isaac(ISAAC_LOG)
    stack.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    print(f"STACK READY after {time.monotonic() - start:.0f} s (robot_service 127.0.0.1:18100)", flush=True)
    while not stop.is_set():
      if stack.isaac and stack.isaac.process.poll() is not None:
        print("Isaac exited", flush=True)
        return 1
      stop.wait(2)
  except KeyboardInterrupt:
    pass
  except Exception as error:  # pylint: disable=broad-except
    print(f"FAIL setup: {p1._describe(error)}", flush=True)
    return 1
  finally:
    stack.stop_container()
    stack.stop_isaac()
    print("stack stopped", flush=True)
  return 0


if __name__ == "__main__":
  sys.exit(main())
