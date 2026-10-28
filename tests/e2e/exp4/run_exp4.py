"""Runs Step 2 experiment scripts (tests/e2e/exp4/) with Isaac and the container up, then stops both:
  .venv/bin/python tests/e2e/exp4/run_exp4.py exp4_cd.py [more scripts]
Each script's output goes to stdout and to logs/phase4_exp_<script stem>.log; exit 0 if every script passed.
"""

from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import harness  # noqa: E402
import phase1_check as p1  # noqa: E402
import phase3_cuj_check as cuj  # noqa: E402
import phase3_hwm_check as hwm  # noqa: E402

HERE = Path(__file__).resolve().parent


def main(scripts: list[str]) -> int:
  harness.sync_isaac()
  hwm.CONTAINER_LOG = harness.LOG_DIR / "phase4_exp_container.log"
  run = hwm._Run()
  failed = 0
  try:
    run.start_isaac(harness.LOG_DIR / "phase4_exp_isaac.log")
    run.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    for script in scripts:
      began = time.monotonic()
      done = subprocess.run([sys.executable, str(HERE / script)], capture_output=True, text=True)
      output = done.stdout + done.stderr
      (harness.LOG_DIR / f"phase4_exp_{Path(script).stem}.log").write_text(output)
      print(f"{script}: exit {done.returncode} after {time.monotonic() - began:.0f} s\n{output}", flush=True)
      failed += done.returncode != 0
  finally:
    run.stop_container()
    run.stop_isaac()
  return 1 if failed else 0


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:]))
