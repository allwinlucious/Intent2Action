"""Shared e2e helpers: run_scene.py in Isaac and probes in the intrinsic-core:dev image.

Used by tests/e2e/phase2_check.py and tests/e2e/phase3_hwm_check.py.
"""

import json
from pathlib import Path
import re
import shlex
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"
ISAAC_COMMAND = [
    "cmd.exe",
    "/c",
    r"cd /d C:\isaac-sim && python.bat C:\isaac-sim-work\repo\sim\run_scene.py",
]
READY_TIMEOUT_S = 180
PROBE_COMMAND = (
    "docker run --rm --name {name} --network host"
    " -v " + str(ROOT) + "/tests/e2e:/probe:ro"
    " -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/app/container/dds/fastdds.xml"
    " -e CYCLONEDDS_URI=file:///opt/app/container/dds/cyclonedds.xml"
    " -e RMW_IMPLEMENTATION={rmw}"
    " --entrypoint bash intrinsic-core:dev -c {script}"
)
PROBE_SCRIPT = (
    "set +u; source /opt/ros/humble/setup.bash &&"
    " source /opt/ros_ext/setup.bash && python3 /probe/{probe} {args}"
)


def docker(command: str) -> list[str]:
  return ["sg", "docker", "-c", command]


def sync_isaac() -> None:
  subprocess.run([str(ROOT / "tools/sync_isaac.sh")], check=True)


class Isaac:
  """run_scene.py, its output copied to log_path line by line."""

  def __init__(self, log_path: Path):
    log_path.parent.mkdir(exist_ok=True)
    self.lines = []
    self._log = open(log_path, "w")
    self.process = subprocess.Popen(
        ISAAC_COMMAND,
        cwd="/mnt/c",
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
    )
    self._thread = threading.Thread(target=self._read, daemon=True)
    self._thread.start()

  def _read(self) -> None:
    for line in self.process.stdout:
      self.lines.append(line.rstrip())
      self._log.write(line)
      self._log.flush()

  def wait_ready(self, timeout_s: float) -> int | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and self.process.poll() is None:
      for line in self.lines:
        if match := re.match(r"READY pid=(\d+)", line):
          return int(match.group(1))
      time.sleep(0.5)
    return None

  def stop(self, pid: int | None) -> None:
    if pid is not None:
      subprocess.run(
          ["taskkill.exe", "/PID", str(pid), "/T", "/F"], capture_output=True
      )
    try:
      self.process.wait(timeout=30)
    except subprocess.TimeoutExpired:
      self.process.kill()
    self._thread.join(timeout=5)
    self._log.close()


def run_probe(name: str, rmw: str, probe: str, args: str, timeout_s: float):
  """Runs a probe container; returns (last JSON line or None, output).

  The output is written to logs/<name>.log.
  """
  script = PROBE_SCRIPT.format(probe=probe, args=args)
  command = PROBE_COMMAND.format(name=name, rmw=rmw, script=shlex.quote(script))
  subprocess.run(docker(f"docker rm -f {name}"), capture_output=True)
  try:
    done = subprocess.run(
        docker(command),
        capture_output=True,
        text=True,
        errors="replace",
        timeout=timeout_s,
    )
    output = done.stdout + done.stderr
  except subprocess.TimeoutExpired as e:
    subprocess.run(docker(f"docker rm -f {name}"), capture_output=True)
    output = f"timeout after {timeout_s} s\n{e.stdout or ''}{e.stderr or ''}"
  (LOG_DIR / f"{name}.log").write_text(output)
  for line in reversed(output.splitlines()):
    if line.startswith("{"):
      return json.loads(line), output
  return None, output
