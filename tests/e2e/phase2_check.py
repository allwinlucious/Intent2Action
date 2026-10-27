"""Phase 2 check: run_scene.py in Isaac, driven by two container probes.

Syncs the repository to Windows, starts sim/run_scene.py, checks that it runs
alone, then runs tests/e2e/phase2_peer_hwm.py (CycloneDDS) and
tests/e2e/phase2_peer_io.py (Fast DDS) in the intrinsic-core:dev image and
prints PASS/FAIL per check (phase_2.md, "Goal and done criteria"):
  .venv/bin/python tests/e2e/phase2_check.py
Stops Isaac with taskkill at the end.
"""

import json
import math
from pathlib import Path
import re
import shlex
import subprocess
import sys
import threading
import time

import numpy as np
import yaml

_ROOT = Path("/home/mew/untitled-project")
LOG_PATH = _ROOT / "logs/phase2_isaac.log"
SCENE_PATH = _ROOT / "sim/scenes/default.yaml"
OBJECTS_DIR = _ROOT / "assets/objects"
ISAAC_COMMAND = [
    "cmd.exe",
    "/c",
    r"cd /d C:\isaac-sim && python.bat C:\isaac-sim-work\repo\sim\run_scene.py",
]
READY_TIMEOUT_S = 180
STANDALONE_S = 30
HWM_TIMEOUT_S = 300
IO_TIMEOUT_S = 900
CAMERA_FRAME = "Camera_Pseudo_Depth"
# Rotations inside rsd455.usd, as (w, x, y, z): RSD455 under the D455 root,
# then Camera_Pseudo_Depth under RSD455 (both with zero translation; logged by
# C:\isaac-sim-work\phase2\scripts\exp2a_rsd455.py, phase_2_inventory.md §4.1).
RSD455_LOCAL_WXYZ = (0.0, 0.0, 0.0, 1.0)
CAMERA_LOCAL_WXYZ = (0.5, 0.5, 0.5, 0.5)
PROBE_COMMAND = (
    "docker run --rm --name {name} --network host"
    " -v /home/mew/untitled-project/tests/e2e:/probe:ro"
    " -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/app/container/dds/fastdds.xml"
    " -e CYCLONEDDS_URI=file:///opt/app/container/dds/cyclonedds.xml"
    " -e RMW_IMPLEMENTATION={rmw}"
    " --entrypoint bash intrinsic-core:dev -c {script}"
)
PROBE_SCRIPT = (
    "set +u; source /opt/ros/humble/setup.bash &&"
    " source /opt/ros_ext/setup.bash && python3 /probe/{probe} {args}"
)
IO_CHECKS = (
    "gripper",
    "wrench",
    "tf",
    "camera",
    "camera_fresh",
    "entities",
    "reset",
)


def _docker(command: str) -> list[str]:
  return ["sg", "docker", "-c", command]


def _matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
  return np.array(
      [
          [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
      ]
  )


def _rot(axis: int, degrees: float) -> np.ndarray:
  """Rotation about x (0), y (1) or z (2)."""
  c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
  return np.array(
      [
          [[1, 0, 0], [0, c, -s], [0, s, c]],
          [[c, 0, s], [0, 1, 0], [-s, 0, c]],
          [[c, -s, 0], [s, c, 0], [0, 0, 1]],
      ][axis]
  )


def expected_scene() -> dict:
  """Expected /tf poses from sim/scenes/default.yaml, for phase2_peer_io.py."""
  scene = yaml.safe_load(SCENE_PATH.read_text())
  out = {"static": {}, "dynamic": {}}
  for name, spec in scene["objects"].items():
    static = yaml.safe_load(
        (OBJECTS_DIR / spec["catalog"] / "object.yaml").read_text()
    )["static"]
    out["static" if static else "dynamic"][name] = {
        "position": spec["position"],
        "rotation": _rot(2, spec["yaw"]).tolist(),
    }
  # isaacsim.core.utils.rotations.euler_angles_to_quat (extrinsic): Rz Ry Rx.
  ex, ey, ez = scene["camera"]["euler_deg"]
  d455 = _rot(2, ez) @ _rot(1, ey) @ _rot(0, ex)
  out["static"][CAMERA_FRAME] = {
      "position": scene["camera"]["position"],
      # The bridge publishes the camera frame in ROS optical convention:
      # USD camera axes rotated 180 deg about x.
      "rotation": (
          d455
          @ _matrix(*RSD455_LOCAL_WXYZ)
          @ _matrix(*CAMERA_LOCAL_WXYZ)
          @ _rot(0, 180.0)
      ).tolist(),
  }
  return out


class _Isaac:
  """run_scene.py, its output copied to LOG_PATH line by line."""

  def __init__(self):
    LOG_PATH.parent.mkdir(exist_ok=True)
    self.lines = []
    self._log = open(LOG_PATH, "w")
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


def _run_probe(name: str, rmw: str, probe: str, args: str, timeout_s: float):
  """Runs a probe container; returns (last JSON line or None, output)."""
  script = PROBE_SCRIPT.format(probe=probe, args=args)
  command = PROBE_COMMAND.format(name=name, rmw=rmw, script=shlex.quote(script))
  subprocess.run(_docker(f"docker rm -f {name}"), capture_output=True)
  try:
    done = subprocess.run(
        _docker(command),
        capture_output=True,
        text=True,
        errors="replace",
        timeout=timeout_s,
    )
    output = done.stdout + done.stderr
  except subprocess.TimeoutExpired as e:
    subprocess.run(_docker(f"docker rm -f {name}"), capture_output=True)
    output = f"timeout after {timeout_s} s\n{e.stdout or ''}{e.stderr or ''}"
  (LOG_PATH.parent / f"phase2_{name}.log").write_text(output)
  for line in reversed(output.splitlines()):
    if line.startswith("{"):
      return json.loads(line), output
  return None, output


def _standalone(isaac: _Isaac) -> tuple[bool, str]:
  running = subprocess.run(
      _docker("docker ps -q"), capture_output=True, text=True
  ).stdout.split()
  before = len(isaac.lines)
  time.sleep(STANDALONE_S)
  progress = [l for l in isaac.lines[before:] if l.startswith("cycles ")]
  bad = [l for l in isaac.lines if "Error" in l or "Traceback" in l]
  ok = (
      not running
      and len(progress) >= 2
      and not bad
      and isaac.process.poll() is None
  )
  return ok, (
      f"{len(running)} containers running; {len(progress)} progress lines in"
      f" {STANDALONE_S} s (last: {progress[-1] if progress else None});"
      f" {len(bad)} Error/Traceback lines {bad[:3]}"
  )


def main() -> int:
  start = time.monotonic()
  subprocess.run([str(_ROOT / "tools/sync_isaac.sh")], check=True)
  results = {}
  isaac = _Isaac()
  pid = isaac.wait_ready(READY_TIMEOUT_S)
  if pid is None:
    results["standalone"] = (False, f"no READY within {READY_TIMEOUT_S} s")
  else:
    print(f"isaac READY pid={pid}", flush=True)
    results["standalone"] = _standalone(isaac)

    hwm, output = _run_probe(
        "phase2_hwm",
        "rmw_cyclonedds_cpp",
        "phase2_peer_hwm.py",
        "--cycles 2000 --pause-at 1000 --pause-s 3",
        HWM_TIMEOUT_S,
    )
    if hwm is None:
      results["lockstep"] = results["free_run"] = (
          False,
          f"no result: {output[-300:]}",
      )
    else:
      results["lockstep"] = (
          hwm["missed"] == 0 and hwm["max_track_err_rad"] < 0.01,
          f"cycles {hwm['cycles']} missed {hwm['missed']} max tracking error"
          f" {hwm['max_track_err_rad']:.5f} rad rtf {hwm['rtf']}",
      )
      advance, resume = hwm["pause_sim_advance_s"], hwm["resume_cycles"]
      results["free_run"] = (
          advance is not None
          and advance > 0.2
          and resume is not None
          and resume <= 10,
          f"sim advanced {advance} s during the pause; lockstep after"
          f" {resume} cycles",
      )

    io, output = _run_probe(
        "phase2_io",
        "rmw_fastrtps_cpp",
        "phase2_peer_io.py",
        f"--scene {shlex.quote(json.dumps(expected_scene()))}",
        IO_TIMEOUT_S,
    )
    for name in IO_CHECKS:
      if io is None or name not in io:
        results[name] = (False, f"no result: {output[-300:]}")
      else:
        results[name] = (io[name]["pass"], io[name]["detail"])
  isaac.stop(pid)

  order = ("lockstep", "free_run") + IO_CHECKS + ("standalone",)
  for name in order:
    passed, detail = results.get(name, (False, "not run"))
    print(f"{'PASS' if passed else 'FAIL'} {name}: {detail}", flush=True)
  print(f"elapsed {time.monotonic() - start:.1f}")
  return 0 if all(results.get(n, (False,))[0] for n in order) else 1


if __name__ == "__main__":
  sys.exit(main())
