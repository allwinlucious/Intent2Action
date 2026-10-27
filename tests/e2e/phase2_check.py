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
import shlex
import subprocess
import sys
import time

import numpy as np
import yaml

from harness import docker, Isaac, READY_TIMEOUT_S, ROOT, run_probe, sync_isaac

LOG_PATH = ROOT / "logs/phase2_isaac.log"
SCENE_PATH = ROOT / "sim/scenes/default.yaml"
OBJECTS_DIR = ROOT / "assets/objects"
STANDALONE_S = 30
HWM_TIMEOUT_S = 300
IO_TIMEOUT_S = 900
CAMERA_FRAME = "Camera_Pseudo_Depth"
# Rotations inside rsd455.usd, as (w, x, y, z): RSD455 under the D455 root,
# then Camera_Pseudo_Depth under RSD455 (both with zero translation; logged by
# C:\isaac-sim-work\phase2\scripts\exp2a_rsd455.py, phase_2_inventory.md §4.1).
RSD455_LOCAL_WXYZ = (0.0, 0.0, 0.0, 1.0)
CAMERA_LOCAL_WXYZ = (0.5, 0.5, 0.5, 0.5)
IO_CHECKS = (
    "gripper",
    "wrench",
    "tf",
    "camera",
    "camera_fresh",
    "entities",
    "reset",
)


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


def _standalone(isaac: Isaac) -> tuple[bool, str]:
  running = subprocess.run(
      docker("docker ps -q"), capture_output=True, text=True
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
  sync_isaac()
  results = {}
  isaac = Isaac(LOG_PATH)
  pid = isaac.wait_ready(READY_TIMEOUT_S)
  if pid is None:
    results["standalone"] = (False, f"no READY within {READY_TIMEOUT_S} s")
  else:
    print(f"isaac READY pid={pid}", flush=True)
    results["standalone"] = _standalone(isaac)

    hwm, output = run_probe(
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
      # No reset has run yet, so the latched epoch is the start-up one.
      results["lockstep"] = (
          hwm["missed"] == 0
          and hwm["max_track_err_rad"] < 0.01
          and hwm["epoch_seen"] == 0,
          f"cycles {hwm['cycles']} missed {hwm['missed']} max tracking error"
          f" {hwm['max_track_err_rad']:.5f} rad rtf {hwm['rtf']}; started"
          f" after {hwm['start_pairs']} replies; latched epoch"
          f" {hwm['epoch_seen']}",
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

    io, output = run_probe(
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
