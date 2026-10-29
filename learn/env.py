"""The sim stack for the learning experiments: start/stop Isaac + container, random scenes, ground truth.

Reuses the e2e harness (tests/e2e). Every episode starts with SimService.ResetScene, then the movable objects
are removed and re-added at random poses, so ids and positions change per episode.
"""

import math
import random
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests/e2e"))
sys.path.insert(0, str(ROOT / "gen"))

import harness  # noqa: E402
import phase1_check as p1  # noqa: E402
import phase3_cuj_check as cuj  # noqa: E402
import phase3_hwm_check as hwm  # noqa: E402
from robot_service import robot_service_pb2 as api  # noqa: E402

LOG_DIR = ROOT / "logs/learn"
# The supervisor stops a step with SIGTERM; as SystemExit, `with Stack(...)` still stops Isaac and the container.
signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
CUBES = ["red_cube", "blue_cube", "green_cube", "small_yellow_cube"]
CUBE_SIZE = {"red_cube": 0.05, "blue_cube": 0.05, "green_cube": 0.05, "small_yellow_cube": 0.035}
MOVABLE = CUBES + ["plate", "peg"]
HOLE_BLOCK_XY = (0.60, 0.30)
# Reachable, collision-free sampling region on the table (base frame).
X_RANGE, Y_RANGE = (0.35, 0.65), (-0.30, 0.25)
RADIUS = {"plate": 0.09, "peg": 0.03, **{c: 0.04 for c in CUBES}}
CLEARANCE_M = 0.03


class Stack:
  """Isaac + the container, started once for many episodes."""

  def __init__(self, tag: str):
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    hwm.CONTAINER_LOG = LOG_DIR / f"{tag}_container.log"
    self.run = hwm._Run()
    self.tag = tag

  def __enter__(self):
    if "--physics-only" not in harness.ISAAC_COMMAND[-1]:
      harness.ISAAC_COMMAND[-1] += " --physics-only"
    harness.sync_isaac()
    self.run.start_isaac(LOG_DIR / f"{self.tag}_isaac.log")
    self.run.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    return self

  def __exit__(self, *exc):
    self.run.stop_container()
    self.run.stop_isaac()


def gpu_used_mib() -> int:
  out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                       capture_output=True, text=True).stdout
  return int(out.split()[0])


def _yaw_pose(x: float, y: float, yaw: float) -> api.Pose:
  return api.Pose(x=x, y=y, z=0.0, qz=math.sin(yaw / 2), qw=math.cos(yaw / 2))


def _sample(rng: random.Random, names: list[str]) -> dict[str, tuple[float, float, float]]:
  for _ in range(50):  # a fresh layout when one gets stuck
    try:
      return _sample_once(rng, names)
    except RuntimeError:
      pass
  raise RuntimeError(f"no layout for {names}")


def _sample_once(rng: random.Random, names: list[str]) -> dict[str, tuple[float, float, float]]:
  """Random non-overlapping layout; the peg first (its region is small), so it always finds a spot."""
  names = sorted(names, key=lambda n: n != "peg")
  placed = {"hole_block": (*HOLE_BLOCK_XY, 0.0)}
  radius = {**RADIUS, "hole_block": 0.06}
  for name in names:
    for _ in range(500):
      x, y = rng.uniform(*X_RANGE), rng.uniform(*Y_RANGE)
      if name == "peg":  # the peg lives near the hole block, as in the insert task
        x, y = rng.uniform(0.30, 0.44), rng.uniform(0.18, 0.32)
      if all(math.hypot(x - px, y - py) > radius[name] + radius[other] + CLEARANCE_M
             for other, (px, py, _) in placed.items()):
        placed[name] = (x, y, rng.uniform(-math.pi / 4, math.pi / 4))
        break
    else:
      raise RuntimeError(f"no free spot for {name}")
  del placed["hole_block"]
  return placed


def new_scene(rng: random.Random, names: list[str]) -> dict[str, str]:
  """Resets the scene and lays out `names` (catalog names) at random; returns catalog -> object id."""
  cuj._call(cuj._sim, "ResetScene")
  cuj._wait_ready()
  ids = cuj.ids(cuj._sim)  # prim (= catalog name in the default scene) -> id
  for prim in MOVABLE:
    cuj._call(cuj._sim, "RemoveObject", api.ObjectRequest(id=ids[prim]))
  out = {}
  for name, (x, y, yaw) in _sample(rng, names).items():
    out[name] = cuj._call(cuj._sim, "AddObject", api.AddObjectRequest(catalog=name, pose=_yaw_pose(x, y, yaw))).id
  deadline = time.monotonic() + 20
  while time.monotonic() < deadline:  # the world model sees new objects from the next /tf
    listed = {o.id for o in cuj._call(cuj._robot, "ListObjects").objects}
    if set(out.values()) <= listed:
      return out
    time.sleep(0.5)
  raise RuntimeError(f"objects {out} not all in list_objects after 20 s")


def true_pose(frame: str) -> api.Pose:
  return cuj._call(cuj._sim, "GetTruePose", api.GetTruePoseRequest(frame=frame))


def robot_state() -> api.GetStateResponse:
  return cuj._state()
