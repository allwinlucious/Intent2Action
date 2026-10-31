"""Phase L1 plan-tier check: real planning with instantaneous execution and no Isaac or ICON."""

import math
from pathlib import Path
import socket
import statistics
import sys
import time

import grpc

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]

import phase3_cuj_check as cuj  # noqa: E402
import phase3_hwm_check as hwm  # noqa: E402
import pickplace_check as pickplace  # noqa: E402
from program_runtime.api import GrpcBackend, RobotAPI  # noqa: E402
from program_runtime.runtime import run  # noqa: E402
from robot_service import robot_service_pb2 as api  # noqa: E402

CONTAINER_LOG = ROOT / "logs" / "plan_tier_container.log"
READY_TIMEOUT_S = 180
BOX = pickplace.BOX
BOX_FLOOR_M = 0.01
SPEED_SCENE = [
    BOX,
    ("red_cube", 0.34, 0.03, 0.0),
    ("blue_cube", 0.46, 0.03, 0.0),
    ("green_cube", 0.58, 0.03, 0.0),
    ("red_cube", 0.46, 0.16, 0.0),
]  # 4 cubes: a box takes 4-6 depending on the approach directions (place planner capacity)
PROGRAM = """\
box = find(category="box", color="red")
for cube in find_all(category="cube"):
  pick(cube)
  place(cube, box)
report(f"{len(find_all(category='cube'))} cubes placed in the red box")
verify(lambda: all(inside(c, box) for c in find_all(category="cube")), "a cube is not in the red box")
"""


def _wait_port(port: int, deadline: float) -> None:
  while True:
    try:
      socket.create_connection(("127.0.0.1", port), timeout=1).close()
      return
    except OSError:
      if time.monotonic() > deadline:
        raise TimeoutError(f"port {port} not open after {READY_TIMEOUT_S} s")
      time.sleep(1)


def _wait_ready() -> None:
  deadline = time.monotonic() + READY_TIMEOUT_S
  for port in (18100, 18101):
    _wait_port(port, deadline)
  while True:
    try:
      truth = cuj.ids(cuj._sim)
      listed = {obj.id for obj in cuj._call(cuj._robot, "ListObjects").objects}
      if truth and set(truth.values()) <= listed:
        return
    except grpc.RpcError:
      pass
    if time.monotonic() > deadline:
      raise RuntimeError(f"ListObjects did not return the default scene within {READY_TIMEOUT_S} s")
    time.sleep(0.5)


def check_truth() -> str:
  box_id, cube_id = pickplace._scene([BOX, ("red_cube", 0.50, 0.10, 0.0)])
  pickplace._pick(cube_id)
  pickplace._place(cube_id, box_id)
  true = {obj.id: obj.pose for obj in cuj._call(cuj._sim, "ListTrueObjects").objects}
  box, cube = true[box_id], true[cube_id]
  yaw = math.atan2(2 * (box.qw * box.qz + box.qx * box.qy), 1 - 2 * (box.qy ** 2 + box.qz ** 2))
  dx, dy = cube.x - box.x, cube.y - box.y
  x = math.cos(yaw) * dx + math.sin(yaw) * dy
  y = -math.sin(yaw) * dx + math.cos(yaw) * dy
  if abs(x) > pickplace.INSIDE[0] or abs(y) > pickplace.INSIDE[1]:
    raise RuntimeError(f"cube outside box footprint: ({x:.3f}, {y:.3f})")
  floor_z = box.z + BOX_FLOOR_M
  if abs(cube.z - floor_z) > 1e-6:
    raise RuntimeError(f"cube z {cube.z:.4f}, box floor z {floor_z:.4f}")
  return f"cube inside footprint at box frame ({x:.3f}, {y:.3f}), z={cube.z:.3f} on floor"


class TimedBackend(GrpcBackend):

  def __init__(self):
    super().__init__()
    self.started = {}
    self.per_cube = []

  def pick(self, object_id: str) -> None:
    self.started[object_id] = time.monotonic()
    super().pick(object_id)

  def place(self, object_id: str, target_id: str) -> None:
    super().place(object_id, target_id)
    self.per_cube.append(time.monotonic() - self.started.pop(object_id))


def check_speed() -> str:
  pickplace._scene(SPEED_SCENE)
  backend = TimedBackend()
  result = run(PROGRAM, RobotAPI(backend))
  if not result.ok:
    raise RuntimeError(result.error or "program runtime failed")
  if len(backend.per_cube) != 4:
    raise RuntimeError(f"timed {len(backend.per_cube)} cubes, expected 4")
  median = statistics.median(backend.per_cube)
  print(f"median episode time per cube: {median:.3f} s (target <= 2.000 s; informational)", flush=True)
  return f"4-cube program finished; median wall time per cube {median:.3f} s"


def main() -> int:
  started = time.monotonic()
  hwm.CONTAINER_LOG = CONTAINER_LOG
  stack = hwm._Run()
  results = []
  try:
    stack.start_container(backend="plan")
    _wait_ready()
    print(f"robot_service plan tier ready after {time.monotonic() - started:.1f} s", flush=True)
    checks = (
        ("pick_place_box", pickplace.check_pick_place_box),
        ("stack", pickplace.check_stack),
        ("errors", pickplace.check_errors),
        ("truth", check_truth),
        ("speed", check_speed),
    )
    results = [hwm._run_check(name, check) for name, check in checks]
  except Exception as error:  # pylint: disable=broad-except
    print(f"FAIL setup: {type(error).__name__}: {error}", flush=True)
    results = [False]
  finally:
    stack.stop_container()
  print(f"elapsed {time.monotonic() - started:.1f} s")
  return 0 if results and all(results) else 1


if __name__ == "__main__":
  sys.exit(main())
