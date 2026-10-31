"""Phase V1 check: object-level Pick and Place, open boxes (phase_v1.md, decision 9 and Addendum A.9).

Syncs the repository to Windows, starts sim/run_scene.py and the container
(ROBOT_BACKEND=isaac, container/run.sh), and prints PASS/FAIL per check, in order:
  .venv/bin/python tests/e2e/pickplace_check.py
Needs the boxes' model.usd (sim/tools/build_catalog.py). Stops the container and Isaac at the end.
"""

import math
import random
import sys
import time

import grpc

import harness
import phase3_hwm_check as hwm  # sets sys.path for gen/ and ICON's Python
import phase1_check as p1
import phase3_cuj_check as cuj
import phase5c_check as p5c
from robot_service import robot_service_pb2 as api  # noqa: E402

ISAAC_LOG = harness.LOG_DIR / "pickplace_check_isaac.log"
CONTAINER_LOG = harness.LOG_DIR / "pickplace_check_container.log"
MOVABLE = ["red_cube", "blue_cube", "green_cube", "small_yellow_cube", "plate", "peg"]
CUBES = ["red_cube", "blue_cube", "green_cube", "small_yellow_cube"]
BOX = ("red_box", 0.38, -0.24, 0.0)
INSIDE = (0.115, 0.095, 0.005, 0.03)  # box frame: |dx|, |dy| at most, z in [min, max]
TRIALS, MIN_SUCCESSES = 10, 9
STACK_TRIALS, STACK_MIN_SUCCESSES = 5, 4
STACK_XY_M = 0.015
STACK_DZ = (0.045, 0.06)
LISTED_TIMEOUT_S = 20.0
_log_start = [0]


def _yaw(p: api.Pose) -> float:
  return math.atan2(2 * (p.qw * p.qz + p.qx * p.qy), 1 - 2 * (p.qy ** 2 + p.qz ** 2))


def _true(object_id: str) -> api.Pose:
  return cuj._call(cuj._sim, "GetTruePose", api.GetTruePoseRequest(frame=object_id))


def _scene(objects) -> list[str]:
  """ResetScene, every default movable object removed, then `objects` ((catalog, x, y, yaw)) added; their ids."""
  cuj._call(cuj._sim, "ResetScene")
  cuj._wait_ready()
  ids = cuj.ids(cuj._sim)
  for prim in MOVABLE:
    cuj._call(cuj._sim, "RemoveObject", api.ObjectRequest(id=ids[prim]))
  added = []
  for catalog, x, y, yaw in objects:
    pose = api.Pose(x=x, y=y, z=0.0, qz=math.sin(yaw / 2), qw=math.cos(yaw / 2))
    added.append(cuj._call(cuj._sim, "AddObject", api.AddObjectRequest(catalog=catalog, pose=pose)).id)
  deadline = time.monotonic() + LISTED_TIMEOUT_S
  while not set(added) <= {o.id for o in cuj._call(cuj._robot, "ListObjects").objects}:  # from the next /tf
    if time.monotonic() > deadline:
      raise RuntimeError(f"objects {added} not all in ListObjects after {LISTED_TIMEOUT_S:.0f} s")
    time.sleep(0.5)
  return added


def _pick(object_id: str) -> api.PickResult:
  return cuj._done(cuj._call(cuj._robot, "Pick", api.PickRequest(object_id=object_id, wait=True)), api.PickResult)


def _place(object_id: str, target_id: str) -> api.PlaceResult:
  request = api.PlaceRequest(object_id=object_id, target_id=target_id, wait=True)
  return cuj._done(cuj._call(cuj._robot, "Place", request), api.PlaceResult)


def _in_box(object_id: str, box_id: str) -> tuple[float, float, float]:
  """The object's true pose in the box frame; raises if it is not inside."""
  box, p = _true(box_id), _true(object_id)
  a, dx, dy = _yaw(box), p.x - box.x, p.y - box.y
  x, y, z = math.cos(a) * dx + math.sin(a) * dy, -math.sin(a) * dx + math.cos(a) * dy, p.z - box.z
  if not (abs(x) <= INSIDE[0] and abs(y) <= INSIDE[1] and INSIDE[2] <= z <= INSIDE[3]):
    raise RuntimeError(f"{object_id} not inside {box_id}: box frame ({x:.3f}, {y:.3f}, {z:.3f})")
  return x, y, z


def _error(call) -> grpc.RpcError:
  try:
    result = call()
  except grpc.RpcError as error:
    return error
  raise RuntimeError(f"returned OK: {result}")


def check_pick_place_box() -> str:
  successes = 0
  for trial in range(TRIALS):
    rng = random.Random(trial)
    cube = (CUBES[trial % len(CUBES)], rng.uniform(0.35, 0.60), rng.uniform(0.00, 0.20), rng.uniform(-0.4, 0.4))
    try:
      box_id, cube_id = _scene([BOX, cube])
      picked = _pick(cube_id)
      placed = _place(cube_id, box_id)
      x, y, z = _in_box(cube_id, box_id)
      successes += 1
      print(f"  trial {trial + 1} {cube[0]}: ok, box frame ({x:.3f}, {y:.3f}, {z:.3f}); pick {picked.duration_s:.1f} s,"
            f" place {placed.duration_s:.1f} s", flush=True)
    except (grpc.RpcError, RuntimeError) as error:
      print(f"  trial {trial + 1} {cube[0]}: FAILED {p1._describe(error)}", flush=True)
  if successes < MIN_SUCCESSES:
    raise RuntimeError(f"{successes}/{TRIALS} placed in the box (need {MIN_SUCCESSES})")
  return f"{successes}/{TRIALS} placed in the box"


def check_fill_box() -> str:
  box_id, *cube_ids = _scene([BOX, ("red_cube", 0.38, 0.05, 0.0), ("blue_cube", 0.52, 0.05, 0.0),
                              ("green_cube", 0.38, 0.18, 0.0), ("small_yellow_cube", 0.52, 0.18, 0.0)])
  for cube_id in cube_ids:
    _pick(cube_id)
    _place(cube_id, box_id)
  spots = [_in_box(cube_id, box_id) for cube_id in cube_ids]
  return "4/4 inside, box frame " + ", ".join(f"({x:.3f}, {y:.3f}, {z:.3f})" for x, y, z in spots)


def check_stack() -> str:
  successes = 0
  for trial in range(STACK_TRIALS):
    try:
      red_id, yellow_id = _scene([("red_cube", 0.50, 0.05, 0.0), ("small_yellow_cube", 0.40, 0.17, 0.0)])
      _pick(yellow_id)
      _place(yellow_id, red_id)
      red, yellow = _true(red_id), _true(yellow_id)
      dxy, dz = math.hypot(yellow.x - red.x, yellow.y - red.y), yellow.z - red.z
      detail = f"xy {dxy * 1e3:.1f} mm, dz {dz * 1e3:.1f} mm"
      if dxy > STACK_XY_M or not STACK_DZ[0] <= dz <= STACK_DZ[1]:
        raise RuntimeError(detail)
      successes += 1
      print(f"  stack trial {trial + 1}: ok, {detail}", flush=True)
    except (grpc.RpcError, RuntimeError) as error:
      print(f"  stack trial {trial + 1}: FAILED {p1._describe(error)}", flush=True)
  if successes < STACK_MIN_SUCCESSES:
    raise RuntimeError(f"{successes}/{STACK_TRIALS} stacked (need {STACK_MIN_SUCCESSES})")
  return f"{successes}/{STACK_TRIALS} stacked"


def check_errors() -> str:
  scene = [("plate", 0.50, 0.12, 0.0), ("red_cube", 0.38, -0.02, 0.0)]
  plate_id, cube_id = _scene(scene)
  results = []

  def expect(label: str, code: grpc.StatusCode, call, text: str = "") -> None:
    error = _error(call)
    if error.code() != code or text not in error.details():
      raise RuntimeError(f"{label} -> {cuj._code(error)} (want {code.name}{f' with {text!r}' if text else ''})")
    results.append(f"{label} -> {error.code().name}")

  expect("Place holding nothing", grpc.StatusCode.FAILED_PRECONDITION, lambda: _place(cube_id, plate_id))
  _pick(cube_id)
  expect("Pick while holding", grpc.StatusCode.FAILED_PRECONDITION, lambda: _pick(plate_id))
  expect("Place(cube, cube)", grpc.StatusCode.INVALID_ARGUMENT, lambda: _place(cube_id, cube_id))
  expect("Pick('obj_999')", grpc.StatusCode.NOT_FOUND, lambda: _pick("obj_999"))
  plate_id, _ = _scene(scene)  # ResetScene releases and detaches the cube
  expect("Pick(plate)", grpc.StatusCode.ABORTED, lambda: _pick(plate_id), "0.075")
  return "; ".join(results)


def check_logs() -> str:
  lines = p5c._log_lines("robot_service")[_log_start[0]:]
  found = {}
  for kind in ("pick", "place"):
    found[kind] = [line for line in lines if f"{kind} object=" in line and "status=" in line]
    if not found[kind]:
      raise RuntimeError(f"no '{kind} object=... status=' line in robot_service.log")
  return "; ".join(f"{len(v)} {k} lines, e.g. {v[-1][:200]!r}" for k, v in found.items())


def main() -> int:
  start = time.monotonic()
  harness.sync_isaac()
  hwm.CONTAINER_LOG = CONTAINER_LOG
  run = hwm._Run()
  results = []
  try:
    run.start_isaac(ISAAC_LOG)
    run.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    print(f"robot_service ready after {time.monotonic() - start:.1f} s", flush=True)
    _log_start[0] = len(p5c._log_lines("robot_service"))
    checks = (
        ("pick_place_box", check_pick_place_box),
        ("fill_box", check_fill_box),
        ("stack", check_stack),
        ("errors", check_errors),
        ("logs", check_logs),
    )
    results = [hwm._run_check(name, check) for name, check in checks]
  except Exception as error:  # pylint: disable=broad-except
    print(f"FAIL setup: {p1._describe(error)}", flush=True)
    results = [False]
  finally:
    run.stop_container()
    run.stop_isaac()
  print(f"elapsed {time.monotonic() - start:.1f}")
  return 0 if results and all(results) else 1


if __name__ == "__main__":
  sys.exit(main())
