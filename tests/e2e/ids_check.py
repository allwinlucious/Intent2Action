"""Phase L-1 check: perception-style object IDs (phase_ids.md, done criteria and Addendum A.2, A.9).

Syncs the repository to Windows, starts sim/run_scene.py and the container like phase3_cuj_check.py, and
prints PASS/FAIL per check:
  .venv/bin/python tests/e2e/ids_check.py
Every call goes to RobotService and SimService on 127.0.0.1:18100. Stops the container and Isaac at the end.
"""

import sys
import time

import grpc

import harness
import phase3_hwm_check as hwm  # sets sys.path for gen/
import phase1_check as p1
import phase3_cuj_check as cuj
import phase3_scene_check as scene
from robot_service import robot_service_pb2 as api  # noqa: E402

ISAAC_LOG = harness.LOG_DIR / "ids_isaac.log"
CONTAINER_LOG = harness.LOG_DIR / "ids_container.log"
NEVER_DRAWN = "obj_00"  # k is drawn from 10-99
NOT_SCENE_OBJECTS = ("franka", "franka_hand", "d455", "calib_board")
RESETS = 5
EXTRA_POSE = api.Pose(x=0.30, y=-0.30, z=0.0, qw=1.0)  # a free spot on the table (as phase3_scene_check.py)


def _listed() -> dict[str, api.WorldObject]:
  return {o.id: o for o in cuj._call(cuj._robot, "ListObjects").objects}


def check_list() -> str:
  objects = list(cuj._call(cuj._robot, "ListObjects").objects)
  defaults = {prim: i for prim, i in cuj.ids(cuj._sim).items() if prim in scene.DEFAULT_OBJECTS}
  problems = []
  if len({o.id for o in objects}) != len(objects):
    problems.append("duplicate ids")
  if set(defaults) != set(scene.DEFAULT_OBJECTS) or not set(defaults.values()) <= {o.id for o in objects}:
    problems.append(f"default prims -> {defaults} not all listed")
  for o in objects:
    # Any string but `category` (which may be e.g. "plate", Addendum A.2) must not be a scene-file name.
    strings = (o.id, o.source, o.attached_to, o.color)
    if (not cuj.ID.fullmatch(o.id) or any(s in scene.DEFAULT_OBJECTS for s in strings) or not o.category
        or not o.color or min(o.size_x, o.size_y, o.size_z) <= 0 or o.source != "ground_truth"
        or not o.HasField("pose")):
      problems.append(f"bad object {o}".replace("\n", " "))
  summary = ", ".join(f"{o.id} {o.color} {o.category} {o.size_x:.3f}x{o.size_y:.3f}x{o.size_z:.3f}" for o in objects)
  if problems:
    raise RuntimeError(f"{'; '.join(problems)} ({summary})")
  return summary


def _not_found(rpc: str, object_id: str, call) -> str:
  """call(object_id) must fail with NOT_FOUND and the standard message (Addendum A.6)."""
  want = f"no object with id {object_id}; list_objects shows the current ids"
  try:
    call(object_id)
  except grpc.RpcError as error:
    if error.code() != grpc.StatusCode.NOT_FOUND or want not in error.details():
      raise RuntimeError(f"{rpc}({object_id}) -> {cuj._code(error)}") from error
    return f"{rpc}({object_id}) -> {cuj._code(error)}"
  raise RuntimeError(f"{rpc}({object_id}) returned OK")


def check_unknown() -> str:
  calls = {
      "PlanGrasp": lambda i: cuj._call(cuj._robot, "PlanGrasp", api.PlanGraspRequest(object_id=i)),
      "AttachObject": lambda i: cuj._call(cuj._robot, "AttachObject", api.ObjectRequest(id=i)),
      "DetachObject": lambda i: cuj._call(cuj._robot, "DetachObject", api.ObjectRequest(id=i)),
      "RemoveObject": lambda i: cuj._call(cuj._sim, "RemoveObject", api.ObjectRequest(id=i)),
      "GetTruePose": lambda i: cuj._call(cuj._sim, "GetTruePose", api.GetTruePoseRequest(frame=i)),
  }
  results = [_not_found(rpc, NEVER_DRAWN, call) for rpc, call in calls.items()]
  results += [_not_found("AttachObject", name, calls["AttachObject"]) for name in NOT_SCENE_OBJECTS]
  return "; ".join(results)


def check_add() -> str:
  added = cuj._call(cuj._sim, "AddObject", api.AddObjectRequest(catalog="red_cube", pose=EXTRA_POSE)).id
  listed = _listed()
  if not cuj.ID.fullmatch(added) or added not in listed or (listed[added].category, listed[added].color) != (
      "cube", "red"):
    raise RuntimeError(f"AddObject -> {added!r}; list_objects {sorted(listed)}")
  cuj._call(cuj._sim, "RemoveObject", api.ObjectRequest(id=added))
  if added in _listed():
    raise RuntimeError(f"{added} still listed after RemoveObject")
  return f"AddObject(red_cube) -> {added}, listed as a red cube; RemoveObject({added}) -> gone from list_objects"


def check_resets() -> str:
  previous, maps = cuj.ids(cuj._sim), []
  for i in range(RESETS):
    start = time.monotonic()
    cuj._call(cuj._sim, "ResetScene")
    took = time.monotonic() - start
    ids = cuj.ids(cuj._sim)
    listed = set(_listed())
    kept = {prim: i for prim, i in ids.items() if previous.get(prim) == i}
    if set(ids) != set(scene.DEFAULT_OBJECTS) or kept or set(ids.values()) != listed or len(listed) != len(ids):
      raise RuntimeError(f"reset {i + 1}: ids {ids} (before {previous}), kept {kept}, listed {sorted(listed)}")
    maps.append(f"{took:.1f} s {sorted(ids.values())}")
    previous = ids
  return f"{RESETS} resets, no object kept its id, all listed when ResetScene returned: " + "; ".join(maps)


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
    checks = (
        ("list", check_list),
        ("unknown", check_unknown),
        ("add", check_add),
        ("resets", check_resets),
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
