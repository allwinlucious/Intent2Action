"""Phase V2 program runtime check; requires the simulator and robot stack."""

import json
import math
from pathlib import Path
import random
import sys
import time

import harness
import phase1_check as p1
import phase3_cuj_check as cuj
import phase3_hwm_check as hwm
from robot_service import robot_service_pb2 as service_pb2

from program_runtime.agent import ScriptedAgent
from program_runtime.api import GrpcBackend, RobotAPI, RobotFailure
from program_runtime.rules import check_intent, truth_snapshot
from program_runtime.runtime import run


ROOT = Path(__file__).resolve().parents[2]
EXAMPLES = ROOT / "program_runtime" / "examples"
ISAAC_LOG = harness.LOG_DIR / "program_check_isaac.log"
CONTAINER_LOG = harness.LOG_DIR / "program_check_container.log"
MOVABLE = ["red_cube", "blue_cube", "green_cube", "small_yellow_cube", "plate", "peg"]
SCENES = {
  "cubes_in_box": ["red_cube", "blue_cube", "small_yellow_cube"],
  "sort_by_color": ["red_cube", "blue_cube", "green_cube"],
  "yellow_on_red": ["red_cube", "small_yellow_cube", "blue_cube"],
}
BOXES = {
  "cubes_in_box": [("red_box", 0.38, -0.24, 0.0)],
  "sort_by_color": [("red_box", 0.38, -0.24, 0.0), ("blue_box", 0.66, -0.20, 0.0)],
  "yellow_on_red": [],
}


def _layouts(seed: int, catalogs: list[str]) -> list[tuple[str, float, float, float]]:
  rng = random.Random(seed)
  placed = []
  for catalog in catalogs:
    while True:
      x = rng.uniform(0.35, 0.60)
      y = rng.uniform(0.00, 0.20)
      yaw = rng.uniform(-0.3, 0.3)
      if all(math.hypot(x - other_x, y - other_y) >= 0.08 for _, other_x, other_y, _ in placed):
        placed.append((catalog, x, y, yaw))
        break
  return placed


def _scene(name: str, layout: int) -> None:
  cuj._call(cuj._sim, "ResetScene")
  cuj._wait_ready()
  ids = cuj.ids(cuj._sim)
  for catalog in MOVABLE:
    cuj._call(cuj._sim, "RemoveObject", service_pb2.ObjectRequest(id=ids[catalog]))
  added = []
  for catalog, x, y, yaw in [*_layouts(layout, SCENES[name]), *BOXES[name]]:
    pose = service_pb2.Pose(x=x, y=y, z=0.0, qz=math.sin(yaw / 2), qw=math.cos(yaw / 2))
    response = cuj._call(
      cuj._sim, "AddObject", service_pb2.AddObjectRequest(catalog=catalog, pose=pose)
    )
    added.append(response.id)
  deadline = time.monotonic() + 20
  while time.monotonic() < deadline:
    listed = {obj.id for obj in cuj._call(cuj._robot, "ListObjects").objects}
    if set(added) <= listed:
      return
    time.sleep(0.5)
  raise RuntimeError("added objects did not appear in ListObjects")


def _log(name: str):
  path = ROOT / "logs" / "runs" / f"program_check_{name}.jsonl"
  path.parent.mkdir(parents=True, exist_ok=True)
  stream = path.open("w")

  def write(event):
    stream.write(json.dumps(event, separators=(",", ":")) + "\n")
    stream.flush()

  return stream, write


def _source(name: str) -> tuple[dict, str]:
  return json.loads((EXAMPLES / f"{name}.json").read_text()), (EXAMPLES / f"{name}.py").read_text()


def check_scripted() -> str:
  successes = 0
  for name in SCENES:
    intent, source = _source(name)
    for layout in range(3):
      _scene(name, layout)
      before = truth_snapshot(cuj._sim)
      stream, logger = _log(f"{name}_{layout}")
      try:
        result = run(source, RobotAPI(GrpcBackend()), log=logger)
      finally:
        stream.close()
      rules = check_intent(intent, before, truth_snapshot(cuj._sim))
      passed = result.ok and rules.ok
      successes += passed
      print(
        f"  {'PASS' if passed else 'FAIL'} {name} layout {layout}: "
        f"runtime={result.ok} rules={rules.summary}", flush=True,
      )
  if successes < 8:
    raise RuntimeError(f"{successes}/9 programs passed, need 8")
  return f"{successes}/9 programs passed"


def _injected(agent: ScriptedAgent, tag: str):
  name = "cubes_in_box"
  intent, source = _source(name)
  _scene(name, 0)
  before = truth_snapshot(cuj._sim)
  backend = GrpcBackend()
  backend.inject = {"pick": [None, RobotFailure("grasp", "grasp blocked (injected)")]}
  stream, logger = _log(tag)
  try:
    result = run(source, RobotAPI(backend), agent=agent, log=logger)
  finally:
    stream.close()
  rules = check_intent(intent, before, truth_snapshot(cuj._sim))
  return result, rules


def check_recovery() -> str:
  result, rules = _injected(
    ScriptedAgent([{"rung": "retry_step", "reason": "transient"}]), "recovery"
  )
  if not (
      result.ok and len(result.handoffs) == 1
      and result.handoffs[0]["outcome"] == "resolved" and rules.ok
  ):
    raise RuntimeError(
      f"runtime={result.ok} handoffs={result.handoffs} rules={rules.summary} error={result.error}"
    )
  return "one handoff resolved and rules passed"


def check_stop() -> str:
  result, _ = _injected(ScriptedAgent([{"rung": "stop", "reason": "test"}]), "stop")
  if result.ok or len(result.handoffs) != 1 or not result.failures:
    raise RuntimeError(
      f"runtime={result.ok} handoffs={result.handoffs} failures={result.failures}"
    )
  return "one handoff stopped with a logged failure"


def main() -> int:
  start = time.monotonic()
  harness.sync_isaac()
  hwm.CONTAINER_LOG = CONTAINER_LOG
  stack = hwm._Run()
  results = []
  try:
    stack.start_isaac(ISAAC_LOG)
    stack.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    checks = (("scripted", check_scripted), ("recovery", check_recovery), ("stop", check_stop))
    results = [hwm._run_check(name, check) for name, check in checks]
  except Exception as error:
    print(f"FAIL setup: {p1._describe(error)}", flush=True)
    results = [False]
  finally:
    stack.stop_container()
    stack.stop_isaac()
  print(f"elapsed {time.monotonic() - start:.1f} s")
  return 0 if results and all(results) else 1


if __name__ == "__main__":
  sys.exit(main())
