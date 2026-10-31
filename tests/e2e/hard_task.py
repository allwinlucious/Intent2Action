"""A physically hard task: every cube into the red box, from a "plus" of five cubes with 1.5 cm gaps.

The program is trivial by the API docs; the physics is not. A Franka finger needs ~2.2 cm free on both sides of a
cube along its closing direction, so the four outer cubes can be grasped only along their free axis, and the centre
cube only after a pair of opposite neighbours is gone: the pick order decides. (A 2 x 3 cluster with these gaps is
impossible in any order without pushing; tried first.)

  .venv/bin/python tests/e2e/hard_task.py write                    # the model writes the program (LLM_* from .env)
  .venv/bin/python tests/e2e/hard_task.py sim [--gui] [--recover]  # runs logs/hard_task/program.py in Isaac;
      --gui: viewer + video; --recover: the model is the recovery agent (handoffs on failures)

Results: logs/hard_task/ (program.py, the run log, rules, and with --gui video.mp4 + end.png).
"""

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "tests/e2e")]
OUT = ROOT / "logs" / "hard_task"
INTENT = {"description": "every cube goes into the red box",
          "rules": [{"select": {"category": "cube"}, "relation": "in", "target": {"category": "box", "color": "red"}}],
          "others": "stay"}
PITCH_M = 0.065  # cube 0.05 m: 1.5 cm gaps
CENTER = (0.50, 0.08)
# (catalog, dx, dy) in units of PITCH_M around CENTER
PLUS = [("green_cube", 0, 0), ("red_cube", 0, 1), ("red_cube", 0, -1), ("blue_cube", 1, 0), ("blue_cube", -1, 0)]
BOX = ("red_box", 0.40, -0.25, 0.0)


def cluster_poses() -> list[tuple[str, float, float, float]]:
  return [(catalog, CENTER[0] + dx * PITCH_M, CENTER[1] + dy * PITCH_M, 0.0) for catalog, dx, dy in PLUS]


def write() -> None:
  from program_runtime.agent import ChatClient, ProgramWriter
  from program_runtime.api import FakeBackend, Obj, RobotAPI
  from program_runtime.runtime import run
  colors = {"red_cube": "red", "blue_cube": "blue", "green_cube": "green"}
  objects = [Obj(id=f"obj_{20 + i}", category="cube", color=colors[c], size="medium", x=x, y=y, z=0.0, yaw=0.0,
                 sx=0.05, sy=0.05, sz=0.05, attached=False) for i, (c, x, y, _) in enumerate(cluster_poses())]
  objects.append(Obj(id="obj_90", category="box", color="red", size="large", x=BOX[1], y=BOX[2], z=0.0, yaw=0.0,
                     sx=0.25, sy=0.21, sz=0.05, attached=False))
  writer = ProgramWriter(ChatClient())
  program = writer.write(INTENT, objects)
  OUT.mkdir(parents=True, exist_ok=True)
  (OUT / "program.py").write_text(program)
  result = run(program, RobotAPI(FakeBackend(objects)))
  print(program)
  print(f"attempts {writer.last_attempts}; fake robot: ok={result.ok} error={result.error}")


def sim(gui: bool, recover: bool) -> int:
  import harness
  import phase1_check as p1
  import phase3_cuj_check as cuj
  import phase3_hwm_check as hwm
  from robot_service import robot_service_pb2 as api
  from program_runtime.api import GrpcBackend, RobotAPI
  from program_runtime.rules import check_intent, truth_snapshot
  from program_runtime.runtime import run
  OUT.mkdir(parents=True, exist_ok=True)
  if gui:
    harness.ISAAC_COMMAND[-1] += " --gui --pretty"
    harness.READY_TIMEOUT_S = 900
  else:
    harness.ISAAC_COMMAND[-1] += " --physics-only"
  harness.sync_isaac()
  hwm.CONTAINER_LOG = OUT / "container.log"
  stack = hwm._Run()
  ok = False
  try:
    stack.start_isaac(OUT / "isaac.log")
    stack.start_container()
    p1._wait_for_ports()
    hwm._require_state("ENABLED", hwm.ENABLED_TIMEOUT_S)
    cuj._wait_ready()
    cuj._call(cuj._sim, "ResetScene")
    cuj._wait_ready()
    ids = cuj.ids(cuj._sim)
    for catalog in ["red_cube", "blue_cube", "green_cube", "small_yellow_cube", "plate", "peg"]:
      cuj._call(cuj._sim, "RemoveObject", api.ObjectRequest(id=ids[catalog]))
    added = []
    for catalog, x, y, yaw in [*cluster_poses(), BOX]:
      pose = api.Pose(x=x, y=y, z=0.0, qz=math.sin(yaw / 2), qw=math.cos(yaw / 2))
      added.append(cuj._call(cuj._sim, "AddObject", api.AddObjectRequest(catalog=catalog, pose=pose)).id)
    import time
    deadline = time.monotonic() + 30
    while {o.id for o in cuj._call(cuj._robot, "ListObjects").objects} < set(added) and time.monotonic() < deadline:
      time.sleep(0.5)
    before = truth_snapshot(cuj._sim)
    events = open(OUT / f"run{'_recover' if recover else ''}.jsonl", "w")
    log = lambda e: (events.write(json.dumps(e) + "\n"), events.flush())  # noqa: E731
    agent = None
    if recover:
      import grpc
      from mcp_server.server import DESCRIPTOR_SET, Tools
      from program_runtime.agent import ChatClient, RecoveryAgent
      from robot_service import robot_service_pb2_grpc
      stub = robot_service_pb2_grpc.RobotServiceStub(grpc.insecure_channel("127.0.0.1:18100"))
      agent = RecoveryAgent(ChatClient(), stub, Tools(DESCRIPTOR_SET))
    result = run((OUT / "program.py").read_text(), RobotAPI(GrpcBackend()), agent=agent, log=log)
    events.close()
    rules = check_intent(INTENT, before, truth_snapshot(cuj._sim))
    ok = result.ok and rules.ok
    (OUT / f"rules{'_recover' if recover else ''}.json").write_text(json.dumps(
        {"ok": rules.ok, "summary": rules.summary, "per_object": rules.per_object, "runtime_ok": result.ok,
         "error": result.error, "handoffs": result.handoffs}, indent=1))
    print(f"runtime ok={result.ok} error={result.error}\nrules: {rules.summary} {rules.per_object}", flush=True)
  finally:
    stack.stop_container()
    stack.stop_isaac()
  if gui:
    import shutil
    import demo_gui
    import imageio.v2 as imageio
    frames = sorted(demo_gui.FRAMES.glob("frame_*.png"))
    if frames:
      with imageio.get_writer(OUT / f"video{'_recover' if recover else ''}.mp4", fps=15, codec="libx264", quality=8, macro_block_size=8) as writer:
        for frame in frames:
          try:
            writer.append_data(imageio.imread(frame)[..., :3])
          except (OSError, ValueError):
            continue
      shutil.copy(frames[-1], OUT / f"end{'_recover' if recover else ''}.png")
  print(f"{'PASS' if ok else 'FAIL'} hard task", flush=True)
  return 0 if ok else 1


if __name__ == "__main__":
  if sys.argv[1:2] == ["write"]:
    write()
  else:
    sys.exit(sim("--gui" in sys.argv, "--recover" in sys.argv))
