"""Phase 1 check: the Franka is in the world, the planner and ICON drive it.

Run against a container started by container/run.sh:
  .venv/bin/python tests/e2e/phase1_check.py
After `docker restart intrinsic`:
  .venv/bin/python tests/e2e/phase1_check.py --after-restart
`fk_20` needs Isaac Sim's Python on Windows (cmd.exe); the rest only the container.
"""

import argparse
import contextlib
import json
from pathlib import Path
import random
import socket
import subprocess
import sys
import time

import grpc
import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [
    str(_ROOT),
    str(_ROOT / "gen"),
    "/home/mew/intrinsic-core/intrinsic_control",
]
from intrinsic.icon.actions import point_to_point_move_utils  # noqa: E402
from intrinsic.icon.actions import trajectory_tracking_action_utils  # noqa: E402
from intrinsic.icon.proto import joint_space_pb2  # noqa: E402
from intrinsic.icon.proto.v1 import service_pb2 as icon_pb  # noqa: E402
from intrinsic.icon.proto.v1 import service_pb2_grpc as icon_grpc  # noqa: E402
from intrinsic.icon.python import _session  # noqa: E402
from intrinsic.motion_planning.proto.v1 import geometric_constraints_pb2 as gc_pb  # noqa: E402
from intrinsic.motion_planning.proto.v1 import motion_planner_service_pb2 as planner_pb  # noqa: E402
from intrinsic.motion_planning.proto.v1 import motion_planner_service_pb2_grpc as planner_grpc  # noqa: E402
from intrinsic.motion_planning.proto.v1 import motion_specification_pb2 as spec_pb  # noqa: E402
from intrinsic.motion_planning.proto.v1 import robot_specification_pb2 as robot_pb  # noqa: E402
from intrinsic.world.proto import object_world_refs_pb2 as refs_pb  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2 as world_pb  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2_grpc as world_grpc  # noqa: E402

from robot_model import franka_scene_object as fso  # noqa: E402

HOST = "127.0.0.1"
WORLD_PORT, PLANNER_PORT, ICON_PORT = 18000, 18002, 18003
DEADLINE_S = 5
MOTION_DEADLINE_S = 60
MOVE_TIMEOUT_S = 30
ICON_WAIT_S = 180
WORLD_ID = "world"
DEFAULT_Q = [0.0, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75]  # Lula default_q
READY = list(fso.HOME)
FK_TOLERANCE = (1e-3, 0.1)  # metres, degrees (G3)
IK_TOLERANCE = (1e-4, 0.01)
JOINT_TOLERANCE_RAD = 1e-3
FK_SAMPLES = 20
URDF_PATH = _ROOT / "assets/robots/franka/lula_franka_gen.urdf"
ISAAC_WORK_DIR = Path("/mnt/c/isaac-sim-work")
ISAAC_SCRIPT = Path(__file__).with_name("isaac_lula_fk.py")
ISAAC_TIMEOUT_S = 120

# Lula FK in the base_link frame (phase_1_inventory.md §6.2), row-major rotation.
FK_REFERENCE = (
    (
        DEFAULT_Q,
        "panda_link8",
        [0.226113, 0, 0.519254],
        [
            [0.66508, -0.619586, 0.416871],
            [-0.681639, -0.731689, 0],
            [0.30502, -0.284155, -0.908966],
        ],
    ),
    (
        DEFAULT_Q,
        "right_gripper",
        [0.2678, 0, 0.428358],
        [
            [-0.908396, -0.032169, 0.416871],
            [-0.035391, 0.999374, 0],
            [-0.41661, -0.014753, -0.908966],
        ],
    ),
    (
        READY,
        "panda_link8",
        [0.30702, 0, 0.59027],
        [[0.707388, -0.706825, 0], [-0.706825, -0.707388, 0], [0, 0, -1]],
    ),
    (
        READY,
        "right_gripper",
        [0.30702, 0, 0.49027],
        [[-1, -0.000398, 0], [-0.000398, 1, 0], [0, 0, -1]],
    ),
)
# Lula frame -> (Intrinsic object, frame).
FRAMES = {
    "panda_link8": ("franka", "flange"),
    "right_gripper": ("franka_hand", "tcp"),
}


def _channel(port: int) -> grpc.Channel:
  return grpc.insecure_channel(f"{HOST}:{port}")


def _world(method: str, request):
  with _channel(WORLD_PORT) as channel:
    return getattr(world_grpc.ObjectWorldServiceStub(channel), method)(
        request, timeout=DEADLINE_S
    )


def _planner(method: str, request, timeout: float = DEADLINE_S):
  with _channel(PLANNER_PORT) as channel:
    return getattr(planner_grpc.MotionPlannerServiceStub(channel), method)(
        request, timeout=timeout
    )


def _icon(method: str, request):
  with _channel(ICON_PORT) as channel:
    return getattr(icon_grpc.IconApiStub(channel), method)(
        request, timeout=DEADLINE_S
    )


def _robot() -> robot_pb.RobotReference:
  return robot_pb.RobotReference(
      object_id=refs_pb.ObjectReference(
          by_name=refs_pb.ObjectReferenceByName(object_name="franka")
      )
  )


def _object_node(name: str) -> refs_pb.TransformNodeReference:
  return refs_pb.TransformNodeReference(
      by_name=refs_pb.TransformNodeReferenceByName(
          object=refs_pb.ObjectReferenceByName(object_name=name)
      )
  )


def _frame_node(object_name: str, frame: str) -> refs_pb.TransformNodeReference:
  return refs_pb.TransformNodeReference(
      by_name=refs_pb.TransformNodeReferenceByName(
          frame=refs_pb.FrameReferenceByName(
              object_name=object_name, frame_name=frame
          )
      )
  )


def _fk(q, object_name: str, frame: str):
  """Pose of the frame relative to object `franka`, as (translation, rotation)."""
  pose = _planner(
      "ComputeFk",
      planner_pb.FkRequest(
          world_id=WORLD_ID,
          joints=joint_space_pb2.JointVec(joints=q),
          robot_reference=_robot(),
          reference=_object_node("franka"),
          target=_frame_node(object_name, frame),
      ),
  ).reference_t_target
  return _pose_to_numpy(pose)


def _pose_to_numpy(pose):
  x, y, z, w = (
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w,
  )
  rotation = np.array(
      [
          [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
      ]
  ) / (x * x + y * y + z * z + w * w)
  translation = np.array([pose.position.x, pose.position.y, pose.position.z])
  return translation, rotation


def _pose_errors(a, b) -> tuple[float, float]:
  """(translation error in metres, rotation error in degrees) between poses."""
  (ta, ra), (tb, rb) = a, b
  cos_angle = np.clip(
      (np.trace(np.asarray(ra).T @ np.asarray(rb)) - 1) / 2, -1, 1
  )
  return float(np.linalg.norm(np.asarray(ta) - np.asarray(tb))), float(
      np.degrees(np.arccos(cos_angle))
  )


def _check_pose(label: str, a, b, tolerance) -> tuple[float, float]:
  errors = _pose_errors(a, b)
  if errors[0] > tolerance[0] or errors[1] > tolerance[1]:
    raise RuntimeError(
        f"{label}: error {errors[0] * 1e3:.4f} mm / {errors[1]:.5f} deg"
        f" exceeds {tolerance[0] * 1e3} mm / {tolerance[1]} deg"
    )
  return errors


def _plan(start, goal, timeout: float) -> joint_space_pb2.JointTrajectoryPVA:
  response = _planner(
      "PlanTrajectory",
      planner_pb.MotionPlanningRequest(
          world_id=WORLD_ID,
          robot_specification=robot_pb.RobotSpecification(
              robot_reference=_robot(),
              start_configuration=joint_space_pb2.JointVec(joints=start),
          ),
          motion_specification=spec_pb.MotionSpecification(
              motion_segments=[
                  spec_pb.MotionSegment(
                      target=gc_pb.GeometricConstraint(
                          joint_position=joint_space_pb2.JointVec(joints=goal)
                      )
                  )
              ]
          ),
      ),
      timeout=timeout,
  )
  if not response.discretized.state:
    raise RuntimeError(
        "PlanTrajectory returned an empty discretized trajectory"
    )
  return response.discretized


def _sensed_arm_positions() -> list[float]:
  status = _icon("GetStatus", icon_pb.GetStatusRequest())
  return [j.position_sensed for j in status.part_status["arm"].joint_states]


def _run_on_arm(action, target, timeout_s: float) -> None:
  with _channel(ICON_PORT) as channel:
    with _session.Session(icon_grpc.IconApiStub(channel), ["arm"]) as session:
      session.add_action(action)
      if not session.start_action_and_wait(action, timeout_s=timeout_s):
        raise RuntimeError(f"action did not finish within {timeout_s} s")
  sensed = _sensed_arm_positions()
  error = max(abs(s - t) for s, t in zip(sensed, target, strict=True))
  if error > JOINT_TOLERANCE_RAD:
    raise RuntimeError(f"sensed {sensed} is {error:.5f} rad from {target}")


def check_objects() -> None:
  names = [
      o.name
      for o in _world(
          "ListObjects", world_pb.ListObjectsRequest(world_id=WORLD_ID)
      ).objects
  ]
  missing = {"franka", "franka_hand"} - set(names)
  if missing:
    raise RuntimeError(f"missing {sorted(missing)} in {names}")


def check_fk_20() -> str:
  urdf = fso.parse_urdf(URDF_PATH)
  limits = [urdf.joints[name].limit for name in fso.ARM_REVOLUTE_JOINTS]
  rng = random.Random(1)
  joint_vectors = [
      [rng.uniform(l.lower, l.upper) for l in limits] for _ in range(FK_SAMPLES)
  ]
  ISAAC_WORK_DIR.mkdir(exist_ok=True)
  (ISAAC_WORK_DIR / "fk_in.json").write_text(json.dumps(joint_vectors))
  (ISAAC_WORK_DIR / "fk_out.json").unlink(missing_ok=True)
  result = subprocess.run(
      ["cmd.exe", "/c", r"cd /d C:\isaac-sim && python.bat -"],
      input=ISAAC_SCRIPT.read_text(),
      capture_output=True,
      text=True,
      timeout=ISAAC_TIMEOUT_S,
  )
  if result.returncode != 0:
    raise RuntimeError(
        f"Isaac exited {result.returncode}: {result.stderr[-500:]}"
    )
  isaac = json.loads((ISAAC_WORK_DIR / "fk_out.json").read_text())
  worst = {}
  for q, lula_poses in zip(joint_vectors, isaac, strict=True):
    for lula_frame, (object_name, frame) in FRAMES.items():
      lula_pose = (
          lula_poses[lula_frame]["translation"],
          lula_poses[lula_frame]["rotation"],
      )
      errors = _check_pose(
          f"{lula_frame} at {q}",
          _fk(q, object_name, frame),
          lula_pose,
          FK_TOLERANCE,
      )
      prev = worst.get(lula_frame, (0.0, 0.0))
      worst[lula_frame] = (max(prev[0], errors[0]), max(prev[1], errors[1]))
  return "worst " + ", ".join(
      f"{frame} {t * 1e3:.6f} mm / {r:.6f} deg"
      for frame, (t, r) in worst.items()
  )


def check_fk_reference() -> None:
  for q, lula_frame, translation, rotation in FK_REFERENCE:
    object_name, frame = FRAMES[lula_frame]
    _check_pose(
        f"{lula_frame} at {q}",
        _fk(q, object_name, frame),
        (translation, rotation),
        FK_TOLERANCE,
    )


def check_ik() -> None:
  target = _planner(
      "ComputeFk",
      planner_pb.FkRequest(
          world_id=WORLD_ID,
          joints=joint_space_pb2.JointVec(joints=READY),
          robot_reference=_robot(),
          reference=_object_node("franka"),
          target=_frame_node("franka", "flange"),
      ),
  ).reference_t_target
  response = _planner(
      "ComputeIk",
      planner_pb.IkRequest(
          world_id=WORLD_ID,
          target=gc_pb.GeometricConstraint(
              cartesian_pose=gc_pb.PoseEquality(
                  moving_frame=_frame_node("franka", "flange"),
                  target_frame=_object_node("franka"),
                  target_frame_offset=target,
              )
          ),
          starting_joints=joint_space_pb2.JointVec(joints=DEFAULT_Q),
          robot_reference=_robot(),
          max_num_solutions=1,
      ),
  )
  if not response.solutions:
    raise RuntimeError("ComputeIk returned no solution")
  solution = list(response.solutions[0].joints)
  _check_pose(
      f"IK solution {solution}",
      _fk(solution, "franka", "flange"),
      _pose_to_numpy(target),
      IK_TOLERANCE,
  )


def check_plan() -> None:
  _plan(DEFAULT_Q, READY, DEADLINE_S)


def check_icon_status() -> None:
  _icon("GetStatus", icon_pb.GetStatusRequest())


def check_icon_parts() -> None:
  parts = list(_icon("ListParts", icon_pb.ListPartsRequest()).parts)
  if "arm" not in parts:
    raise RuntimeError(f"no part arm in {parts}")


def check_icon_move() -> None:
  _run_on_arm(
      point_to_point_move_utils.create_point_to_point_move_action(
          1, "arm", goal_position=READY
      ),
      READY,
      MOVE_TIMEOUT_S,
  )


def check_icon_trajectory() -> None:
  trajectory = _plan(READY, DEFAULT_Q, MOTION_DEADLINE_S)
  _run_on_arm(
      trajectory_tracking_action_utils.create_trajectory_tracking_action(
          1, "arm", trajectory
      ),
      DEFAULT_Q,
      MOTION_DEADLINE_S,
  )


CHECKS = (
    ("objects", check_objects),
    ("fk_20", check_fk_20),
    ("fk_reference", check_fk_reference),
    ("ik", check_ik),
    ("plan", check_plan),
    ("icon_status", check_icon_status),
    ("icon_parts", check_icon_parts),
    ("icon_move", check_icon_move),
    ("icon_trajectory", check_icon_trajectory),
)


def check_idempotent() -> None:
  """After a restart: ICON comes back and the loader did not duplicate the robot."""
  _wait_for_ports()
  names = [
      o.name
      for o in _world(
          "ListObjects", world_pb.ListObjectsRequest(world_id=WORLD_ID)
      ).objects
  ]
  counts = {name: names.count(name) for name in ("franka", "franka_hand")}
  if counts != {"franka": 1, "franka_hand": 1}:
    raise RuntimeError(
        f"expected one franka and one franka_hand, got {counts} in {names}"
    )


def _describe(error: Exception) -> str:
  if isinstance(error, grpc.RpcError):
    return f"{error.code().name}: {error.details()}"
  return f"{type(error).__name__}: {error}"


def _wait_for_ports() -> None:
  # ICON opens last, but the planner can still be starting; wait for all ports.
  deadline = time.monotonic() + ICON_WAIT_S
  for port in (18000, 18001, 18002, ICON_PORT):
    while True:
      try:
        socket.create_connection((HOST, port), timeout=1).close()
        break
      except OSError:
        if time.monotonic() > deadline:
          raise TimeoutError(f"port {port} not open after {ICON_WAIT_S} s")
        time.sleep(1)


def _run(name: str, check) -> bool:
  try:
    detail = check()
    print(f"PASS {name}" + (f": {detail}" if detail else ""), flush=True)
    return True
  except Exception as error:  # pylint: disable=broad-except
    print(f"FAIL {name}: {_describe(error)}", flush=True)
    return False


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--after-restart", action="store_true")
  args = parser.parse_args()
  start = time.monotonic()
  if args.after_restart:
    ok = _run("idempotent", check_idempotent)
    checks = [
        (f"idempotent/{name}", check)
        for name, check in CHECKS
        if name != "fk_20"
    ]
  else:
    with contextlib.suppress(TimeoutError):  # the ICON checks then report it
      _wait_for_ports()
    ok = True
    checks = list(CHECKS)
  results = [_run(name, check) for name, check in checks]
  ok = ok and all(results)
  print(f"elapsed {time.monotonic() - start:.1f}")
  return 0 if ok else 1


if __name__ == "__main__":
  sys.exit(main())
