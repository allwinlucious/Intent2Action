"""Phase 4 experiments c and d (phase_4.md Step 2), against a running container and Isaac:
  c. Board attached, arm at the decision 6 start pose: Initialize + one CaptureData in < 5 s with NO_ERRORS.
  d. Detection rate: sampled poses (decision 8), moved to in order until 20 moves succeeded -> >= 16 NO_ERRORS.
Moves go through RobotService (MoveToPose, tool FLANGE); the CalibrationService (18006) is called directly;
poses come from the image's sample_poses CLI (docker exec). Ends with Home and DetachCalibrationBoard.
  .venv/bin/python tests/e2e/exp4/exp4_cd.py   ->   progress lines, then one JSON line; exit 0 if both hold
"""

import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]
from google.protobuf import empty_pb2  # noqa: E402
import grpc  # noqa: E402
from intrinsic.perception.proto.v1 import calibration_service_pb2 as calib_pb  # noqa: E402
from intrinsic.perception.proto.v1 import calibration_service_pb2_grpc as calib_grpc  # noqa: E402
from intrinsic.world.proto import object_world_refs_pb2 as refs_pb  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2 as world_pb  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2_grpc as world_grpc  # noqa: E402
from robot_service import robot_service_pb2 as api  # noqa: E402
from robot_service import robot_service_pb2_grpc as api_grpc  # noqa: E402

# Appended after the import above: container/services/robot_service.py must not shadow the generated package.
sys.path.append(str(ROOT / "container/services"))
from robot import calibration  # noqa: E402
from robot.motion import from_pose_proto  # noqa: E402

CAPTURE_BUDGET_S = 5.0
MOVES = 20
MIN_DETECTIONS = 16
SETTLE_S = 0.5  # wall, after each move, so world's joints follow ICON's
TIMEOUT_S = 120


def _sample(start: api.Pose, camera: api.Pose) -> list[api.Pose]:
  def run(_, **kwargs):
    return subprocess.run(["sg", "docker", "-c", f"docker exec -i intrinsic {calibration.SAMPLE_POSES}"], **kwargs)
  return calibration.sample_poses(start, camera, run=run)


def _move(robot, pose: api.Pose) -> str | None:
  """None on success, else the error."""
  try:
    robot.MoveToPose(api.MoveToPoseRequest(pose=pose, tool=api.FLANGE, motion=api.ANY, wait=True), timeout=TIMEOUT_S)
    return None
  except grpc.RpcError as error:
    return f"{error.code().name}: {error.details()}"


def main() -> int:
  channel = grpc.insecure_channel("127.0.0.1:18100")
  robot, sim = api_grpc.RobotServiceStub(channel), api_grpc.SimServiceStub(channel)
  service = calib_grpc.CalibrationServiceStub(grpc.insecure_channel("127.0.0.1:18006"))
  world = world_grpc.ObjectWorldServiceStub(grpc.insecure_channel("localhost:18000"))
  result = {}
  sim.AttachCalibrationBoard(empty_pb2.Empty(), timeout=TIMEOUT_S)
  try:
    camera = from_pose_proto(world.GetTransform(world_pb.GetTransformRequest(
        world_id="world", node_a=refs_pb.TransformNodeReference(id="root"),
        node_b=refs_pb.TransformNodeReference(by_name=refs_pb.TransformNodeReferenceByName(
            frame=refs_pb.FrameReferenceByName(object_name="d455", frame_name="sensor"))))).a_t_b)
    start = calibration.start_pose((camera.x, camera.y, camera.z))
    error = _move(robot, start)
    if error:
      result["c"] = {"error": f"start pose not reached: {error}"}
      print(json.dumps(result))
      return 1
    time.sleep(SETTLE_S)
    began = time.monotonic()
    service.Initialize(calibration.initialize_request(), timeout=TIMEOUT_S)
    initialized = time.monotonic()
    status = calibration.status_name(service.CaptureData(calib_pb.CaptureDataRequest(), timeout=TIMEOUT_S))
    took = time.monotonic() - began
    result["c"] = {"status": status, "initialize_s": round(initialized - began, 2), "total_s": round(took, 2)}
    print(f"c: {result['c']}", flush=True)

    statuses, skipped = [], []
    for i, pose in enumerate(_sample(start, camera)):
      if len(statuses) == MOVES:
        break
      error = _move(robot, pose)
      if error:
        skipped.append(i)
        print(f"d: sample {i} skipped: {error}", flush=True)
        continue
      time.sleep(SETTLE_S)
      statuses.append(calibration.status_name(service.CaptureData(calib_pb.CaptureDataRequest(), timeout=TIMEOUT_S)))
      print(f"d: sample {i} capture {len(statuses)}/{MOVES}: {statuses[-1]}", flush=True)
    detections = statuses.count(calibration.NO_ERRORS)
    result["d"] = {"captures": len(statuses), "no_errors": detections, "statuses": statuses, "skipped_samples": skipped}
  finally:
    robot.Home(api.HomeRequest(wait=True), timeout=TIMEOUT_S)
    sim.DetachCalibrationBoard(empty_pb2.Empty(), timeout=TIMEOUT_S)
  ok_c = result["c"]["status"] == calibration.NO_ERRORS and result["c"]["total_s"] < CAPTURE_BUDGET_S
  ok_d = result["d"]["captures"] == MOVES and result["d"]["no_errors"] >= MIN_DETECTIONS
  result["pass"] = {"c": ok_c, "d": ok_d}
  print(json.dumps(result))
  return 0 if ok_c and ok_d else 1


if __name__ == "__main__":
  sys.exit(main())
