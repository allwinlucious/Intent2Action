"""Phase 4 check: camera-to-robot calibration through CalibrationService (phase_4.md, done criteria).

Syncs the repository to Windows, starts sim/run_scene.py and the container
(ROBOT_BACKEND=isaac, container/run.sh), and prints PASS/FAIL per check, in order:
  .venv/bin/python tests/e2e/phase4_check.py
Robot and simulation calls go to RobotService / SimService on 127.0.0.1:18100; the camera belief is
read from `world` and `init_world` (:18000); Isaac's /tf through tests/e2e/phase3_probe.py in the image.
Stops the container and Isaac at the end.
"""

import socket
import subprocess
import sys
import time

import numpy as np

import harness
import phase3_hwm_check as hwm  # sets sys.path for gen/ and ICON's Python
import phase1_check as p1
import phase3_cuj_check as cuj
import phase3_scene_check as scene
from google.longrunning import operations_pb2  # noqa: E402
from intrinsic.world.proto import object_world_refs_pb2 as refs_pb  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2 as world_pb  # noqa: E402
from intrinsic.world.proto import object_world_updates_pb2 as updates_pb  # noqa: E402
from robot_service import robot_service_pb2 as api  # noqa: E402

ISAAC_LOG = harness.LOG_DIR / "phase4_isaac.log"
CONTAINER_LOG = harness.LOG_DIR / "phase4_container.log"
RESTARTED_CONTAINER_LOG = harness.LOG_DIR / "phase4_container_restarted.log"
CALIBRATION_PORT = 18006
SERVICE_UP_S = 60
STAYS_UP_S = 5
RESTART_TIMEOUT_S = 180
CALIBRATE_TIMEOUT_S = 15 * 60
POLL_S = 2.0
DETACH_TIMEOUT_S = 10
CAMERA, SENSOR, BOARD = "d455", "sensor", "calib_board"
TRUE_CAMERA_FRAME = "Camera_Pseudo_Depth"  # Isaac's /tf frame of the camera, optical axes
SEED_MIN = (0.02, 2.0)  # m, deg: the seed is at least this far from the true pose
ACCURACY = (2e-3, 0.2)  # m, deg (G6)
PERSISTED = (1e-4, 0.01)  # m, deg
MIN_POSES = 12
CLOUD_STRIDE = 1000  # GetPointCloud is read only for camera_calibrated
_calibrated_pose = {}


def _port_open(port: int) -> bool:
  try:
    socket.create_connection((p1.HOST, port), timeout=1).close()
    return True
  except OSError:
    return False


def _matrix(pose) -> np.ndarray:
  """4x4 of an intrinsic_proto.Pose."""
  p, q = pose.position, pose.orientation
  return cuj._matrix(api.Pose(x=p.x, y=p.y, z=p.z, qx=q.x, qy=q.y, qz=q.z, qw=q.w))


def _belief(world_id: str = "world") -> np.ndarray:
  """d455 `sensor` in the base frame (= the world root; the Franka stands at the origin)."""
  return _matrix(p1._world("GetTransform", world_pb.GetTransformRequest(
      world_id=world_id, node_a=refs_pb.TransformNodeReference(id="root"),
      node_b=refs_pb.TransformNodeReference(by_name=refs_pb.TransformNodeReferenceByName(
          frame=refs_pb.FrameReferenceByName(object_name=CAMERA, frame_name=SENSOR))))).a_t_b)


def _truth() -> np.ndarray:
  return cuj._matrix(cuj._call(cuj._sim, "GetTruePose", api.GetTruePoseRequest(frame=TRUE_CAMERA_FRAME)))


def _calibrated() -> bool:
  return cuj._call(cuj._robot, "GetPointCloud", api.GetPointCloudRequest(stride=CLOUD_STRIDE)).camera_calibrated


def _describe(error: tuple[float, float]) -> str:
  return f"{error[0] * 1e3:.2f} mm / {error[1]:.3f} deg"


def _restart() -> float:
  """docker restart intrinsic; seconds until robot_service is enabled again."""
  start = time.monotonic()
  subprocess.run(harness.docker("docker restart intrinsic"), check=True, capture_output=True)
  p1._wait_for_ports()
  hwm._require_state("ENABLED", RESTART_TIMEOUT_S)
  cuj._wait_ready()
  took = time.monotonic() - start
  if took > RESTART_TIMEOUT_S:
    raise RuntimeError(f"robot_service enabled {took:.0f} s after docker restart (limit {RESTART_TIMEOUT_S} s)")
  return took


def check_service_up() -> str:
  start = time.monotonic()
  while not _port_open(CALIBRATION_PORT):
    if time.monotonic() - start > SERVICE_UP_S:
      raise RuntimeError(f"nothing listening on {CALIBRATION_PORT} within {SERVICE_UP_S} s")
    time.sleep(0.5)
  time.sleep(STAYS_UP_S)
  process = subprocess.run(harness.docker("docker exec intrinsic pgrep -f calibration_service_main"),
                           capture_output=True, text=True)
  if process.returncode != 0 or not _port_open(CALIBRATION_PORT):
    raise RuntimeError(f"calibration_service_main not running {STAYS_UP_S} s later (pgrep exit {process.returncode})")
  return f"calibration_service_main (pid {process.stdout.split()[0]}) on {CALIBRATION_PORT}, still up after {STAYS_UP_S} s"


def check_fresh_seed() -> str:
  names = {o.name for o in p1._world("ListObjects", world_pb.ListObjectsRequest(world_id="init_world")).objects}
  if CAMERA in names:
    p1._world("DeleteObject", updates_pb.DeleteObjectRequest(
        world_id="init_world", object=refs_pb.ObjectReference(by_name=refs_pb.ObjectReferenceByName(object_name=CAMERA))))
  took = _restart()
  error = cuj._pose_error(_belief(), _truth())
  calibrated = _calibrated()
  if error[0] < SEED_MIN[0] or error[1] < SEED_MIN[1] or calibrated:
    raise RuntimeError(f"after restart d455 is {_describe(error)} from the true pose, camera_calibrated {calibrated}")
  return (f"deleted {CAMERA} from init_world (was there: {CAMERA in names}); restart {took:.0f} s; seed"
          f" {_describe(error)} from Isaac's true pose; camera_calibrated false")


def check_calibrate() -> str:
  cuj._call(cuj._sim, "AttachCalibrationBoard")
  start = time.monotonic()
  operation = cuj._call(cuj._robot, "CalibrateCamera", api.CalibrateCameraRequest(wait=False))
  info, last = api.OperationInfo(), None
  while not operation.done:
    if time.monotonic() - start > CALIBRATE_TIMEOUT_S:
      raise RuntimeError(f"{operation.name} not done after {CALIBRATE_TIMEOUT_S} s ({info.state}: {info.detail})")
    time.sleep(POLL_S)
    operation = cuj._call(cuj._robot, "GetOperation", operations_pb2.GetOperationRequest(name=operation.name))
    operation.metadata.Unpack(info)
    if info.detail != last:
      print(f"  {time.monotonic() - start:6.1f} s {info.state}: {info.detail}", flush=True)
      last = info.detail
  result = cuj._done(operation, api.CalibrationResult)
  if result.poses_used < MIN_POSES:
    raise RuntimeError(f"{operation.name}: poses_used {result.poses_used} < {MIN_POSES}")
  return (f"{operation.name} done in {time.monotonic() - start:.0f} s: poses_used {result.poses_used},"
          f" rms {result.rms_translation_m * 1e3:.3f} mm / {result.rms_rotation_deg:.4f} deg")


def check_accuracy() -> str:
  belief = _belief()
  error = cuj._pose_error(belief, _truth())
  calibrated = _calibrated()
  if error[0] >= ACCURACY[0] or error[1] >= ACCURACY[1] or not calibrated:
    raise RuntimeError(f"d455 {_describe(error)} from the true pose (need < {_describe(ACCURACY)}), "
                       f"camera_calibrated {calibrated}")
  _calibrated_pose["world"] = belief
  return f"d455 sensor {_describe(error)} from Isaac's true pose; camera_calibrated true"


def check_persist() -> str:
  if "world" not in _calibrated_pose:
    raise RuntimeError("no accurate calibrated pose to compare (accuracy failed)")
  took = _restart()
  error = cuj._pose_error(_belief(), _calibrated_pose["world"])
  calibrated = _calibrated()
  if error[0] > PERSISTED[0] or error[1] > PERSISTED[1] or not calibrated:
    raise RuntimeError(f"after restart d455 is {_describe(error)} from the calibrated pose, camera_calibrated {calibrated}")
  return f"restart {took:.0f} s; d455 {_describe(error)} from the calibrated pose; camera_calibrated true"


def check_detach() -> str:
  cuj._call(cuj._sim, "DetachCalibrationBoard")
  start = time.monotonic()
  while BOARD in scene._objects("world"):
    if time.monotonic() - start > DETACH_TIMEOUT_S:
      raise RuntimeError(f"{BOARD} still in world {DETACH_TIMEOUT_S} s after DetachCalibrationBoard")
    time.sleep(0.2)
  gone = time.monotonic() - start
  frames = scene._true_frames()
  if BOARD in frames:
    raise RuntimeError(f"{BOARD} still on Isaac's /tf")
  return f"{BOARD} gone from world {gone:.1f} s after DetachCalibrationBoard; not on Isaac's /tf ({len(frames)} frames)"


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
        ("service_up", check_service_up),
        ("fresh_seed", check_fresh_seed),
        ("calibrate", check_calibrate),
        ("accuracy", check_accuracy),
        ("persist", check_persist),
        ("detach", check_detach),
    )
    results = [hwm._run_check(name, check) for name, check in checks]
  except Exception as error:  # pylint: disable=broad-except
    print(f"FAIL setup: {p1._describe(error)}", flush=True)
    results = [False]
  finally:
    logs = subprocess.run(harness.docker("docker logs intrinsic"), capture_output=True, text=True, errors="replace")
    RESTARTED_CONTAINER_LOG.write_text(logs.stdout + logs.stderr)
    run.stop_container()
    run.stop_isaac()
  print(f"elapsed {time.monotonic() - start:.1f}")
  return 0 if results and all(results) else 1


if __name__ == "__main__":
  sys.exit(main())
