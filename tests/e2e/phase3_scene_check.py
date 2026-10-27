"""Phase 3 Part 2 check: scene_sync, camera_sim and the ICON registry entry.

Syncs the repository to Windows, starts sim/run_scene.py and the container
(ROBOT_BACKEND=isaac, container/run.sh), and prints PASS/FAIL per check
(phase_3.md, Part 2):
  .venv/bin/python tests/e2e/phase3_scene_check.py
Isaac's /tf is read by tests/e2e/phase3_probe.py in the image; the worlds,
SimLink, CameraService and ICON over gRPC from WSL. Stops the container and
Isaac at the end.
"""

import subprocess
import sys
import time

import grpc
from google.protobuf import empty_pb2
from google.protobuf import wrappers_pb2
import numpy as np
import yaml

import harness
import phase3_hwm_check as hwm  # sets sys.path for gen/ and ICON's Python
import phase1_check as p1
from intrinsic.perception.proto.v1 import camera_identifier_pb2  # noqa: E402
from intrinsic.perception.proto.v1 import camera_service_pb2 as camera_pb  # noqa: E402
from intrinsic.perception.proto.v1 import camera_service_pb2_grpc as camera_grpc  # noqa: E402
from intrinsic.perception.proto.v1 import image_buffer_pb2 as ib  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2 as world_pb  # noqa: E402
from intrinsic.world.proto import object_world_updates_pb2 as updates_pb  # noqa: E402
from robot_service import robot_service_pb2  # noqa: E402
from robot_service import simlink_pb2  # noqa: E402
from robot_service import simlink_pb2_grpc  # noqa: E402

ISAAC_LOG = harness.LOG_DIR / "phase3_scene_isaac.log"
CONTAINER_LOG = harness.LOG_DIR / "phase3_scene_container.log"
RESTARTED_CONTAINER_LOG = harness.LOG_DIR / "phase3_scene_container_restarted.log"
SIMLINK_PORT, CAMERA_PORT = 18101, 18005
WORLDS = ("sim_world", "world")
DEFAULT_OBJECTS = tuple(
    yaml.safe_load((harness.ROOT / "sim/scenes/default.yaml").read_text())[
        "objects"
    ]
)
ROBOT_OBJECTS = ("franka", "franka_hand")
POSITION_TOLERANCE_M = 0.005
SYNC_TIMEOUT_S = 60
EXTRA = "extra_cube"
# A free spot on the table (as phase2_peer_io.py).
EXTRA_POSE = robot_service_pb2.Pose(x=0.30, y=-0.30, z=0.0, qw=1.0)
APPEAR_S = 5.0
GONE_TIMEOUT_S = 10.0
JOINT_TOLERANCE_RAD = 0.01
JOINT_SYNC_TIMEOUT_S = 5.0
WIDTH, HEIGHT = 848, 480
FX = 420.08
FX_TOLERANCE = 0.01
RESTART_TIMEOUT_S = 240
RESTART_SETTLE_S = 3.0


def _simlink(method: str, request, timeout: float = 30):
  with p1._channel(SIMLINK_PORT) as channel:
    return getattr(simlink_pb2_grpc.SimLinkStub(channel), method)(
        request, timeout=timeout
    )


def _camera(method: str, request, timeout: float = 30):
  with p1._channel(CAMERA_PORT) as channel:
    return getattr(camera_grpc.CameraServiceStub(channel), method)(
        request, timeout=timeout
    )


def _objects(world_id: str) -> dict[str, list]:
  """Name -> objects with that name (FULL view), without root."""
  found = {}
  for o in p1._world(
      "ListObjects",
      world_pb.ListObjectsRequest(world_id=world_id, view=updates_pb.FULL),
  ).objects:
    if o.name != "root":
      found.setdefault(o.name, []).append(o)
  return found


def _wait_objects(names, present: bool, timeout_s: float) -> float:
  """Seconds until every name is present (or absent) in both worlds."""
  start = time.monotonic()
  while True:
    try:
      worlds = [_objects(w) for w in WORLDS]
      if all((n in objects) == present for objects in worlds for n in names):
        return time.monotonic() - start
    except grpc.RpcError:
      pass  # a world not created yet
    if time.monotonic() - start > timeout_s:
      raise RuntimeError(
          f"{sorted(names)} not {'present' if present else 'gone'} in"
          f" {WORLDS} within {timeout_s} s"
      )
    time.sleep(0.2)


def _true_frames() -> dict[str, list[float]]:
  result, output = harness.run_probe(
      "phase3_tf", "rmw_fastrtps_cpp", "phase3_probe.py", "tf", hwm.PROBE_TIMEOUT_S
  )
  if result is None or "error" in result:
    raise RuntimeError(f"tf probe: {result or output[-300:]}")
  return result["frames"]


def _source(o) -> str:
  value = wrappers_pb2.StringValue()
  o.object_component.user_data["source"].Unpack(value)
  return value.value


def check_objects() -> str:
  _wait_objects(DEFAULT_OBJECTS, True, SYNC_TIMEOUT_S)
  truth = _true_frames()
  details, ok = [], True
  for w in WORLDS:
    objects = _objects(w)
    worst = 0.0
    for name in DEFAULT_OBJECTS:
      (o,) = objects[name]
      p = o.object_component.parent_t_this.position
      error = float(np.linalg.norm(np.array([p.x, p.y, p.z]) - truth[name][:3]))
      worst = max(worst, error)
      ok &= o.parent.name == "root" and _source(o) == "ground_truth"
    ok &= worst <= POSITION_TOLERANCE_M
    details.append(f"{w}: worst {worst * 1e3:.2f} mm, sources ground_truth")
  if not ok:
    raise RuntimeError("; ".join(details))
  return "; ".join(details)


def check_spawn() -> str:
  _simlink(
      "Spawn",
      simlink_pb2.SpawnRequest(catalog="red_cube", name=EXTRA, pose=EXTRA_POSE),
  )
  appeared = _wait_objects([EXTRA], True, APPEAR_S)
  _simlink("Delete", simlink_pb2.NameRequest(name=EXTRA))
  gone = _wait_objects([EXTRA], False, GONE_TIMEOUT_S)
  return (
      f"{EXTRA} in both worlds {appeared:.1f} s after Spawn, gone"
      f" {gone:.1f} s after Delete"
  )


def check_joints() -> str:
  goal = list(p1.READY)
  goal[0] += hwm.MOVE_OFFSET_RAD
  hwm._move(goal)
  icon = [j.position_sensed for j in hwm._part_status("arm").joint_states]
  deadline = time.monotonic() + JOINT_SYNC_TIMEOUT_S
  while True:
    (franka,) = _objects("world")["franka"]
    world = list(franka.kinematic_object_component.joint_positions)
    error = max(abs(a - b) for a, b in zip(world, icon, strict=True))
    if error <= JOINT_TOLERANCE_RAD:
      return f"world franka joints within {error:.5f} rad of ICON's"
    if time.monotonic() > deadline:
      raise RuntimeError(
          f"world franka {np.round(world, 4).tolist()} vs ICON"
          f" {np.round(icon, 4).tolist()}: error {error:.4f} rad"
      )
    time.sleep(0.2)


def _d455() -> camera_identifier_pb2.CameraIdentifier:
  identifier = camera_identifier_pb2.CameraIdentifier()
  identifier.ros.device_id = "d455"
  return identifier


def _seconds(timestamp) -> float:
  return timestamp.seconds + timestamp.nanos * 1e-9


def check_camera() -> str:
  listed = _camera(
      "ListAvailableCameras", camera_pb.ListAvailableCamerasRequest()
  ).camera_identifiers
  if list(listed) != [_d455()]:
    raise RuntimeError(f"ListAvailableCameras: {list(listed)}")
  clock = _simlink("GetClock", empty_pb2.Empty()).sim_time
  request = camera_pb.CaptureRequest()
  request.camera_config.identifier.CopyFrom(_d455())
  result = _camera("Capture", request).capture_result
  images = {i.sensor_config.id: i for i in result.sensor_images}
  want = {
      1: (ib.PIXEL_INTENSITY, 3, ib.TYPE_UINT8, 1),
      4: (ib.PIXEL_DEPTH, 1, ib.TYPE_FLOAT32, 4),
  }
  problems = []
  if set(images) != set(want):
    raise RuntimeError(f"Capture returned sensors {sorted(images)}")
  for sensor_id, (pixel, channels, data_type, size) in want.items():
    image, b = images[sensor_id], images[sensor_id].buffer
    fx = image.sensor_config.camera_params.intrinsic_params.focal_length_x
    got = (b.encoding, b.pixel_type, b.num_channels, b.type, b.packing_type)
    if got != (ib.ENCODING_UNSPECIFIED, pixel, channels, data_type,
               ib.PACKING_TYPE_INTERLEAVED):
      problems.append(f"sensor {sensor_id} buffer {got}")
    if (b.dimensions.cols, b.dimensions.rows) != (WIDTH, HEIGHT) or len(
        b.data
    ) != WIDTH * HEIGHT * channels * size:
      problems.append(f"sensor {sensor_id} {b.dimensions} {len(b.data)} bytes")
    if abs(fx - FX) > FX_TOLERANCE:
      problems.append(f"sensor {sensor_id} fx {fx}")
    if _seconds(image.acquisition_time) <= clock:
      problems.append(
          f"sensor {sensor_id} stamped {_seconds(image.acquisition_time):.3f}"
          f" s, request after {clock:.3f} s"
      )
  depth = np.frombuffer(images[4].buffer.data, dtype=np.float32)
  sensors = _camera(
      "DescribeCamera", camera_pb.DescribeCameraRequest(camera_identifier=_d455())
  ).sensors
  described = [
      (
          s.id,
          s.display_name,
          list(s.supported_pixel_types),
          (s.dimensions.cols, s.dimensions.rows),
      )
      for s in sensors
  ]
  if described != [
      (1, "color", [ib.PIXEL_INTENSITY], (WIDTH, HEIGHT)),
      (4, "depth", [ib.PIXEL_DEPTH], (WIDTH, HEIGHT)),
  ] or any(
      abs(s.factory_camera_params.intrinsic_params.focal_length_x - FX)
      > FX_TOLERANCE
      for s in sensors
  ):
    problems.append(f"DescribeCamera {described}")
  detail = (
      f"RGB + depth {WIDTH}x{HEIGHT}, fx"
      f" {images[1].sensor_config.camera_params.intrinsic_params.focal_length_x:.2f},"
      f" stamped {_seconds(images[1].acquisition_time) - clock:+.3f} s after"
      f" the clock read before Capture, depth median"
      f" {np.median(depth[np.isfinite(depth)]):.3f} m; DescribeCamera"
      f" {described}"
  )
  if problems:
    raise RuntimeError(f"{'; '.join(problems)} ({detail})")
  return detail


def _object_ids() -> dict[tuple[str, str], set[str]]:
  return {
      (w, n): {o.id for o in objects.get(n, [])}
      for w in WORLDS
      for objects in [_objects(w)]
      for n in DEFAULT_OBJECTS
  }


def check_restart() -> str:
  before = _object_ids()
  subprocess.run(
      harness.docker("docker restart intrinsic"), check=True, capture_output=True
  )
  start = time.monotonic()
  p1._wait_for_ports()
  # The worlds are persisted: wait until the new scene_sync has replaced
  # every object (new IDs), then count.
  while True:
    try:
      after = _object_ids()
      if all(after[k] and not after[k] & before[k] for k in before):
        break
    except grpc.RpcError:
      pass
    if time.monotonic() - start > RESTART_TIMEOUT_S:
      raise RuntimeError(f"objects not re-created within {RESTART_TIMEOUT_S} s")
    time.sleep(0.5)
  time.sleep(RESTART_SETTLE_S)
  counts = {
      w: {n: len(v) for n, v in _objects(w).items()} for w in WORLDS
  }
  want = {n: 1 for n in DEFAULT_OBJECTS + ROBOT_OBJECTS}
  if any(c != want for c in counts.values()):
    raise RuntimeError(f"object counts after restart: {counts}")
  return (
      f"objects re-created {time.monotonic() - start - RESTART_SETTLE_S:.0f} s"
      " after docker restart;"
      f" exactly one of each in both worlds: {sorted(want)}"
  )


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
    print(f"ICON ENABLED after {time.monotonic() - start:.1f} s", flush=True)
    checks = (
        ("objects", check_objects),
        ("spawn", check_spawn),
        ("joints", check_joints),
        ("camera", check_camera),
        ("restart", check_restart),
    )
    results = [hwm._run_check(name, check) for name, check in checks]
  except Exception as error:  # pylint: disable=broad-except
    print(f"FAIL setup: {p1._describe(error)}", flush=True)
    results = [False]
  finally:
    logs = subprocess.run(
        harness.docker("docker logs intrinsic"),
        capture_output=True,
        text=True,
        errors="replace",
    )
    RESTARTED_CONTAINER_LOG.write_text(logs.stdout + logs.stderr)
    run.stop_container()
    run.stop_isaac()
  print(f"elapsed {time.monotonic() - start:.1f}")
  return 0 if results and all(results) else 1


if __name__ == "__main__":
  sys.exit(main())
