"""Phase 3 Part 3 check: the robot API (robot_service) over gRPC from WSL.

Syncs the repository to Windows, starts sim/run_scene.py and the container
(ROBOT_BACKEND=isaac, container/run.sh), and prints PASS/FAIL per check
(phase_3.md, Part 3):
  .venv/bin/python tests/e2e/phase3_cuj_check.py
Every call goes to RobotService and SimService on 127.0.0.1:18100 (operations are
polled with RobotService.GetOperation / WaitOperation),
except CUJ 5's fault trigger (SimLink.Reset on :18101). Stops the container
and Isaac at the end.
"""

import math
import struct
import subprocess
import sys
import time
import zlib

import grpc
from google.protobuf import empty_pb2
import numpy as np

import harness
import phase3_hwm_check as hwm  # sets sys.path for gen/ and ICON's Python
import phase1_check as p1
import phase3_scene_check as scene
from google.longrunning import operations_pb2  # noqa: E402
from robot_model import franka_scene_object as fso  # noqa: E402
from robot_service import robot_service_pb2 as api  # noqa: E402
from robot_service import robot_service_pb2_grpc as api_grpc  # noqa: E402

ISAAC_LOG = harness.LOG_DIR / "phase3_cuj_isaac.log"
CONTAINER_LOG = harness.LOG_DIR / "phase3_cuj_container.log"
ADDRESS = "127.0.0.1:18100"
CALL_TIMEOUT_S = 90
READY_TIMEOUT_S = 60
EMPTY = empty_pb2.Empty()
POSE_TOLERANCE = (1e-3, 0.5)  # m, deg (CUJ 2)
DURATION_TOLERANCE = 0.05  # PlanOnly vs MoveWaypoints
JOG = (1, 0.1, 1.0)  # joint, rad/s, s
JOG_TOLERANCE_RAD = 0.01
FK_TOLERANCE_M = 1e-4
FAULT_WAIT_S = 5.0
MOTION_AFTER_FAULT_S = 1.0
CUBE = "red_cube"
CUBE_HALF_M = 0.025
PREGRASP_UP_M = 0.10
LIFT_M = 0.05
LIFT_MIN_M = 0.04
OBSTACLE = "obstacle"
OBSTACLE_POSE = api.Pose(x=0.45, y=0.0, z=0.0, qw=1.0)  # box 0.1 x 0.1 x 0.5 m
OBSTACLE_SIDES_Y = (0.25, -0.25)
OBSTACLE_Z = 0.25
PATH_SAMPLES = 20
SPEED = 0.5
SPEED_TOLERANCE = 0.10
PAYLOAD_KG = 0.5
WIDTH, HEIGHT = 1280, 720
# Default stride 4: at most 1280 * 720 / 4 = 230 400 points; every pixel sees the floor or the table.
CLOUD_POINTS = (226_000, 230_400)
# phase_3.md says (0.2, 4); the floor (1.65 m below the camera, pitched 45 deg) reaches 5.43 m at
# the top image row (vertical FOV 59.5 deg).
CLOUD_Z = (0.2, 6.0)
UNREACHABLE = api.Pose(x=2.0, y=0.0, z=0.5, qw=1.0)

_channel = grpc.insecure_channel(ADDRESS)  # default 4 MB receive limit (GetPointCloud must fit)
_robot = api_grpc.RobotServiceStub(_channel)
_sim = api_grpc.SimServiceStub(_channel)


def _call(stub, method: str, request=EMPTY):
  return getattr(stub, method)(request, timeout=CALL_TIMEOUT_S)


def _done(operation, result_type):
  """The operation's response; raises if it failed or is still running."""
  if not operation.done:
    raise RuntimeError(f"{operation.name} still running after wait")
  if operation.HasField("error"):
    code = next(c.name for c in grpc.StatusCode if c.value[0] == operation.error.code)
    raise RuntimeError(f"{operation.name} failed: {code} {operation.error.message}")
  result = result_type()
  operation.response.Unpack(result)
  return result


def _move(method: str, **fields) -> api.MotionResult:
  request = getattr(api, f"{method}Request")(wait=True, **fields)
  return _done(_call(_robot, method, request), api.MotionResult)


def _gripper(method: str) -> api.GripperResult:
  return _done(
      _call(_robot, method, api.GripperRequest(wait=True)), api.GripperResult
  )


def _state() -> api.GetStateResponse:
  return _call(_robot, "GetState")


def _wait_ready() -> None:
  deadline = time.monotonic() + READY_TIMEOUT_S
  while True:
    try:
      if _state().enabled:
        return
    except grpc.RpcError:
      pass
    if time.monotonic() > deadline:
      raise RuntimeError(f"robot_service not enabled within {READY_TIMEOUT_S} s")
    time.sleep(0.5)


def _quat(p: api.Pose) -> np.ndarray:
  return np.array([p.qw, p.qx, p.qy, p.qz])


def _matrix(p: api.Pose) -> np.ndarray:
  w, x, y, z = _quat(p) / np.linalg.norm(_quat(p))
  m = np.eye(4)
  m[:3, :3] = [
      [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ]
  m[:3, 3] = [p.x, p.y, p.z]
  return m


def _origin_matrix(origin: fso.Origin) -> np.ndarray:
  r, p, y = origin.rpy
  rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
  ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
  rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
  m = np.eye(4)
  m[:3, :3] = rz @ ry @ rx
  m[:3, 3] = origin.xyz
  return m


def _pose_error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
  """Position (m) and rotation (deg) between two 4x4 transforms."""
  cos = (np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2
  return (
      float(np.linalg.norm(a[:3, 3] - b[:3, 3])),
      math.degrees(math.acos(max(-1.0, min(1.0, cos)))),
  )


def _hand_t_tcp() -> np.ndarray:
  urdf = fso.parse_urdf(p1.URDF_PATH)
  flange_t_hand = _origin_matrix(urdf.joints[fso.HAND_JOINT].origin)
  flange_t_tcp = _origin_matrix(urdf.joints[fso.TCP_JOINT].origin)
  return np.linalg.inv(flange_t_hand) @ flange_t_tcp


def _at(base: api.Pose, x: float, y: float, z: float) -> api.Pose:
  """`base`'s orientation at position (x, y, z)."""
  return api.Pose(x=x, y=y, z=z, qx=base.qx, qy=base.qy, qz=base.qz, qw=base.qw)


def _down() -> api.Pose:
  """The TCP pose at home (gripper pointing down)."""
  _move("Home")
  return _state().tool_pose


def _true_z(name: str) -> float:
  return _call(_sim, "GetTruePose", api.GetTruePoseRequest(frame=name)).z


def _code(error: grpc.RpcError) -> str:
  return f"{error.code().name} {error.details()!r}"


def check_logging() -> str:
  try:
    _call(_robot, "MoveToPose", api.MoveToPoseRequest(pose=UNREACHABLE, wait=True))
  except grpc.RpcError as error:
    code, message = error.code(), error.details()
  else:
    raise RuntimeError("unreachable MoveToPose returned OK")
  if code == grpc.StatusCode.OK or not message:
    raise RuntimeError(f"unreachable MoveToPose: {code} {message!r}")
  log = subprocess.run(
      harness.docker(
          "docker exec intrinsic tail -n 200 /var/lib/intrinsic/logs/robot_service.log"
      ),
      capture_output=True,
      text=True,
      check=True,
  ).stdout
  lines = [
      line for line in log.splitlines()
      if "rpc MoveToPose " in line and f"status={code.name}" in line
  ]
  if not lines:
    raise RuntimeError(f"no robot_service.log line for MoveToPose {code.name}")
  return f"unreachable MoveToPose -> {code.name} {message!r}; logged: {lines[-1][:160]}"


def check_cuj2() -> str:
  down = _down()
  target = _at(down, down.x + 0.10, down.y + 0.10, down.z - 0.15)
  _move("MoveToPose", pose=target, motion=api.LINEAR)
  hand = _call(_sim, "GetTruePose", api.GetTruePoseRequest(frame="panda_hand"))
  position, angle = _pose_error(_matrix(hand) @ _hand_t_tcp(), _matrix(target))
  if position > POSE_TOLERANCE[0] or angle > POSE_TOLERANCE[1]:
    raise RuntimeError(f"true TCP {position * 1e3:.2f} mm / {angle:.3f} deg from target")
  return f"LINEAR MoveToPose: true TCP {position * 1e3:.2f} mm / {angle:.3f} deg from target"


def check_cuj4() -> str:
  goal = list(fso.HOME)
  goal[0] += 1.0
  operation = _call(
      _robot, "MoveJoints",
      api.MoveJointsRequest(joints=api.Joints(positions=goal), wait=False),
  )
  time.sleep(1.0)
  _call(_robot, "Stop")
  operation = _call(
      _robot, "GetOperation", operations_pb2.GetOperationRequest(name=operation.name)
  )
  info = api.OperationInfo()
  operation.metadata.Unpack(info)
  if not (operation.done and operation.error.code == grpc.StatusCode.CANCELLED.value[0]
          and info.state == "CANCELLED"):
    raise RuntimeError(
        f"after Stop: done={operation.done} error={operation.error} state={info.state}"
    )
  result = _move("Home")
  return (
      f"Stop -> {operation.name} CANCELLED ({operation.error.message!r});"
      f" next Home done in {result.duration_s:.2f} s"
  )


def check_cuj5() -> str:
  scene._simlink("Reset", EMPTY)
  start = time.monotonic()
  while not (state := _state()).faulted:
    if time.monotonic() - start > FAULT_WAIT_S:
      raise RuntimeError(f"not faulted within {FAULT_WAIT_S} s after SimLink.Reset")
    time.sleep(0.05)
  faulted_at = time.monotonic()
  try:
    _call(_robot, "Home", api.HomeRequest(wait=False))
  except grpc.RpcError as error:
    rejected = error
  else:
    raise RuntimeError("Home while faulted was accepted")
  rejected_after = time.monotonic() - faulted_at
  if rejected.code() != grpc.StatusCode.FAILED_PRECONDITION or rejected_after > MOTION_AFTER_FAULT_S:
    raise RuntimeError(f"Home {rejected_after:.2f} s after the fault: {_code(rejected)}")
  _call(_robot, "ClearFaults")
  result = _move("Home")
  return (
      f"fault {state.fault_reason!r}; Home {rejected_after:.2f} s later -> {_code(rejected)};"
      f" after ClearFaults Home done in {result.duration_s:.2f} s"
  )


def check_cuj7() -> str:
  down = _down()
  waypoints = [
      api.Waypoint(pose=_at(down, down.x + 0.10, down.y - 0.10, down.z - 0.10),
                   motion=api.LINEAR, blend_radius=0.03),
      api.Waypoint(pose=_at(down, down.x + 0.10, down.y + 0.10, down.z - 0.10),
                   motion=api.LINEAR, blend_radius=0.03),
      api.Waypoint(pose=_at(down, down.x, down.y + 0.10, down.z), motion=api.LINEAR),
  ]
  planned = _call(_robot, "PlanOnly", api.PlanOnlyRequest(waypoints=waypoints))
  executed = _move("MoveWaypoints", waypoints=waypoints)
  ratio = executed.duration_s / planned.duration_s
  problems = []
  if abs(ratio - 1) > DURATION_TOLERANCE:
    problems.append(f"MoveWaypoints {executed.duration_s:.3f} s vs PlanOnly {planned.duration_s:.3f} s")
  joint, velocity, duration = JOG
  before = _state().joints.positions[joint - 1]
  _call(_robot, "Jog", api.JogRequest(joint=joint, velocity=velocity, duration_s=duration))
  state = _state()
  moved = state.joints.positions[joint - 1] - before
  if abs(moved - velocity * duration) > JOG_TOLERANCE_RAD:
    problems.append(f"Jog moved joint {joint} by {moved:.4f} rad")
  fk = _call(_robot, "ComputeFk", api.ComputeFkRequest(joints=state.joints, tool=api.TCP)).pose
  fk_error, _ = _pose_error(_matrix(fk), _matrix(state.tool_pose))
  if fk_error > FK_TOLERANCE_M:
    problems.append(f"ComputeFk {fk_error * 1e3:.3f} mm from GetState.tool_pose")
  detail = (
      f"MoveWaypoints {executed.duration_s:.3f} s / PlanOnly {planned.duration_s:.3f} s;"
      f" Jog moved joint {joint} by {moved:.4f} rad; ComputeFk vs tool_pose"
      f" {fk_error * 1e3:.4f} mm"
  )
  if problems:
    raise RuntimeError(f"{'; '.join(problems)} ({detail})")
  return detail


def check_grasp() -> str:
  down = _down()
  cube = {o.name: o for o in _call(_robot, "ListObjects").objects}[CUBE].pose
  centre = _at(down, cube.x, cube.y, cube.z + CUBE_HALF_M)
  _move("MoveToPose", pose=_at(down, cube.x, cube.y, centre.z + PREGRASP_UP_M))
  _move("MoveToPose", pose=centre, motion=api.LINEAR)
  grasped = _gripper("Grasp")
  if not grasped.gripping:
    raise RuntimeError(f"Grasp: width {grasped.width:.4f} m, not gripping")
  _call(_robot, "AttachObject", api.ObjectRequest(name=CUBE))
  z0 = _true_z(CUBE)
  _move("MoveToPose", pose=_at(centre, centre.x, centre.y, centre.z + LIFT_M), motion=api.LINEAR)
  rose = _true_z(CUBE) - z0
  _gripper("Release")
  _call(_robot, "DetachObject", api.ObjectRequest(name=CUBE))
  _move("Home")
  if rose < LIFT_MIN_M:
    raise RuntimeError(f"cube rose {rose * 1e3:.1f} mm on a {LIFT_M * 1e3:.0f} mm lift")
  return f"Grasp width {grasped.width * 1e3:.1f} mm gripping; cube rose {rose * 1e3:.1f} mm"


def check_obstacle() -> str:
  down = _down()
  a, b = (_at(down, OBSTACLE_POSE.x, y, OBSTACLE_Z) for y in OBSTACLE_SIDES_Y)
  _move("MoveToPose", pose=a)
  _call(_sim, "AddObject", api.AddObjectRequest(catalog=OBSTACLE, name=OBSTACLE, pose=OBSTACLE_POSE))
  try:
    blocked = _call(
        _robot, "CheckCollisions",
        api.CheckCollisionsRequest(pose=_at(down, OBSTACLE_POSE.x, 0.0, OBSTACLE_Z)),
    )
    if not blocked.in_collision:
      raise RuntimeError("the straight-line midpoint inside the obstacle is not in collision")
    waypoints = [api.Waypoint(pose=b)]
    planned = _call(_robot, "PlanOnly", api.PlanOnlyRequest(waypoints=waypoints))
    # PlanOnlyResponse has no path: sample the executed plan instead.
    operation = _call(_robot, "MoveWaypoints", api.MoveWaypointsRequest(waypoints=waypoints))
    info = api.OperationInfo()
    operation.metadata.Unpack(info)
    period = info.planned_duration_s / PATH_SAMPLES
    colliding, samples = [], []
    for _ in range(PATH_SAMPLES):
      time.sleep(period)
      q = _state().joints
      samples.append(q)
      if _call(_robot, "CheckCollisions", api.CheckCollisionsRequest(joints=q)).in_collision:
        colliding.append(np.round(q.positions, 3).tolist())
    operation = _call(
        _robot, "WaitOperation",
        operations_pb2.WaitOperationRequest(name=operation.name),
    )
    _done(operation, api.MotionResult)
  finally:
    _call(_robot, "Stop")
    _call(_sim, "RemoveObject", api.ObjectRequest(name=OBSTACLE))
  _move("Home")
  if colliding:
    raise RuntimeError(f"{len(colliding)}/{PATH_SAMPLES} path samples in collision: {colliding[:3]}")
  return (
      f"midpoint in collision; PlanOnly {planned.duration_s:.2f} s around the obstacle;"
      f" {PATH_SAMPLES} samples of the executed path collision-free"
  )


def _png(data: bytes) -> tuple[int, int, int, int]:
  """(width, height, bit depth, colour type) after inflating all rows."""
  if data[:8] != b"\x89PNG\r\n\x1a\n":
    raise RuntimeError("not a PNG")
  pos, idat, header = 8, b"", None
  while pos < len(data):
    (length,) = struct.unpack(">I", data[pos:pos + 4])
    kind, body = data[pos + 4:pos + 8], data[pos + 8:pos + 8 + length]
    if zlib.crc32(kind + body) != struct.unpack(">I", data[pos + 8 + length:pos + 12 + length])[0]:
      raise RuntimeError(f"bad CRC in {kind}")
    if kind == b"IHDR":
      header = struct.unpack(">IIBB", body[:10])
    elif kind == b"IDAT":
      idat += body
    pos += 12 + length
  width, height, depth, colour = header
  bytes_per_pixel = {2: 3, 0: 1}[colour] * depth // 8
  if len(zlib.decompress(idat)) != height * (1 + width * bytes_per_pixel):
    raise RuntimeError("PNG data size does not match IHDR")
  return width, height, depth, colour


def check_misc() -> str:
  details = []
  _move("Home")
  _call(_robot, "SetPayload", api.SetPayloadRequest(mass=PAYLOAD_KG, com_z=0.05))
  state = _state()
  if not state.enabled or state.faulted:
    raise RuntimeError(f"after SetPayload: enabled={state.enabled} fault={state.fault_reason!r}")
  details.append(f"SetPayload({PAYLOAD_KG}) ok")
  goal = list(fso.HOME)
  goal[0] += 0.5
  fast = _move("MoveJoints", joints=api.Joints(positions=goal))
  _move("Home")
  _call(_robot, "SetSpeed", api.SetSpeedRequest(factor=SPEED))
  slow = _move("MoveJoints", joints=api.Joints(positions=goal))
  _call(_robot, "SetSpeed", api.SetSpeedRequest(factor=1.0))
  ratio = slow.duration_s / fast.duration_s
  if abs(ratio * SPEED - 1) > SPEED_TOLERANCE:
    raise RuntimeError(f"SetSpeed({SPEED}): {slow.duration_s:.2f} s vs {fast.duration_s:.2f} s")
  details.append(f"SetSpeed({SPEED}) {fast.duration_s:.2f} -> {slow.duration_s:.2f} s")
  _call(_sim, "ResetScene")
  state = _state()
  if not state.enabled or state.speed_factor != 1.0:
    raise RuntimeError(f"after ResetScene: enabled={state.enabled} speed={state.speed_factor}")
  details.append("ResetScene ok")
  objects = _call(_robot, "ListObjects").objects
  sources = {o.source for o in objects}
  names = {o.name for o in objects}
  if sources != {"ground_truth"} or not set(scene.DEFAULT_OBJECTS) <= names:
    raise RuntimeError(f"ListObjects: {sorted(names)} sources {sources}")
  details.append(f"ListObjects {sorted(names)} ground_truth")
  for kind, want in ((api.RGB, (8, 2)), (api.DEPTH, (16, 0))):
    image = _call(_robot, "GetImage", api.GetImageRequest(kind=kind))
    decoded = _png(image.png)
    if decoded != (WIDTH, HEIGHT, *want) or (image.width, image.height) != (WIDTH, HEIGHT):
      raise RuntimeError(f"GetImage {api.ImageKind.Name(kind)}: PNG {decoded}")
  details.append(f"GetImage RGB/DEPTH {WIDTH}x{HEIGHT} PNG")
  cloud = _call(_robot, "GetPointCloud", api.GetPointCloudRequest())
  xyz = np.array(cloud.xyz, dtype=np.float32).reshape(-1, 3)
  z = xyz[:, 2]
  if (not CLOUD_POINTS[0] <= len(xyz) <= CLOUD_POINTS[1]
      or not np.all((z > CLOUD_Z[0]) & (z < CLOUD_Z[1]))):
    raise RuntimeError(f"GetPointCloud: {len(xyz)} points, z {z.min():.3f}..{z.max():.3f}")
  details.append(f"GetPointCloud {len(xyz)} points, z {z.min():.2f}..{z.max():.2f} m")
  return "; ".join(details)


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
    _wait_ready()
    print(f"robot_service ready after {time.monotonic() - start:.1f} s", flush=True)
    checks = (
        ("logging", check_logging),
        ("cuj2", check_cuj2),
        ("cuj7", check_cuj7),
        ("cuj4", check_cuj4),
        ("grasp", check_grasp),
        ("obstacle", check_obstacle),
        ("cuj5", check_cuj5),
        ("misc", check_misc),
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
