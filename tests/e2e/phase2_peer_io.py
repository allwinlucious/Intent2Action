"""Phase 2 I/O probe (Fast DDS): gripper, wrench, tf, camera and scene services.

Acts as the lockstep peer (echoes each /joint_states stamp with a held arm
target) and runs the checks of phase_2.md in order. Run in the
intrinsic-core:dev image by tests/e2e/phase2_check.py:
  phase2_peer_io.py --scene '<json>' [--checks camera_fresh --repeat 10]
--scene holds the expected poses (built from sim/scenes/default.yaml):
  {"static": {frame: {"position", "rotation"}}, "dynamic": {frame: {"position"}}}
with rotation a 3x3 matrix. Prints one JSON line:
  {"<check>": {"pass": bool, "detail": str}, ...}
"""

import argparse
import collections
import json
import sys
import threading
import time

from geometry_msgs.msg import WrenchStamped
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from simulation_interfaces.msg import Result
from simulation_interfaces.srv import DeleteEntity
from simulation_interfaces.srv import GetEntities
from simulation_interfaces.srv import SpawnEntity
from std_msgs.msg import Header
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

CHECKS = (
    "gripper",
    "wrench",
    "tf",
    "camera",
    "camera_fresh",
    "entities",
    "reset",
)
HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
ARM = [f"panda_joint{i}" for i in range(1, 8)]
FINGER = "panda_finger_joint1"
FIRST_STATE_TIMEOUT_S = 120
WAIT_WALL_S = 60
CALL_TIMEOUT_S = 10
IMAGE_TIMEOUT_S = 20
MOVE_RAD = 0.5
MOVE_S = 1.0
SETTLE_S = 0.5
GRIPPER_EFFORT_N = 20.0
HAND_LOAD_N = np.array([0.0, 0.0, 5.75])  # hand + fingers weight; hand points down at home (phase_2_notes.md)
BOARD_LOAD_N = 0.2 * 9.81
AVERAGE_S = 0.2
EXPECTED_K = [420.08, 0, 424, 0, 420.08, 240, 0, 0, 1]
OPTICAL = "d455_optical"
CAMERA_FRAME = "Camera_Pseudo_Depth"
OBJECT_FILTER = "^/World/objects/[^/]+$"
EXTRA_CUBE = "/World/objects/extra_cube"
EXTRA_CUBE_URI = "C:/isaac-sim-work/repo/assets/objects/red_cube/model.usd"
# A free spot on the table (bottom centre, m); the plan gives no spawn pose.
EXTRA_CUBE_POSITION = (0.30, -0.30, 0.0)
TF_APPEAR_S = 5.0
RESET_SETTLE_S = 1.0
HOME_TOLERANCE_RAD = 0.001
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
TF_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _seconds(header) -> float:
  return header.stamp.sec + header.stamp.nanosec * 1e-9


def _quat_to_matrix(x, y, z, w) -> np.ndarray:
  return np.array(
      [
          [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
      ]
  )


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
  return float(
      np.degrees(np.arccos(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1)))
  )


class Peer:
  """Subscriptions, the echoing lockstep peer, and blocking helpers."""

  def __init__(self):
    self.node = rclpy.create_node("phase2_peer_io")
    self.state = None  # (t, {name: position}, {name: velocity})
    self.echo = True
    self.target = list(HOME)
    self.move = None  # (t0, start, delta)
    self.wrenches = collections.deque(maxlen=2000)  # (t, force)
    self.tf = {}  # child frame -> (wall, parent, translation, rotation)
    self.images = {
        topic: collections.deque(maxlen=8)
        for topic in ("/camera/rgb", "/camera/depth", "/camera/camera_info")
    }
    self.epoch = None
    n = self.node
    self.command = n.create_publisher(JointState, "/joint_command", QOS)
    self.gripper = n.create_publisher(JointState, "/gripper_command", QOS)
    self.request = n.create_publisher(Header, "/camera/request", QOS)
    n.create_subscription(JointState, "/joint_states", self._on_state, QOS)
    n.create_subscription(WrenchStamped, "/wrist_wrench", self._on_wrench, QOS)
    # Two bridge nodes publish /tf back to back; depth 1 would drop one.
    n.create_subscription(TFMessage, "/tf", self._on_tf, TF_QOS)
    n.create_subscription(UInt32, "/sim/epoch", self._on_epoch, LATCHED)
    for topic in ("/camera/rgb", "/camera/depth"):
      n.create_subscription(
          Image, topic, lambda m, t=topic: self.images[t].append(m), QOS
      )
    n.create_subscription(
        CameraInfo,
        "/camera/camera_info",
        lambda m: self.images["/camera/camera_info"].append(m),
        QOS,
    )
    self.clients = {
        name: n.create_client(srv, name)
        for name, srv in (
            ("/sim/reset", Trigger),
            ("/sim/attach_board", Trigger),
            ("/sim/detach_board", Trigger),
            ("/get_entities", GetEntities),
            ("/spawn_entity", SpawnEntity),
            ("/delete_entity", DeleteEntity),
        )
    }
    self.executor = SingleThreadedExecutor()
    self.executor.add_node(n)
    threading.Thread(target=self.executor.spin, daemon=True).start()

  def _on_state(self, msg: JointState) -> None:
    t = _seconds(msg.header)
    self.state = (
        t,
        dict(zip(msg.name, msg.position)),
        dict(zip(msg.name, msg.velocity)),
    )
    if not self.echo:
      return
    velocity = [0.0] * 7
    if self.move is not None:
      t0, start, delta = self.move
      s = min(max((t - t0) / MOVE_S, 0.0), 1.0)
      self.target[0] = start + delta * s
      if s < 1.0:
        velocity[0] = delta / MOVE_S
      else:
        self.move = None
    out = JointState(name=ARM, position=list(self.target), velocity=velocity)
    out.header.stamp = msg.header.stamp
    self.command.publish(out)

  def _on_wrench(self, msg: WrenchStamped) -> None:
    f = msg.wrench.force
    self.wrenches.append((_seconds(msg.header), np.array([f.x, f.y, f.z])))

  def _on_tf(self, msg: TFMessage) -> None:
    wall = time.monotonic()
    for tr in msg.transforms:
      p, q = tr.transform.translation, tr.transform.rotation
      self.tf[tr.child_frame_id] = (
          wall,
          tr.header.frame_id,
          np.array([p.x, p.y, p.z]),
          _quat_to_matrix(q.x, q.y, q.z, q.w),
      )

  def _on_epoch(self, msg: UInt32) -> None:
    self.epoch = msg.data

  # Blocking helpers (main thread).
  def now(self) -> float:
    return self.state[0]

  def wait_until(self, predicate, timeout_s: float = WAIT_WALL_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while not predicate():
      if time.monotonic() > deadline:
        return False
      time.sleep(0.005)
    return True

  def wait_sim(self, dt: float, predicate=lambda: False) -> bool:
    """Waits dt of simulation time; True as soon as predicate holds."""
    end = self.now() + dt
    self.wait_until(lambda: predicate() or self.now() >= end)
    return predicate()

  def move_joint1(self, delta: float) -> None:
    self.move = (self.now(), self.target[0], delta)
    self.wait_until(lambda: self.move is None)
    self.wait_sim(SETTLE_S)

  def mean_force(self) -> np.ndarray:
    self.wait_sim(AVERAGE_S)
    t_end = self.now()
    forces = [f for t, f in list(self.wrenches) if t > t_end - AVERAGE_S]
    return np.mean(forces, axis=0)

  def call(self, name: str, request):
    client = self.clients[name]
    if not client.wait_for_service(timeout_sec=CALL_TIMEOUT_S):
      return None
    future = client.call_async(request)
    # sim_control only works inside renders; Isaac renders on /camera/request
    # (otherwise only every render_period_wall_s).
    deadline = time.monotonic() + CALL_TIMEOUT_S
    while not future.done() and time.monotonic() < deadline:
      self._request_render()
      self.wait_until(future.done, 0.25)
    return future.result() if future.done() else None

  def _request_render(self) -> None:
    header = Header()
    header.stamp.sec, header.stamp.nanosec = divmod(
        round(self.now() * 1e9), 1_000_000_000
    )
    self.request.publish(header)

  def request_image(self, stamp: float) -> dict:
    """Requests a frame showing sim time >= stamp; the first message per topic."""
    header = Header()
    header.stamp.sec, header.stamp.nanosec = divmod(
        round(stamp * 1e9), 1_000_000_000
    )
    self.request.publish(header)
    found = {}

    def complete() -> bool:
      for topic, queue in self.images.items():
        if topic not in found:
          fresh = [m for m in list(queue) if _seconds(m.header) >= stamp - 1e-6]
          if fresh:
            found[topic] = fresh[0]
      return len(found) == len(self.images)

    self.wait_until(complete, IMAGE_TIMEOUT_S)
    return found

  def frames_seen_since(self, wall: float) -> set:
    return {f for f, (w, *_) in self.tf.items() if w >= wall}


def _rgb(msg: Image) -> np.ndarray:
  return (
      np.frombuffer(bytes(msg.data), dtype=np.uint8)
      .reshape(msg.height, msg.width, 3)
      .astype(np.float32)
  )


def check_gripper(p: Peer, _) -> tuple[bool, str]:
  results = []
  for width, done in ((0.0, lambda q: q < 0.002), (0.08, lambda q: q > 0.038)):
    command = JointState(
        name=[FINGER], position=[width / 2], effort=[GRIPPER_EFFORT_N]
    )
    command.header.stamp.sec, command.header.stamp.nanosec = divmod(
        round(p.now() * 1e9), 1_000_000_000
    )
    p.gripper.publish(command)
    ok = p.wait_sim(1.0, lambda: done(p.state[1][FINGER]))
    results.append((ok, f"width {width}: finger {p.state[1][FINGER]:.4f} m"))
  return all(ok for ok, _ in results), "; ".join(d for _, d in results)


def check_wrench(p: Peer, _) -> tuple[bool, str]:
  p.wait_sim(SETTLE_S)
  base = p.mean_force()
  base_ok = np.linalg.norm(base - HAND_LOAD_N) <= 0.3
  attach = p.call("/sim/attach_board", Trigger.Request())
  p.wait_sim(1.0)
  loaded = p.mean_force()
  rise = np.linalg.norm(loaded) - np.linalg.norm(base)
  rise_ok = (
      bool(attach and attach.success)
      and abs(rise - BOARD_LOAD_N) <= 0.05 * BOARD_LOAD_N
  )
  detach = p.call("/sim/detach_board", Trigger.Request())
  p.wait_sim(1.0)
  after = p.mean_force()
  back = np.linalg.norm(after - base)
  back_ok = bool(detach and detach.success) and back <= 0.1
  detail = (
      f"home {np.round(base, 3).tolist()} N (off by"
      f" {np.linalg.norm(base - HAND_LOAD_N):.3f}); board rise {rise:.3f} N"
      f" (want {BOARD_LOAD_N:.3f} ±5%, attach"
      f" {attach.message if attach else 'no reply'}); after detach off by"
      f" {back:.3f} N ({detach.message if detach else 'no reply'})"
  )
  return base_ok and rise_ok and back_ok, detail


def _pose_errors(
    p: Peer, scene: dict, objects_only: bool = False
) -> tuple[bool, list[str]]:
  ok, details = True, []
  for frame, want in scene["static"].items():
    if objects_only and frame == CAMERA_FRAME:
      continue
    if frame not in p.tf:
      ok = False
      details.append(f"{frame} missing")
      continue
    _, parent, position, rotation = p.tf[frame]
    mm = np.linalg.norm(position - want["position"]) * 1000
    deg = _angle_deg(rotation, np.array(want["rotation"]))
    ok &= parent == "world" and mm <= 1.0 and deg <= 0.1
    details.append(f"{frame} {mm:.3f} mm {deg:.3f} deg")
  for frame, want in scene["dynamic"].items():
    if frame not in p.tf:
      ok = False
      details.append(f"{frame} missing")
      continue
    _, parent, position, _ = p.tf[frame]
    mm = np.linalg.norm(position - want["position"]) * 1000
    ok &= parent == "world" and mm <= 5.0
    details.append(f"{frame} {mm:.2f} mm")
  return ok, details


def check_tf(p: Peer, scene: dict) -> tuple[bool, str]:
  p.wait_until(lambda: p.now() >= 2.0)
  p.wait_sim(0.5)
  frames = set(scene["static"]) | set(scene["dynamic"]) | {OPTICAL}
  missing = frames - set(p.tf)
  optical_ok = OPTICAL in p.tf and p.tf[OPTICAL][1] == CAMERA_FRAME
  poses_ok, details = _pose_errors(p, scene)
  return (
      not missing and optical_ok and poses_ok,
      f"missing {sorted(missing)}; {OPTICAL} parent"
      f" {p.tf[OPTICAL][1] if OPTICAL in p.tf else None}; "
      + ", ".join(details),
  )


def check_camera(p: Peer, _) -> tuple[bool, str]:
  stamp = p.now()
  found = p.request_image(stamp)
  rgb = found.get("/camera/rgb")
  depth = found.get("/camera/depth")
  info = found.get("/camera/camera_info")
  if not (rgb and depth and info):
    return False, f"received {sorted(found)} within {IMAGE_TIMEOUT_S} s"
  ok = (
      (rgb.encoding, rgb.width, rgb.height) == ("rgb8", 848, 480)
      and (depth.encoding, depth.width, depth.height) == ("32FC1", 848, 480)
      and np.allclose(info.k, EXPECTED_K, atol=0.1)
      and {m.header.frame_id for m in (rgb, depth, info)} == {OPTICAL}
  )
  depth_m = np.frombuffer(bytes(depth.data), dtype=np.float32)
  finite = depth_m[np.isfinite(depth_m)]
  return ok, (
      f"rgb {rgb.encoding} {rgb.width}x{rgb.height}, depth {depth.encoding}"
      f" {depth.width}x{depth.height} median {np.median(finite):.3f} m, K"
      f" {np.round(info.k, 2).tolist()}, frames"
      f" {sorted({m.header.frame_id for m in (rgb, depth, info)})}, stamps"
      f" {[round(_seconds(m.header) - stamp, 4) for m in (rgb, depth, info)]}"
      " s after request"
  )


def check_camera_fresh(p: Peer, _) -> tuple[bool, str]:
  before = p.request_image(p.now()).get("/camera/rgb")
  p.move_joint1(MOVE_RAD)
  stamp = p.now()
  sent = time.monotonic()
  after = p.request_image(stamp).get("/camera/rgb")
  latency = time.monotonic() - sent
  p.move_joint1(-MOVE_RAD)
  if before is None or after is None:
    return False, "no frame"
  diff = float(np.mean(np.abs(_rgb(after) - _rgb(before))))
  return diff > 2.0, (
      f"mean abs diff {diff:.2f}; frame stamped"
      f" {_seconds(after.header) - stamp:+.4f} s, received {latency:.2f} s wall"
      " after the request"
  )


def _entities(p: Peer):
  request = GetEntities.Request()
  request.filters.filter = OBJECT_FILTER
  return p.call("/get_entities", request)


def _spawn_extra_cube(p: Peer):
  request = SpawnEntity.Request()
  request.name = EXTRA_CUBE
  request.allow_renaming = False
  request.uri = EXTRA_CUBE_URI
  pose = request.initial_pose.pose
  pose.position.x, pose.position.y, pose.position.z = EXTRA_CUBE_POSITION
  pose.orientation.w = 1.0
  return p.call("/spawn_entity", request)


def check_entities(p: Peer, _) -> tuple[bool, str]:
  listed = _entities(p)
  list_ok = listed is not None and "/World/objects/red_cube" in listed.entities
  spawned = _spawn_extra_cube(p)
  spawn_ok = spawned is not None and spawned.result.result == Result.RESULT_OK
  seen_from = time.monotonic()
  on_tf = spawn_ok and p.wait_until(
      lambda: "extra_cube" in p.frames_seen_since(seen_from), TF_APPEAR_S
  )
  request = DeleteEntity.Request()
  request.entity = EXTRA_CUBE
  deleted = p.call("/delete_entity", request)
  delete_ok = deleted is not None and deleted.result.result == Result.RESULT_OK
  after = _entities(p)
  gone = after is not None and EXTRA_CUBE not in after.entities
  return list_ok and spawn_ok and on_tf and delete_ok and gone, (
      f"listed {list(listed.entities) if listed else None}; spawn"
      f" {spawned.result.result if spawned else None}"
      f" {spawned.result.error_message if spawned else ''}; on /tf {on_tf};"
      f" delete {deleted.result.result if deleted else None}; gone {gone}"
  )


def check_reset(p: Peer, scene: dict) -> tuple[bool, str]:
  p.move_joint1(MOVE_RAD)
  spawned = _spawn_extra_cube(p)
  seen_from = time.monotonic()
  p.wait_until(
      lambda: "extra_cube" in p.frames_seen_since(seen_from), TF_APPEAR_S
  )
  epoch_before = p.epoch
  p.echo = False
  reply = p.call("/sim/reset", Trigger.Request())
  p.wait_sim(RESET_SETTLE_S)
  error = max(abs(p.state[1][j] - h) for j, h in zip(ARM, HOME))
  after = _entities(p)
  gone = after is not None and EXTRA_CUBE not in after.entities
  poses_ok, details = _pose_errors(p, scene, objects_only=True)
  epoch_ok = p.wait_until(
      # Fast DDS late joiners miss the latched value across Windows <-> WSL
      # (phase_2_notes.md), so with no earlier value expect the first reset: 1.
      lambda: p.epoch == (1 if epoch_before is None else epoch_before + 1),
      5.0,
  )
  p.target = list(HOME)
  p.echo = True
  ok = bool(reply and reply.success) and error <= HOME_TOLERANCE_RAD
  return ok and gone and poses_ok and epoch_ok, (
      f"spawn {spawned.result.result if spawned else None}; reset"
      f" {reply.message if reply else 'no reply'}; max joint error"
      f" {error:.5f} rad; extra_cube gone {gone}; epoch {epoch_before} ->"
      f" {p.epoch}; " + ", ".join(details)
  )


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--scene", required=True)
  parser.add_argument("--checks", default=",".join(CHECKS))
  parser.add_argument("--repeat", type=int, default=1)
  args = parser.parse_args()
  scene = json.loads(args.scene)

  rclpy.init()
  peer = Peer()
  results = {}
  if not peer.wait_until(lambda: peer.state is not None, FIRST_STATE_TIMEOUT_S):
    print(json.dumps({"io": {"pass": False, "detail": "no /joint_states"}}))
    return 1
  for name in args.checks.split(","):
    outcomes = [
        globals()[f"check_{name}"](peer, scene) for _ in range(args.repeat)
    ]
    passed = sum(ok for ok, _ in outcomes)
    detail = " | ".join(d for _, d in outcomes)
    if args.repeat > 1:
      detail = f"{passed}/{args.repeat} passed: {detail}"
    results[name] = {"pass": bool(passed == args.repeat), "detail": detail}
    print(f"CHECK {name} {results[name]}", file=sys.stderr, flush=True)
  print(json.dumps(results), flush=True)
  peer.executor.shutdown()
  return 0


if __name__ == "__main__":
  sys.exit(main())
