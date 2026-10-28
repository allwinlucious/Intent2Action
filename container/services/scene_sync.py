"""Mirrors Isaac's objects into the world service and serves SimLink (phase_3.md decision 15).

A Fast DDS ROS 2 node in the image, started by container/entrypoint.sh:
- /tf frames with parent `world` (except the camera and the hand) are objects. Catalog objects are
  created in `sim_world` and, with belief_source GROUND_TRUTH, in `world`; their poses follow /tf; a
  name absent from /tf for 2 s is deleted from both worlds.
- SimLink on 127.0.0.1:18101 (robot_service/simlink.proto): every ROS 2 service call to Isaac.
"""

import argparse
from concurrent import futures
import functools
import logging
from pathlib import Path
import re
import sys
import threading
import time

# robot_service.py next to this file would shadow the generated (namespace) package `robot_service`.
sys.path.pop(0)

from google.protobuf import empty_pb2
from google.protobuf import text_format
from google.protobuf import wrappers_pb2
import grpc
from intrinsic.geometry.proto import geometry_service_pb2
from intrinsic.geometry.proto import geometry_service_pb2_grpc
from intrinsic.geometry.proto import geometry_service_types_pb2
from intrinsic.math.proto import point_pb2
from intrinsic.math.proto import pose_pb2
from intrinsic.math.proto import quaternion_pb2
from intrinsic.world.proto import object_world_refs_pb2 as refs_pb
from intrinsic.world.proto import object_world_service_pb2 as world_pb
from intrinsic.world.proto import object_world_service_pb2_grpc as world_grpc
from intrinsic.world.proto import object_world_updates_pb2 as updates_pb
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from robot_service import robot_service_pb2
from robot_service import simlink_pb2
from robot_service import simlink_pb2_grpc
from rosgraph_msgs.msg import Clock
from simulation_interfaces.msg import Result
from simulation_interfaces.srv import DeleteEntity
from simulation_interfaces.srv import SpawnEntity
from std_msgs.msg import Header
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import yaml

APP = Path("/opt/app")
SCENE_PATH = APP / "config/default.yaml"
SETTINGS_PATH = APP / "config/robot_service.yaml"
CATALOG_DIR = APP / "assets/objects"
ISAAC_CATALOG_DIR = "C:/isaac-sim-work/repo/assets/objects"
OBJECTS_PRIM = "/World/objects"
WORLD = "world"
SIM_WORLD = "sim_world"
NOT_OBJECTS = {"Camera_Pseudo_Depth", "panda_hand"}  # world children on /tf that are not objects
ROBOT_OBJECTS = {"franka", "franka_hand"}
VISUAL_SET, COLLISION_SET = "Intrinsic_Visual", "Intrinsic_Collision"
# Object names in the world service: a letter, then letters, digits or '_'.
NAME_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
ABSENT_S = 2.0
STALE_AFTER_S = 5.0  # after the first /tf, ground-truth objects not on /tf are left over from earlier runs
CALL_TIMEOUT_S = 10.0
RESET_EPOCH_TIMEOUT_S = 5.0
RENDER_REQUEST_PERIOD_S = 0.25  # sim_control handles calls only inside renders (phase 2)
RPC_TIMEOUT_S = 30.0
CHANNEL_WAIT_S = 60.0
REPEAT_LOG_S = 10.0
SUMMARY_MAX_CHARS = 200
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
# Two bridge nodes publish /tf back to back; depth 1 would drop one.
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
SPAWN_CODES = {
    Result.RESULT_NOT_FOUND: grpc.StatusCode.NOT_FOUND,
    SpawnEntity.Response.NAME_NOT_UNIQUE: grpc.StatusCode.ALREADY_EXISTS,
    SpawnEntity.Response.NAME_INVALID: grpc.StatusCode.INVALID_ARGUMENT,
    SpawnEntity.Response.INVALID_POSE: grpc.StatusCode.INVALID_ARGUMENT,
}

log = logging.getLogger("scene_sync")

Pose = tuple[float, float, float, float, float, float, float]  # x y z qx qy qz qw


def _seconds(stamp) -> float:
  return stamp.sec + stamp.nanosec * 1e-9


def _stamp(header: Header, t: float) -> None:
  header.stamp.sec, header.stamp.nanosec = divmod(round(t * 1e9), 1_000_000_000)


def _by_name(name: str) -> refs_pb.ObjectReference:
  return refs_pb.ObjectReference(by_name=refs_pb.ObjectReferenceByName(object_name=name))


class Ros:
  """The node: latest /tf, /clock and /sim/epoch, and blocking calls to Isaac's services."""

  def __init__(self):
    self.node = n = rclpy.create_node("scene_sync")
    self.lock = threading.Lock()
    self.poses: dict[str, Pose] = {}  # latest /tf pose of every frame whose parent is world
    self.pending: dict[str, Pose] = {}  # object poses not yet written to the worlds
    self.tf_arrived = threading.Event()
    self.clock: float | None = None
    self.epoch: int | None = None
    self.request = n.create_publisher(Header, "/camera/request", QOS)
    n.create_subscription(TFMessage, "/tf", self._on_tf, TF_QOS)
    n.create_subscription(Clock, "/clock", self._on_clock, QOS)
    n.create_subscription(UInt32, "/sim/epoch", self._on_epoch, LATCHED)
    self.clients = {
        name: n.create_client(srv, name)
        for name, srv in (
            ("/spawn_entity", SpawnEntity),
            ("/delete_entity", DeleteEntity),
            ("/sim/reset", Trigger),
            ("/sim/attach_board", Trigger),
            ("/sim/detach_board", Trigger),
        )
    }
    self.executor = SingleThreadedExecutor()
    self.executor.add_node(n)
    threading.Thread(target=self.executor.spin, daemon=True).start()

  def _on_tf(self, msg: TFMessage) -> None:
    with self.lock:
      for tr in msg.transforms:
        if tr.header.frame_id != WORLD:
          continue
        p, q = tr.transform.translation, tr.transform.rotation
        pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        self.poses[tr.child_frame_id] = pose
        if tr.child_frame_id not in NOT_OBJECTS:
          self.pending[tr.child_frame_id] = pose
    if self.pending:
      self.tf_arrived.set()

  def _on_clock(self, msg: Clock) -> None:
    self.clock = _seconds(msg.clock)

  def _on_epoch(self, msg: UInt32) -> None:
    if msg.data != self.epoch:
      log.info("sim epoch %s (was %s)", msg.data, self.epoch)
    self.epoch = msg.data

  def take_pending(self) -> dict[str, Pose]:
    with self.lock:
      pending, self.pending = self.pending, {}
      self.tf_arrived.clear()
    return pending

  def call(self, context, service: str, request, request_renders: bool):
    """Calls an Isaac service; aborts the RPC with UNAVAILABLE or DEADLINE_EXCEEDED."""
    client = self.clients[service]
    if not client.wait_for_service(timeout_sec=CALL_TIMEOUT_S):
      context.abort(grpc.StatusCode.UNAVAILABLE, f"Isaac's {service} is not available")
    done = threading.Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _: done.set())
    deadline = time.monotonic() + CALL_TIMEOUT_S
    while not done.wait(RENDER_REQUEST_PERIOD_S if request_renders else CALL_TIMEOUT_S):
      if time.monotonic() > deadline:
        client.remove_pending_request(future)
        context.abort(
            grpc.StatusCode.DEADLINE_EXCEEDED, f"Isaac's {service} did not answer within {CALL_TIMEOUT_S} s"
        )
      if request_renders and self.clock is not None:
        header = Header()
        _stamp(header, self.clock)
        self.request.publish(header)
    return future.result()


class Sync:
  """Keeps sim_world (and world with GROUND_TRUTH) equal to the objects on /tf."""

  def __init__(self, ros: Ros, world, geometry, catalogs: dict[str, str], mirror_world: bool):
    self.ros = ros
    self.world = world
    self.geometry = geometry
    self.catalogs = catalogs  # object name -> catalog name; Spawn adds to it
    self.worlds = [SIM_WORLD, WORLD] if mirror_world else [SIM_WORLD]
    self.attached: set[str] = set()
    self.known: set[str] = set()  # objects created by this process
    self.last_seen: dict[str, float] = {}
    self.geometry_refs = {}  # catalog -> GeometryStorageRefs
    self.unknown_logged: set[str] = set()
    self.error_logged: dict[str, float] = {}

  def ensure_sim_world(self) -> None:
    worlds = self.world.ListWorlds(world_pb.ListWorldsRequest(), timeout=RPC_TIMEOUT_S)
    if SIM_WORLD in {w.id for w in worlds.world_metadatas}:
      return
    self.world.CloneWorld(
        world_pb.CloneWorldRequest(world_id=WORLD, cloned_world_id=SIM_WORLD), timeout=RPC_TIMEOUT_S
    )
    removed = [name for name in self._object_names(SIM_WORLD) if name not in ROBOT_OBJECTS]
    for name in removed:
      self._delete(SIM_WORLD, name, reparent=True)
    log.info("world created world=%s cloned_from=%s removed=%s", SIM_WORLD, WORLD, removed)

  def run(self) -> None:
    first_tf = None
    stale_removed = False
    while True:
      self.ros.tf_arrived.wait()
      poses = self.ros.take_pending()
      now = time.monotonic()
      first_tf = first_tf or now
      try:
        self._sync(poses, now)
        if not stale_removed and now - first_tf >= STALE_AFTER_S:
          self._delete_stale(now)
          stale_removed = True
      except grpc.RpcError as error:
        self._log_error(f"world update failed: {error.code().name}: {error.details()}")
        # Objects removed from a world by someone else are re-created on their next sight.
        try:
          self.known.intersection_update(*(self._object_names(w) for w in self.worlds))
        except grpc.RpcError:
          pass  # the next failure is logged

  def _sync(self, poses: dict[str, Pose], now: float) -> None:
    for name in poses:
      self.last_seen[name] = now
    new = [n for n in poses if n not in self.known and self._catalog(n)]
    if new:
      existing = {w: self._object_names(w) for w in self.worlds}
      for name in new:
        self._create(name, poses[name], existing)
    for w in self.worlds:
      updates = [
          updates_pb.ObjectWorldUpdate(
              update_transform=updates_pb.UpdateTransformRequest(
                  node_a=refs_pb.TransformNodeReference(id="root"),
                  node_b=refs_pb.TransformNodeReference(
                      by_name=refs_pb.TransformNodeReferenceByName(
                          object=refs_pb.ObjectReferenceByName(object_name=name)
                      )
                  ),
                  a_t_b=self._pose_proto(pose),
              )
          )
          for name, pose in poses.items()
          if name in self.known and name not in new and name not in self.attached
      ]
      if updates:
        self.world.UpdateWorldResources(
            world_pb.UpdateWorldResourcesRequest(
                world_id=w,
                world_updates=updates_pb.ObjectWorldUpdates(updates=updates),
                view=updates_pb.BASIC,
            ),
            timeout=RPC_TIMEOUT_S,
        )
    for name in [n for n in self.known if now - self.last_seen[n] > ABSENT_S]:
      self.known.discard(name)
      for w in self.worlds:
        self._delete(w, name)
      log.info("object deleted name=%s worlds=%s reason=absent_from_tf_%.0fs", name, self.worlds, ABSENT_S)

  def _delete_stale(self, now: float) -> None:
    on_tf = {n for n, t in self.last_seen.items() if now - t <= ABSENT_S}
    for w in (SIM_WORLD, WORLD):
      objects = self.world.ListObjects(
          world_pb.ListObjectsRequest(world_id=w, view=updates_pb.FULL), timeout=RPC_TIMEOUT_S
      ).objects
      for o in objects:
        if o.name in on_tf or "source" not in o.object_component.user_data:
          continue
        source = wrappers_pb2.StringValue()
        o.object_component.user_data["source"].Unpack(source)
        if source.value == "ground_truth":
          self._delete(w, o.name)
          self.known.discard(o.name)
          log.info("object deleted name=%s world=%s reason=stale_not_on_tf", o.name, w)

  def _catalog(self, name: str) -> str | None:
    catalog = self.catalogs.get(name)
    if catalog is None and name not in self.unknown_logged:
      self.unknown_logged.add(name)
      log.warning("ignoring /tf frame %s: no catalog entry for it", name)
    return catalog

  def _create(self, name: str, pose: Pose, existing: dict[str, set[str]]) -> None:
    catalog = self.catalogs[name]
    refs = self._geometry(catalog)
    for w in self.worlds:
      if name in existing[w]:
        self._delete(w, name)
      request = updates_pb.CreateObjectRequest(
          world_id=w,
          name=name,
          name_is_global_alias=True,
          parent_object=refs_pb.ObjectReferenceWithEntityFilter(
              reference=refs_pb.ObjectReference(id="root"),
              entity_filter=refs_pb.ObjectEntityFilter(include_final_entity=True),
          ),
          parent_object_t_created_object=self._pose_proto(pose),
          view=updates_pb.BASIC,
      )
      geometries = request.create_single_entity_object.geometry_component.named_geometries
      for set_name in (VISUAL_SET, COLLISION_SET):
        geometries[set_name].named_geometries[catalog].geometry.geo_ref.CopyFrom(refs)
      request.user_data["source"].Pack(wrappers_pb2.StringValue(value="ground_truth"))
      self.world.CreateObject(request, timeout=RPC_TIMEOUT_S)
      log.info(
          "object created name=%s catalog=%s world=%s replaced=%s pose=%s",
          name, catalog, w, name in existing[w], [round(v, 4) for v in pose],
      )
    self.known.add(name)

  def _geometry(self, catalog: str):
    if catalog not in self.geometry_refs:
      glb = (CATALOG_DIR / catalog / "mesh.glb").read_bytes()
      response = self.geometry.CreateGeometry(
          geometry_service_pb2.CreateGeometryRequest(data=geometry_service_types_pb2.GeometryData(gltf_bytes=glb)),
          timeout=RPC_TIMEOUT_S,
      )
      self.geometry_refs[catalog] = response.geometry_storage_refs
    return self.geometry_refs[catalog]

  def _object_names(self, world_id: str) -> set[str]:
    objects = self.world.ListObjects(world_pb.ListObjectsRequest(world_id=world_id), timeout=RPC_TIMEOUT_S).objects
    return {o.name for o in objects} - {"root"}

  def _delete(self, world_id: str, name: str, reparent: bool = False) -> None:
    self.world.DeleteObject(
        updates_pb.DeleteObjectRequest(world_id=world_id, object=_by_name(name), reparent_child_objects=reparent),
        timeout=RPC_TIMEOUT_S,
    )

  @staticmethod
  def _pose_proto(pose: Pose) -> pose_pb2.Pose:
    x, y, z, qx, qy, qz, qw = pose
    return pose_pb2.Pose(
        position=point_pb2.Point(x=x, y=y, z=z), orientation=quaternion_pb2.Quaternion(x=qx, y=qy, z=qz, w=qw)
    )

  def _log_error(self, message: str) -> None:
    now = time.monotonic()
    key = re.sub(r"\d+", "#", message)  # repeats differ only in IDs and timestamps
    if now - self.error_logged.get(key, -REPEAT_LOG_S) >= REPEAT_LOG_S:
      self.error_logged[key] = now
      log.error("%s (repeats are logged every %.0f s)", message, REPEAT_LOG_S)


def _logged(method):
  """Logs one line per SimLink call: method, request, duration, status."""

  @functools.wraps(method)
  def wrapper(self, request, context):
    start = time.monotonic()
    summary = text_format.MessageToString(request, as_one_line=True)[:SUMMARY_MAX_CHARS]
    try:
      response = method(self, request, context)
    except Exception:
      code = context.code() or grpc.StatusCode.UNKNOWN
      log.warning(
          "SimLink.%s request={%s} ms=%.0f status=%s message=%s",
          method.__name__, summary, (time.monotonic() - start) * 1e3, code.name, context.details(),
      )
      raise
    result = text_format.MessageToString(response, as_one_line=True)[:SUMMARY_MAX_CHARS]
    log.info(
        "SimLink.%s request={%s} ms=%.0f status=OK result={%s}",
        method.__name__, summary, (time.monotonic() - start) * 1e3, result,
    )
    return response

  return wrapper


class SimLink(simlink_pb2_grpc.SimLinkServicer):

  def __init__(self, ros: Ros, sync: Sync):
    self.ros = ros
    self.sync = sync

  @_logged
  def Spawn(self, request, context):
    if not NAME_PATTERN.fullmatch(request.name):
      context.abort(grpc.StatusCode.INVALID_ARGUMENT, f"invalid object name {request.name!r} (want {NAME_PATTERN.pattern})")
    if not (CATALOG_DIR / request.catalog / "mesh.glb").is_file():
      context.abort(grpc.StatusCode.NOT_FOUND, f"no catalog entry {request.catalog!r} in {CATALOG_DIR}")
    self.sync.catalogs[request.name] = request.catalog
    spawn = SpawnEntity.Request(
        name=f"{OBJECTS_PRIM}/{request.name}",
        allow_renaming=False,
        uri=f"{ISAAC_CATALOG_DIR}/{request.catalog}/model.usd",
    )
    p, pose = request.pose, spawn.initial_pose.pose
    pose.position.x, pose.position.y, pose.position.z = p.x, p.y, p.z
    o = pose.orientation
    o.x, o.y, o.z, o.w = (p.qx, p.qy, p.qz, p.qw) if any((p.qx, p.qy, p.qz, p.qw)) else (0.0, 0.0, 0.0, 1.0)
    result = self.ros.call(context, "/spawn_entity", spawn, request_renders=True).result
    if result.result != Result.RESULT_OK:
      context.abort(
          SPAWN_CODES.get(result.result, grpc.StatusCode.INTERNAL),
          f"Isaac SpawnEntity {request.name}: result {result.result}: {result.error_message}",
      )
    return empty_pb2.Empty()

  @_logged
  def Delete(self, request, context):
    delete = DeleteEntity.Request(entity=f"{OBJECTS_PRIM}/{request.name}")
    result = self.ros.call(context, "/delete_entity", delete, request_renders=True).result
    if result.result != Result.RESULT_OK:
      context.abort(
          SPAWN_CODES.get(result.result, grpc.StatusCode.INTERNAL),
          f"Isaac DeleteEntity {request.name}: result {result.result}: {result.error_message}",
      )
    return empty_pb2.Empty()

  @_logged
  def Reset(self, request, context):
    before = self.ros.epoch
    self._trigger(context, "/sim/reset")
    deadline = time.monotonic() + RESET_EPOCH_TIMEOUT_S
    while self.ros.epoch == before:
      if time.monotonic() > deadline:
        context.abort(
            grpc.StatusCode.DEADLINE_EXCEEDED,
            f"/sim/reset succeeded but /sim/epoch stayed {before} for {RESET_EPOCH_TIMEOUT_S} s",
        )
      time.sleep(0.01)
    return simlink_pb2.ResetResponse(epoch=self.ros.epoch)

  @_logged
  def AttachBoard(self, request, context):
    self._trigger(context, "/sim/attach_board")
    return empty_pb2.Empty()

  @_logged
  def DetachBoard(self, request, context):
    self._trigger(context, "/sim/detach_board")
    return empty_pb2.Empty()

  @_logged
  def SetAttached(self, request, context):
    if request.attached:
      self.sync.attached.add(request.name)
    else:
      self.sync.attached.discard(request.name)
    return empty_pb2.Empty()

  @_logged
  def GetTruePose(self, request, context):
    pose = self.ros.poses.get(request.frame)
    if pose is None:
      context.abort(grpc.StatusCode.NOT_FOUND, f"no /tf frame {request.frame!r} with parent {WORLD}")
    return robot_service_pb2.Pose(**dict(zip(("x", "y", "z", "qx", "qy", "qz", "qw"), pose)))

  @_logged
  def GetClock(self, request, context):
    if self.ros.clock is None:
      context.abort(grpc.StatusCode.UNAVAILABLE, "no /clock received from Isaac yet")
    return simlink_pb2.ClockResponse(sim_time=self.ros.clock)

  def _trigger(self, context, service: str) -> None:
    reply = self.ros.call(context, service, Trigger.Request(), request_renders=False)
    if not reply.success:
      context.abort(grpc.StatusCode.FAILED_PRECONDITION, f"Isaac {service}: {reply.message}")


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--world", default="localhost:18000")
  parser.add_argument("--geometry", default="localhost:18001")
  parser.add_argument("--port", type=int, default=18101)
  args = parser.parse_args()
  logging.basicConfig(
      stream=sys.stdout, level=logging.INFO, format="%(asctime)s %(levelname)s scene_sync %(message)s"
  )
  catalogs = {name: spec["catalog"] for name, spec in yaml.safe_load(SCENE_PATH.read_text())["objects"].items()}
  catalogs["calib_board"] = "calib_board"
  belief_source = yaml.safe_load(SETTINGS_PATH.read_text())["belief_source"]
  if belief_source not in ("GROUND_TRUTH", "PERCEPTION"):
    log.error("belief_source must be GROUND_TRUTH or PERCEPTION, got %r (%s)", belief_source, SETTINGS_PATH)
    return 1

  world_channel = grpc.insecure_channel(args.world)
  geometry_channel = grpc.insecure_channel(args.geometry)
  for channel in (world_channel, geometry_channel):
    grpc.channel_ready_future(channel).result(timeout=CHANNEL_WAIT_S)
  rclpy.init()
  ros = Ros()
  sync = Sync(
      ros,
      world_grpc.ObjectWorldServiceStub(world_channel),
      geometry_service_pb2_grpc.GeometryServiceStub(geometry_channel),
      catalogs,
      mirror_world=belief_source == "GROUND_TRUTH",
  )
  sync.ensure_sim_world()

  server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))
  simlink_pb2_grpc.add_SimLinkServicer_to_server(SimLink(ros, sync), server)
  server.add_insecure_port(f"127.0.0.1:{args.port}")
  server.start()
  log.info(
      "started simlink=127.0.0.1:%d belief_source=%s worlds=%s catalog_names=%s",
      args.port, belief_source, sync.worlds, sorted(catalogs),
  )
  sync.run()
  return 0


if __name__ == "__main__":
  sys.exit(main())
