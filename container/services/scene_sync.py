"""Mirrors Isaac's objects into the world service and serves SimLink (phase_3.md decision 15, phase_ids.md).

A Fast DDS ROS 2 node in the image, started by container/entrypoint.sh:
- /tf frames with parent `world` (except the camera and the hand) are objects (Isaac prims). Catalog
  objects are created in `sim_world` and, with belief_source GROUND_TRUTH, in `world`; their poses follow
  /tf; a prim absent from /tf for 2 s is deleted from both worlds.
- Objects are named by a random ID obj_<k> in the worlds (user_data "prim" holds the prim name);
  calib_board keeps its name. Internal structures are keyed by prim. IDs are adopted from persisted worlds
  at start-up and redrawn on every SimLink.Reset.
- SimLink on 127.0.0.1:18101 (robot_service/simlink.proto): every ROS 2 service call to Isaac; object
  fields are IDs.
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

from container.services.scene_ids import ID_PATTERN
from container.services.scene_ids import IdAllocator
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
CALIB_BOARD = "calib_board"  # internal: keeps its name, never given an ID
GROUND_TRUTH = "ground_truth"
SPAWN_PREFIX = "spawn_"  # prims created by Spawn: spawn_<n>
VISUAL_SET, COLLISION_SET = "Intrinsic_Visual", "Intrinsic_Collision"
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


def _user_string(o, key: str) -> str:
  """A StringValue in a world object's user data ("" if absent)."""
  if key not in o.object_component.user_data:
    return ""
  value = wrappers_pb2.StringValue()
  o.object_component.user_data[key].Unpack(value)
  return value.value


def _api_pose(pose: Pose) -> robot_service_pb2.Pose:
  return robot_service_pb2.Pose(**dict(zip(("x", "y", "z", "qx", "qy", "qz", "qw"), pose)))


def unknown_object(object_id: str) -> str:
  return f"no object with id {object_id}; list_objects shows the current ids"


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
  """Keeps sim_world (and world with GROUND_TRUTH) equal to the objects on /tf.

  Every structure is keyed by Isaac prim name (= /tf frame); `ids` maps prims to the world object names.
  """

  def __init__(self, ros: Ros, world, geometry, catalogs: dict[str, str], mirror_world: bool):
    self.ros = ros
    self.world = world
    self.geometry = geometry
    self.default_catalogs = dict(catalogs)  # the scene file's prims; Reset returns to these
    self.catalogs = catalogs  # prim -> catalog name; Spawn adds to it
    self.worlds = [SIM_WORLD, WORLD] if mirror_world else [SIM_WORLD]
    self.ids = IdAllocator()
    # Held by the sync thread for each /tf pass and by SimLink calls that change the prim structures.
    self.lock = threading.Lock()
    self.attached: set[str] = set()
    self.known: set[str] = set()  # objects created by this process
    self.last_seen: dict[str, float] = {}
    self.spawned = 0  # last n of spawn_<n>
    self.geometry_refs = {}  # glb path -> GeometryStorageRefs
    self.attributes: dict[str, tuple[str, str]] = {}  # catalog -> (category, color word) from object.yaml
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

  def assign_ids(self) -> None:
    """Addendum A.5: persisted ground-truth objects keep their ID; the other scene prims get a fresh one.

    Ground-truth objects not adopted (e.g. named by prim before IDs existed) are deleted now rather than by
    _delete_stale 5 s after the first /tf, so ListObjects never shows them."""
    adopted, objects = {}, {w: self._full_objects(w) for w in (SIM_WORLD, WORLD)}
    for w in self.worlds:
      for o in objects[w]:
        prim = _user_string(o, "prim")
        if (prim in self.catalogs and prim != CALIB_BOARD and _user_string(o, "source") == GROUND_TRUTH
            and ID_PATTERN.fullmatch(o.name) and o.name not in adopted.values()):
          adopted.setdefault(prim, o.name)
    self.ids.adopt(adopted)
    drawn = {p: self.ids.add(p) for p in self.catalogs if p != CALIB_BOARD and p not in adopted}
    log.info("ids adopted=%s drawn=%s", adopted, drawn)
    for w, found in objects.items():
      for o in found:
        if _user_string(o, "source") == GROUND_TRUTH and o.name not in adopted.values():
          self._delete(w, o.name)
          log.info("object deleted id=%s prim=%s world=%s reason=not_adopted_at_start", o.name,
                   _user_string(o, "prim"), w)

  def name(self, prim: str) -> str:
    """The world object name of a prim: its ID (calib_board keeps its name)."""
    return self.ids.ids.get(prim, prim)

  def run(self) -> None:
    first_tf = None
    stale_removed = False
    while True:
      self.ros.tf_arrived.wait()
      with self.lock:
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
            names = [self._object_names(w) for w in self.worlds]
            self.known = {p for p in self.known if all(self.name(p) in n for n in names)}
          except grpc.RpcError:
            pass  # the next failure is logged

  def _sync(self, poses: dict[str, Pose], now: float) -> None:
    for prim in poses:
      self.last_seen[prim] = now
    new = [p for p in poses if p not in self.known and self._catalog(p)]
    if new:
      existing = {w: self._object_names(w) for w in self.worlds}
      for prim in new:
        self._create(prim, poses[prim], existing)
    for w in self.worlds:
      updates = [
          updates_pb.ObjectWorldUpdate(
              update_transform=updates_pb.UpdateTransformRequest(
                  node_a=refs_pb.TransformNodeReference(id="root"),
                  node_b=refs_pb.TransformNodeReference(
                      by_name=refs_pb.TransformNodeReferenceByName(
                          object=refs_pb.ObjectReferenceByName(object_name=self.name(prim))
                      )
                  ),
                  a_t_b=self._pose_proto(pose),
              )
          )
          for prim, pose in poses.items()
          if prim in self.known and prim not in new and prim not in self.attached
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
    for prim in [p for p in self.known if now - self.last_seen[p] > ABSENT_S]:
      self.known.discard(prim)
      for w in self.worlds:
        self._delete(w, self.name(prim))
      log.info("object deleted id=%s prim=%s worlds=%s reason=absent_from_tf_%.0fs",
               self.name(prim), prim, self.worlds, ABSENT_S)

  def _delete_stale(self, now: float) -> None:
    on_tf = {p for p, t in self.last_seen.items() if now - t <= ABSENT_S}
    for w in (SIM_WORLD, WORLD):
      for o in self._full_objects(w):
        prim = _user_string(o, "prim")
        if _user_string(o, "source") != GROUND_TRUTH or (prim in on_tf and o.name == self.name(prim)):
          continue
        self._delete(w, o.name)
        log.info("object deleted id=%s prim=%s world=%s reason=stale_not_on_tf", o.name, prim, w)

  def reset(self) -> dict[str, str]:
    """Addendum A.4, with the lock held after Isaac's reset: every ground-truth object out of both worlds,
    the prim structures cleared, a new ID map; the next /tf re-creates the objects."""
    deleted = []
    for w in (SIM_WORLD, WORLD):
      for o in self._full_objects(w):
        if _user_string(o, "source") == GROUND_TRUTH:
          self._delete(w, o.name)
          deleted.append(o.name)
    self.known.clear()
    self.last_seen.clear()
    self.attached.clear()
    self.catalogs.clear()
    self.catalogs.update(self.default_catalogs)  # Isaac's reset removed the spawned prims
    with self.ros.lock:
      self.ros.pending.clear()
      for prim in [p for p in self.ros.poses if p not in NOT_OBJECTS]:
        del self.ros.poses[prim]
    ids = self.ids.draw([p for p in self.catalogs if p != CALIB_BOARD])
    log.info("reset: deleted %d ground-truth objects from %s; ids=%s", len(deleted), (SIM_WORLD, WORLD), ids)
    return ids

  def spawn(self, catalog: str) -> tuple[str, str]:
    """A new prim spawn_<n> (skipping prims Isaac already has, e.g. from an earlier run) and its ID."""
    self.spawned += 1
    while f"{SPAWN_PREFIX}{self.spawned}" in self.ros.poses or f"{SPAWN_PREFIX}{self.spawned}" in self.catalogs:
      self.spawned += 1
    prim = f"{SPAWN_PREFIX}{self.spawned}"
    self.catalogs[prim] = catalog
    return prim, self.ids.add(prim)

  def forget(self, prim: str, reason: str) -> None:
    """A prim deleted from Isaac: out of both worlds and every prim structure."""
    name = self.name(prim)
    if prim in self.known:
      for w in self.worlds:
        self._delete(w, name)
    self.known.discard(prim)
    self.attached.discard(prim)
    self.last_seen.pop(prim, None)
    self.catalogs.pop(prim, None)
    with self.ros.lock:
      self.ros.pending.pop(prim, None)
      self.ros.poses.pop(prim, None)
    self.ids.forget(prim)
    log.info("object deleted id=%s prim=%s worlds=%s reason=%s", name, prim, self.worlds, reason)

  def _catalog(self, prim: str) -> str | None:
    catalog = self.catalogs.get(prim)
    if catalog is None and prim not in self.unknown_logged:
      self.unknown_logged.add(prim)
      log.warning("ignoring /tf frame %s: no catalog entry for it", prim)
    return catalog

  def _attributes(self, catalog: str) -> tuple[str, str]:
    if catalog not in self.attributes:
      entry = yaml.safe_load((CATALOG_DIR / catalog / "object.yaml").read_text())
      self.attributes[catalog] = (entry["category"], entry["color_name"])
    return self.attributes[catalog]

  def _create(self, prim: str, pose: Pose, existing: dict[str, set[str]]) -> None:
    catalog = self.catalogs[prim]
    name = self.name(prim)
    category, color = self._attributes(catalog)
    refs = self._geometry(CATALOG_DIR / catalog / "mesh.glb")
    # An open container collides as child objects, one convex slab each (floor + walls): the planner checks an
    # object's collision geometry as one convex hull, which would fill the cavity.
    parts = sorted((CATALOG_DIR / catalog / "collision_parts").glob("part_*.glb"))
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
      geometries[VISUAL_SET].named_geometries[catalog].geometry.geo_ref.CopyFrom(refs)
      if not parts:
        geometries[COLLISION_SET].named_geometries[catalog].geometry.geo_ref.CopyFrom(refs)
      # catalog: PlanGrasp and ListObjects read its dims; category and color: ListObjects.
      for key, value in (("source", GROUND_TRUTH), ("catalog", catalog), ("prim", prim), ("category", category),
                         ("color", color)):
        request.user_data[key].Pack(wrappers_pb2.StringValue(value=value))
      self.world.CreateObject(request, timeout=RPC_TIMEOUT_S)
      for part in parts:
        self._create_part(w, name, part)
      log.info(
          "object created id=%s prim=%s catalog=%s world=%s replaced=%s pose=%s",
          name, prim, catalog, w, name in existing[w], [round(v, 4) for v in pose],
      )
    self.known.add(prim)

  def _create_part(self, world_id: str, parent: str, glb: Path) -> None:
    """One collision slab of container `parent`, as its child object (user data part_of = parent)."""
    request = updates_pb.CreateObjectRequest(
        world_id=world_id,
        name=f"{parent}_{glb.stem}",
        parent_object=refs_pb.ObjectReferenceWithEntityFilter(
            reference=_by_name(parent),
            entity_filter=refs_pb.ObjectEntityFilter(include_final_entity=True),
        ),
        parent_object_t_created_object=self._pose_proto((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)),
        view=updates_pb.BASIC,
    )
    geometries = request.create_single_entity_object.geometry_component.named_geometries
    geometries[COLLISION_SET].named_geometries[glb.stem].geometry.geo_ref.CopyFrom(self._geometry(glb))
    request.user_data["part_of"].Pack(wrappers_pb2.StringValue(value=parent))
    self.world.CreateObject(request, timeout=RPC_TIMEOUT_S)

  def _geometry(self, path: Path):
    if path not in self.geometry_refs:
      response = self.geometry.CreateGeometry(
          geometry_service_pb2.CreateGeometryRequest(
              data=geometry_service_types_pb2.GeometryData(gltf_bytes=path.read_bytes())),
          timeout=RPC_TIMEOUT_S,
      )
      self.geometry_refs[path] = response.geometry_storage_refs
    return self.geometry_refs[path]

  def _full_objects(self, world_id: str) -> list:
    return self.world.ListObjects(
        world_pb.ListObjectsRequest(world_id=world_id, view=updates_pb.FULL), timeout=RPC_TIMEOUT_S
    ).objects

  def _object_names(self, world_id: str) -> set[str]:
    objects = self.world.ListObjects(world_pb.ListObjectsRequest(world_id=world_id), timeout=RPC_TIMEOUT_S).objects
    return {o.name for o in objects} - {"root"}

  def _delete(self, world_id: str, name: str, reparent: bool = False) -> None:
    self.world.DeleteObject(
        # force (unless reparenting): a container's collision parts are its children and go with it.
        updates_pb.DeleteObjectRequest(world_id=world_id, object=_by_name(name), reparent_child_objects=reparent,
                                       force=not reparent),
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

  def _prim(self, object_id: str, context, also=()) -> str:
    """The prim of an object ID (or a name in `also`, passed through); NOT_FOUND for anything else."""
    if object_id in also:
      return object_id
    prim = self.sync.ids.prim(object_id)
    if prim is None:
      context.abort(grpc.StatusCode.NOT_FOUND, unknown_object(object_id))
    return prim

  @staticmethod
  def _world_failed(context, error: grpc.RpcError) -> None:
    context.abort(error.code(), f"world service: {error.details()}")

  @_logged
  def Spawn(self, request, context):
    if not (CATALOG_DIR / request.catalog / "mesh.glb").is_file():
      context.abort(grpc.StatusCode.NOT_FOUND, f"no catalog entry {request.catalog!r} in {CATALOG_DIR}")
    with self.sync.lock:
      prim, object_id = self.sync.spawn(request.catalog)
    spawn = SpawnEntity.Request(
        name=f"{OBJECTS_PRIM}/{prim}",
        allow_renaming=False,
        uri=f"{ISAAC_CATALOG_DIR}/{request.catalog}/model.usd",
    )
    p, pose = request.pose, spawn.initial_pose.pose
    pose.position.x, pose.position.y, pose.position.z = p.x, p.y, p.z
    o = pose.orientation
    o.x, o.y, o.z, o.w = (p.qx, p.qy, p.qz, p.qw) if any((p.qx, p.qy, p.qz, p.qw)) else (0.0, 0.0, 0.0, 1.0)
    result = self.ros.call(context, "/spawn_entity", spawn, request_renders=True).result
    if result.result != Result.RESULT_OK:
      with self.sync.lock:
        self.sync.forget(prim, "spawn_failed")
      context.abort(
          SPAWN_CODES.get(result.result, grpc.StatusCode.INTERNAL),
          f"Isaac SpawnEntity {prim}: result {result.result}: {result.error_message}",
      )
    log.info("spawned id=%s prim=%s catalog=%s", object_id, prim, request.catalog)
    return simlink_pb2.SpawnResponse(id=object_id)

  @_logged
  def Delete(self, request, context):
    prim = self._prim(request.id, context, also={CALIB_BOARD})
    delete = DeleteEntity.Request(entity=f"{OBJECTS_PRIM}/{prim}")
    result = self.ros.call(context, "/delete_entity", delete, request_renders=True).result
    if result.result != Result.RESULT_OK:
      context.abort(
          SPAWN_CODES.get(result.result, grpc.StatusCode.INTERNAL),
          f"Isaac DeleteEntity {prim}: result {result.result}: {result.error_message}",
      )
    if prim != CALIB_BOARD:  # calib_board stays in the catalogs (AttachBoard re-creates it)
      with self.sync.lock:
        try:
          self.sync.forget(prim, "Delete")
        except grpc.RpcError as error:
          self._world_failed(context, error)
    return empty_pb2.Empty()

  @_logged
  def Reset(self, request, context):
    with self.sync.lock:
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
      try:
        self.sync.reset()
      except grpc.RpcError as error:
        self._world_failed(context, error)
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
    prim = self._prim(request.id, context, also={CALIB_BOARD})
    if request.attached:
      self.sync.attached.add(prim)
    else:
      self.sync.attached.discard(prim)
    return empty_pb2.Empty()

  @_logged
  def GetTruePose(self, request, context):
    frame = self._prim(request.frame, context, also=NOT_OBJECTS | {CALIB_BOARD})
    pose = self.ros.poses.get(frame)
    if pose is None:
      context.abort(grpc.StatusCode.NOT_FOUND, f"no /tf frame {frame!r} with parent {WORLD} ({request.frame})")
    return _api_pose(pose)

  @_logged
  def GetClock(self, request, context):
    if self.ros.clock is None:
      context.abort(grpc.StatusCode.UNAVAILABLE, "no /clock received from Isaac yet")
    return simlink_pb2.ClockResponse(sim_time=self.ros.clock)

  @_logged
  def ListTrue(self, request, context):
    response = robot_service_pb2.TrueObjects()
    for prim, object_id in sorted(dict(self.sync.ids.ids).items(), key=lambda item: item[1]):
      true = response.objects.add(id=object_id, catalog=self.sync.catalogs.get(prim, ""), prim=prim)
      if (pose := self.ros.poses.get(prim)) is not None:
        true.pose.CopyFrom(_api_pose(pose))
    return response

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
  catalogs[CALIB_BOARD] = CALIB_BOARD
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
  sync.assign_ids()

  server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))
  simlink_pb2_grpc.add_SimLinkServicer_to_server(SimLink(ros, sync), server)
  server.add_insecure_port(f"127.0.0.1:{args.port}")
  server.start()
  log.info(
      "started simlink=127.0.0.1:%d belief_source=%s worlds=%s prims=%s ids=%s",
      args.port, belief_source, sync.worlds, sorted(catalogs), sync.ids.ids,
  )
  sync.run()
  return 0


if __name__ == "__main__":
  sys.exit(main())
