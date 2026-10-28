"""Creates the camera object `d455` in `world` and clones `world` into `init_world` (phase_4.md decision 7).

Runs at container start-up right after load_franka. The pose comes from `init_world`'s `d455` (the last
`CalibrationService.Save`) if there is one, else from a nominal seed (Isaac's camera pose moved 3 cm
and turned 3 deg) flagged `calibrated: false`. Never from Isaac's true pose.
  python3 -m robot_model.load_camera [--world=…] [--scene=…]
Exit 0 on success, 1 with the gRPC error otherwise.
"""

import argparse
import math
import sys

from google.protobuf import wrappers_pb2
import grpc
from intrinsic.math.proto import point_pb2
from intrinsic.math.proto import pose_pb2
from intrinsic.math.proto import quaternion_pb2
from intrinsic.math.proto import vector3_pb2
from intrinsic.scene.proto.v1 import entity_pb2
from intrinsic.scene.proto.v1 import scene_object_pb2
from intrinsic.world.proto import object_world_refs_pb2 as refs_pb
from intrinsic.world.proto import object_world_service_pb2 as world_pb
from intrinsic.world.proto import object_world_service_pb2_grpc as world_grpc
from intrinsic.world.proto import object_world_updates_pb2 as updates_pb
import numpy as np
from robot_model import franka_scene_object as fso
import yaml

WORLD, INIT_WORLD = "world", "init_world"
CAMERA = "d455"
SENSOR_FRAME = "sensor"  # CalibrationService solves the pose of the frame named "sensor"
CALIBRATED_KEY = "calibrated"
RPC_TIMEOUT_S = 30
# Rotations inside rsd455.usd as (w, x, y, z): RSD455 under the D455 root, Camera_Pseudo_Depth under
# RSD455; /tf carries the camera frame in ROS optical convention (USD camera axes turned 180 deg about x).
# Same conversion as tests/e2e/phase2_check.py.
RSD455_LOCAL_WXYZ = (0.0, 0.0, 0.0, 1.0)
CAMERA_LOCAL_WXYZ = (0.5, 0.5, 0.5, 0.5)
SEED_OFFSET_M = (0.03, 0.0, 0.0)  # along world x
SEED_YAW_DEG = 3.0  # about world z
BOX_SIZE_M = (0.124, 0.029, 0.026)  # D455 housing in the optical frame (x right, y down, z forward)


def _rot(axis: int, degrees: float) -> np.ndarray:
  c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
  return np.array([[[1, 0, 0], [0, c, -s], [0, s, c]],
                   [[c, 0, s], [0, 1, 0], [-s, 0, c]],
                   [[c, -s, 0], [s, c, 0], [0, 0, 1]]][axis])


def _matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
  return np.array([
      [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ])


def quaternion_wxyz(m: np.ndarray) -> tuple[float, float, float, float]:
  """(w, x, y, z) of a rotation matrix (Shepperd's method)."""
  trace = np.trace(m)
  if trace > 0:
    s = 2 * math.sqrt(1 + trace)
    return s / 4, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
  i = int(np.argmax(np.diag(m)))
  j, k = (i + 1) % 3, (i + 2) % 3
  s = 2 * math.sqrt(1 + m[i, i] - m[j, j] - m[k, k])
  q = [0.0, 0.0, 0.0]
  q[i], q[j], q[k] = s / 4, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
  return (m[k, j] - m[j, k]) / s, q[0], q[1], q[2]


def seed_pose(scene: dict) -> pose_pb2.Pose:
  """Isaac's nominal optical camera frame from the scene file, moved by SEED_OFFSET_M and SEED_YAW_DEG."""
  ex, ey, ez = scene["camera"]["euler_deg"]
  nominal = (_rot(2, ez) @ _rot(1, ey) @ _rot(0, ex) @ _matrix(*RSD455_LOCAL_WXYZ)
             @ _matrix(*CAMERA_LOCAL_WXYZ) @ _rot(0, 180.0))
  w, x, y, z = quaternion_wxyz(_rot(2, SEED_YAW_DEG) @ nominal)
  px, py, pz = np.add(scene["camera"]["position"], SEED_OFFSET_M)
  return pose_pb2.Pose(position=point_pb2.Point(x=px, y=py, z=pz),
                       orientation=quaternion_pb2.Quaternion(x=x, y=y, z=z, w=w))


def camera_scene_object() -> scene_object_pb2.SceneObject:
  """Entity `d455` (its local name lets CalibrationService find the object by resource handle name)
  with a child frame `sensor` at identity.

  The world reserves the frame name `sensor` for CreateFrame, so it comes in as a scene-object entity.
  The world also rejects an object without geometry: the link carries a visual-only box, which keeps
  the camera out of collision checks.
  """
  link = entity_pb2.Entity(name=CAMERA)
  link.link.SetInParent()
  box = (link.link.geometry_component.named_geometries[fso.VISUAL_SET].named_geometries[CAMERA]
         .geometry.inline_geometry_data.exact_geometry.primitive_set.primitives.add())
  box.shape.box.size.CopyFrom(vector3_pb2.Vector3(x=BOX_SIZE_M[0], y=BOX_SIZE_M[1], z=BOX_SIZE_M[2]))
  sensor = entity_pb2.Entity(name=SENSOR_FRAME, parent_name=CAMERA, parent_t_this=fso.pose_proto(),
                             frame=entity_pb2.Frame())
  return scene_object_pb2.SceneObject(name=CAMERA, entities=[link, sensor])


def _by_name(name: str) -> refs_pb.ObjectReference:
  return refs_pb.ObjectReference(by_name=refs_pb.ObjectReferenceByName(object_name=name))


def _root() -> refs_pb.ObjectReferenceWithEntityFilter:
  return refs_pb.ObjectReferenceWithEntityFilter(
      reference=refs_pb.ObjectReference(id="root"),
      entity_filter=refs_pb.ObjectEntityFilter(include_final_entity=True))


def saved_camera(world) -> tuple[pose_pb2.Pose, bool] | None:
  """(pose, calibrated) of `init_world`'s `d455`, or None if it has none."""
  names = {o.name for o in world.ListObjects(world_pb.ListObjectsRequest(world_id=INIT_WORLD),
                                             timeout=RPC_TIMEOUT_S).objects}
  if CAMERA not in names:
    return None
  obj = world.GetObject(world_pb.GetObjectRequest(world_id=INIT_WORLD, object=_by_name(CAMERA),
                                                  view=updates_pb.FULL), timeout=RPC_TIMEOUT_S)
  pose = world.GetTransform(world_pb.GetTransformRequest(
      world_id=INIT_WORLD, node_a=refs_pb.TransformNodeReference(id="root"),
      node_b=refs_pb.TransformNodeReference(by_name=refs_pb.TransformNodeReferenceByName(
          object=refs_pb.ObjectReferenceByName(object_name=CAMERA)))), timeout=RPC_TIMEOUT_S).a_t_b
  calibrated = wrappers_pb2.BoolValue()
  if CALIBRATED_KEY in obj.object_component.user_data:
    obj.object_component.user_data[CALIBRATED_KEY].Unpack(calibrated)
  return pose, calibrated.value


def load(world, scene: dict) -> None:
  saved = saved_camera(world)
  pose, calibrated = saved if saved is not None else (seed_pose(scene), False)
  source = "init_world" if saved is not None else "seed"
  names = {o.name for o in world.ListObjects(world_pb.ListObjectsRequest(world_id=WORLD),
                                             timeout=RPC_TIMEOUT_S).objects}
  if CAMERA in names:
    world.DeleteObject(updates_pb.DeleteObjectRequest(world_id=WORLD, object=_by_name(CAMERA)),
                       timeout=RPC_TIMEOUT_S)
    print(f"[camera_loader] deleted existing {CAMERA} in {WORLD}", flush=True)
  request = updates_pb.CreateObjectRequest(
      world_id=WORLD, name=CAMERA, name_is_global_alias=True, parent_object=_root(),
      parent_object_t_created_object=pose,
      create_from_scene_object=updates_pb.ObjectSpecFromSceneObject(scene_object=camera_scene_object()))
  request.user_data[CALIBRATED_KEY].Pack(wrappers_pb2.BoolValue(value=calibrated))
  world.CreateObject(request, timeout=RPC_TIMEOUT_S)
  p, q = pose.position, pose.orientation
  print(f"[camera_loader] created {CAMERA} with frame {SENSOR_FRAME} in {WORLD} source={source} "
        f"calibrated={calibrated} position=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
        f"quaternion_xyzw=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})", flush=True)
  # Same object IDs in both worlds: CalibrationService.Save sends `world`'s robot and flange IDs.
  world.CloneWorld(world_pb.CloneWorldRequest(world_id=WORLD, cloned_world_id=INIT_WORLD, allow_overwrite=True),
                   timeout=RPC_TIMEOUT_S)
  print(f"[camera_loader] cloned {WORLD} into {INIT_WORLD}", flush=True)


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--world", default="localhost:18000")
  parser.add_argument("--scene", default="/opt/app/config/default.yaml")
  args = parser.parse_args()
  with open(args.scene) as f:
    scene = yaml.safe_load(f)
  with grpc.insecure_channel(args.world) as channel:
    try:
      load(world_grpc.ObjectWorldServiceStub(channel), scene)
    except grpc.RpcError as error:
      print(f"[camera_loader] {error.code().name}: {error.details()}", file=sys.stderr)
      return 1
  return 0


if __name__ == "__main__":
  sys.exit(main())
