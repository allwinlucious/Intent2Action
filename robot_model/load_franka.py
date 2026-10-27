"""Loads the Franka (`franka` + `franka_hand`) into the world service.

Runs at container start-up, before ICON, so ICON's main loop sees the robot.
Delete-then-create, so it also works on a persisted world:
  python3 -m robot_model.load_franka [--world=…] [--geometry=…] [--assets=…]
Exit 0 on success, 1 with the gRPC error otherwise.
"""

import argparse
import ast
from pathlib import Path
import sys
import time

import grpc
from intrinsic.geometry.proto import geometry_service_pb2_grpc
from intrinsic.world.proto import object_world_refs_pb2 as refs_pb
from intrinsic.world.proto import object_world_service_pb2 as world_pb
from intrinsic.world.proto import object_world_service_pb2_grpc as world_grpc
from intrinsic.world.proto import object_world_updates_pb2 as updates_pb

from robot_model import franka_scene_object as fso

WORLD_ID = "world"
CHANNEL_WAIT_S = 60
RPC_TIMEOUT_S = 30


def read_lula_limits(descriptor_path: Path) -> fso.LulaLimits:
  """Reads `acceleration_limits` and `jerk_limits` (one-line lists) from the YAML."""
  values = {}
  for line in descriptor_path.read_text().splitlines():
    key, _, rest = line.partition(":")
    if key in ("acceleration_limits", "jerk_limits"):
      values[key] = tuple(float(v) for v in ast.literal_eval(rest.strip()))
  return fso.LulaLimits(
      acceleration=values["acceleration_limits"], jerk=values["jerk_limits"]
  )


def _by_name(name: str) -> refs_pb.ObjectReference:
  return refs_pb.ObjectReference(
      by_name=refs_pb.ObjectReferenceByName(object_name=name)
  )


def _create_request(
    scene_object, parent: refs_pb.ObjectReferenceWithEntityFilter
) -> updates_pb.CreateObjectRequest:
  return updates_pb.CreateObjectRequest(
      world_id=WORLD_ID,
      name=scene_object.name,
      name_is_global_alias=True,
      parent_object=parent,
      parent_object_t_created_object=fso.pose_proto(),
      create_from_scene_object=updates_pb.ObjectSpecFromSceneObject(
          scene_object=scene_object
      ),
  )


def load(world_channel, geometry_channel, asset_dir: Path) -> None:
  world = world_grpc.ObjectWorldServiceStub(world_channel)
  geometry = geometry_service_pb2_grpc.GeometryServiceStub(geometry_channel)

  refs = fso.upload_meshes(geometry, asset_dir, RPC_TIMEOUT_S)
  urdf = fso.parse_urdf(asset_dir / "lula_franka_gen.urdf")
  arm = fso.build_arm(
      urdf, refs, read_lula_limits(asset_dir / "robot_descriptor.yaml")
  )
  hand = fso.build_hand(urdf, refs)

  existing = {
      obj.name
      for obj in world.ListObjects(
          world_pb.ListObjectsRequest(world_id=WORLD_ID), timeout=RPC_TIMEOUT_S
      ).objects
  }
  for name in (fso.HAND_NAME, fso.ARM_NAME):  # child before parent
    if name in existing:
      world.DeleteObject(
          updates_pb.DeleteObjectRequest(
              world_id=WORLD_ID, object=_by_name(name), force=True
          ),
          timeout=RPC_TIMEOUT_S,
      )
      print(f"[loader] deleted existing {name}", flush=True)

  world.CreateObject(
      _create_request(
          arm,
          refs_pb.ObjectReferenceWithEntityFilter(
              reference=refs_pb.ObjectReference(id="root"),
              entity_filter=refs_pb.ObjectEntityFilter(
                  include_final_entity=True
              ),
          ),
      ),
      timeout=RPC_TIMEOUT_S,
  )
  world.CreateObject(
      _create_request(
          hand,
          refs_pb.ObjectReferenceWithEntityFilter(
              reference=_by_name(fso.ARM_NAME),
              entity_filter=refs_pb.ObjectEntityFilter(
                  entity_names=[fso.FLANGE_FRAME]
              ),
          ),
      ),
      timeout=RPC_TIMEOUT_S,
  )
  # The hand mesh overlaps panda_link7's at the flange; the world only
  # excludes adjacent links inside one object.
  world.ToggleCollisions(
      updates_pb.ToggleCollisionsRequest(
          world_id=WORLD_ID,
          toggle_mode=updates_pb.TOGGLE_MODE_DISABLE,
          object_a=refs_pb.ObjectReferenceWithEntityFilter(
              reference=_by_name(fso.HAND_NAME),
              entity_filter=refs_pb.ObjectEntityFilter(
                  include_all_entities=True
              ),
          ),
          object_b=refs_pb.ObjectReferenceWithEntityFilter(
              reference=_by_name(fso.ARM_NAME),
              entity_filter=refs_pb.ObjectEntityFilter(
                  entity_names=["panda_link7"]
              ),
          ),
      ),
      timeout=RPC_TIMEOUT_S,
  )


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--world", default="localhost:18000")
  parser.add_argument("--geometry", default="localhost:18001")
  parser.add_argument("--assets", default="/opt/app/assets/robots/franka")
  args = parser.parse_args()

  with grpc.insecure_channel(
      args.world
  ) as world_channel, grpc.insecure_channel(args.geometry) as geometry_channel:
    deadline = time.monotonic() + CHANNEL_WAIT_S
    try:
      for channel in (world_channel, geometry_channel):
        grpc.channel_ready_future(channel).result(
            timeout=max(0.0, deadline - time.monotonic())
        )
    except grpc.FutureTimeoutError:
      print(
          f"[loader] world or geometry not ready after {CHANNEL_WAIT_S} s",
          file=sys.stderr,
      )
      return 1
    try:
      load(world_channel, geometry_channel, Path(args.assets))
    except grpc.RpcError as error:
      print(f"[loader] {error.code().name}: {error.details()}", file=sys.stderr)
      return 1
  print(f"[loader] created {fso.ARM_NAME} and {fso.HAND_NAME}", flush=True)
  return 0


if __name__ == "__main__":
  sys.exit(main())
