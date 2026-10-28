"""Stand-ins for the Intrinsic platform services that k3s normally provides.

- ResourceRegistry: the world service refuses to start without one. It only calls ListResourceInstances and
  GetResourceInstance, so an in-memory registry is enough. It holds the ICON robot (`robot`), from which the
  world service subscribes to ICON's joint state and updates `franka` in `world`.
- InstalledAssetsReader: the motion planner only asks it for its own asset version at startup.
- AssetInstances: the calibration service resolves its camera's CameraConfig from the camera asset instance
  (phase_4.md decision 3); only `d455` exists.
- Operations, DataLogger: the calibration service only needs a reachable gRPC server behind those keys
  (phase_4.md decision 2); they are not served, so calls get UNIMPLEMENTED.
"""

import argparse
import logging
import sys
from concurrent import futures
from pathlib import Path

import grpc

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gen"))
from intrinsic.assets.proto import installed_assets_pb2 as assets_pb  # noqa: E402
from intrinsic.assets.proto import installed_assets_pb2_grpc as assets_grpc  # noqa: E402
from intrinsic.assets.proto.v1 import asset_instances_pb2 as instances_pb  # noqa: E402
from intrinsic.assets.proto.v1 import asset_instances_pb2_grpc as instances_grpc  # noqa: E402
from intrinsic.icon.equipment import icon_equipment_pb2  # noqa: E402
from intrinsic.perception.proto.v1 import camera_config_pb2  # noqa: E402
from intrinsic.resources.proto import resource_handle_pb2 as handle_pb  # noqa: E402
from intrinsic.resources.proto import resource_registry_pb2 as registry_pb  # noqa: E402
from intrinsic.resources.proto import resource_registry_pb2_grpc as registry_grpc  # noqa: E402

LOCAL_VERSION = "0.0.0-local"
ICON_ADDRESS = "localhost:18003"
# Capability keys of intrinsic/icon/equipment/equipment_utils.h.
ICON_API = "intrinsic_proto.icon.v1.IconApi"
POSITION_PART = "Icon2PositionPart"
CAMERA = "d455"


def _robot() -> registry_pb.ResourceInstance:
    handle = handle_pb.ResourceHandle(name="robot")
    handle.resource_data[ICON_API].SetInParent()
    handle.resource_data[POSITION_PART].contents.Pack(
        icon_equipment_pb2.Icon2PositionPart(object_names={"arm": "franka"})
    )
    handle.connection_info.grpc.address = ICON_ADDRESS
    return registry_pb.ResourceInstance(name="robot", resource_handle=handle)


class ResourceRegistry(registry_grpc.ResourceRegistryServicer):
    instances: dict[str, registry_pb.ResourceInstance] = {"robot": _robot()}

    def ListResourceInstances(self, request, context):
        wanted = set(request.strict_filter.capability_names)
        return registry_pb.ListResourceInstanceResponse(
            instances=[i for i in self.instances.values() if wanted <= i.resource_handle.resource_data.keys()]
        )

    def GetResourceInstance(self, request, context):
        if request.name not in self.instances:
            context.abort(grpc.StatusCode.NOT_FOUND, f"no resource instance {request.name!r}")
        return self.instances[request.name]


class InstalledAssetsReader(assets_grpc.InstalledAssetsReaderServicer):
    def GetInstalledAsset(self, request, context):
        asset = assets_pb.InstalledAsset()
        asset.metadata.id_version.id.CopyFrom(request.id)
        asset.metadata.id_version.version = LOCAL_VERSION
        return asset


def _camera() -> instances_pb.AssetInstance:
    instance = instances_pb.AssetInstance(name=CAMERA)
    config = camera_config_pb2.CameraConfig()
    config.identifier.ros.device_id = CAMERA
    instance.config.hardware_device.service.service_config.Pack(config)
    return instance


class AssetInstances(instances_grpc.AssetInstancesServicer):
    instances: dict[str, instances_pb.AssetInstance] = {CAMERA: _camera()}

    def GetAssetInstance(self, request, context):
        if request.name not in self.instances:
            logging.warning("GetAssetInstance NOT_FOUND name=%s", request.name)
            context.abort(grpc.StatusCode.NOT_FOUND, f"no asset instance {request.name!r} (only {CAMERA!r} exists)")
        logging.info("GetAssetInstance OK name=%s", request.name)
        return self.instances[request.name]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=18010)
    args = parser.parse_args()
    logging.basicConfig(stream=sys.stdout, level=logging.INFO,
                        format="%(asctime)s %(levelname)s platform_stubs %(message)s")
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    registry_grpc.add_ResourceRegistryServicer_to_server(ResourceRegistry(), server)
    assets_grpc.add_InstalledAssetsReaderServicer_to_server(InstalledAssetsReader(), server)
    instances_grpc.add_AssetInstancesServicer_to_server(AssetInstances(), server)
    server.add_insecure_port(f"[::]:{args.port}")
    server.start()
    print(f"platform stubs listening on :{args.port}", flush=True)
    server.wait_for_termination()


if __name__ == "__main__":
    main()
