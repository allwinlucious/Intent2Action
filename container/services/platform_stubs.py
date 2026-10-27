"""Stand-ins for the Intrinsic platform services that k3s normally provides.

- ResourceRegistry: the world service refuses to start without one. It only calls ListResourceInstances and
  GetResourceInstance, so an in-memory registry is enough.
- InstalledAssetsReader: the motion planner only asks it for its own asset version at startup.
"""

import argparse
import sys
from concurrent import futures
from pathlib import Path

import grpc

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gen"))
from intrinsic.assets.proto import installed_assets_pb2 as assets_pb  # noqa: E402
from intrinsic.assets.proto import installed_assets_pb2_grpc as assets_grpc  # noqa: E402
from intrinsic.resources.proto import resource_registry_pb2 as registry_pb  # noqa: E402
from intrinsic.resources.proto import resource_registry_pb2_grpc as registry_grpc  # noqa: E402

LOCAL_VERSION = "0.0.0-local"


class ResourceRegistry(registry_grpc.ResourceRegistryServicer):
    # shortcut: no resource instances; ICON reads its config file and does not need one.
    # Add instances if a service starts requiring them.
    instances: dict[str, registry_pb.ResourceInstance] = {}

    def ListResourceInstances(self, request, context):
        return registry_pb.ListResourceInstanceResponse(instances=list(self.instances.values()))

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=18010)
    args = parser.parse_args()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    registry_grpc.add_ResourceRegistryServicer_to_server(ResourceRegistry(), server)
    assets_grpc.add_InstalledAssetsReaderServicer_to_server(InstalledAssetsReader(), server)
    server.add_insecure_port(f"[::]:{args.port}")
    server.start()
    print(f"platform stubs listening on :{args.port}", flush=True)
    server.wait_for_termination()


if __name__ == "__main__":
    main()
