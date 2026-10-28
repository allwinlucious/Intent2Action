"""Phase 4 experiments a and b (phase_4.md Step 2), against a running container (after load_camera):
  a. GetObject(resource_handle_name="d455") on `world` finds the decision 7 object (with frame `sensor`).
  b. `franka`'s object ID (and its flange frame ID) is the same in `world` and `init_world`.
  .venv/bin/python tests/e2e/exp4/exp4_ab.py   ->   one JSON line, exit 0 if both hold
"""

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / "gen")]
import grpc  # noqa: E402
from intrinsic.world.proto import object_world_refs_pb2 as refs_pb  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2 as world_pb  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2_grpc as world_grpc  # noqa: E402
from intrinsic.world.proto import object_world_updates_pb2 as updates_pb  # noqa: E402

WORLD_ADDRESS = "localhost:18000"
TIMEOUT_S = 30


def main() -> int:
  world = world_grpc.ObjectWorldServiceStub(grpc.insecure_channel(WORLD_ADDRESS))
  result = {}
  try:
    camera = world.GetObject(world_pb.GetObjectRequest(world_id="world", resource_handle_name="d455",
                                                       view=updates_pb.FULL), timeout=TIMEOUT_S)
    result["a"] = {"found": camera.name, "frames": sorted(f.name for f in camera.frames)}
  except grpc.RpcError as error:
    result["a"] = {"error": f"{error.code().name}: {error.details()}"}
  ids = {}
  for world_id in ("world", "init_world"):
    franka = world.GetObject(world_pb.GetObjectRequest(
        world_id=world_id, object=refs_pb.ObjectReference(by_name=refs_pb.ObjectReferenceByName(object_name="franka")),
        view=updates_pb.FULL), timeout=TIMEOUT_S)
    ids[world_id] = {"franka": franka.id, "flange": next(f.id for f in franka.frames if f.name == "flange")}
  result["b"] = ids
  ok_a = result["a"].get("found") == "d455" and "sensor" in result["a"].get("frames", [])
  ok_b = ids["world"] == ids["init_world"]
  result["pass"] = {"a": ok_a, "b": ok_b}
  print(json.dumps(result))
  return 0 if ok_a and ok_b else 1


if __name__ == "__main__":
  sys.exit(main())
