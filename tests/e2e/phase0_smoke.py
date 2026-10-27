"""Phase 0 smoke test: every process in the intrinsic-core:dev container answers.

Run against a container started by container/run.sh:
  .venv/bin/python tests/e2e/phase0_smoke.py
"""

from pathlib import Path
import socket
import subprocess
import sys
import time

from google.protobuf import empty_pb2
import grpc

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gen"))
from intrinsic.icon.proto.v1 import service_pb2 as icon_pb  # noqa: E402
from intrinsic.icon.proto.v1 import service_pb2_grpc as icon_grpc  # noqa: E402
from intrinsic.motion_planning.proto.v1 import motion_planner_service_pb2_grpc as planner_grpc  # noqa: E402
from intrinsic.resources.proto import resource_registry_pb2 as registry_pb  # noqa: E402
from intrinsic.resources.proto import resource_registry_pb2_grpc as registry_grpc  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2 as world_pb  # noqa: E402
from intrinsic.world.proto import object_world_service_pb2_grpc as world_grpc  # noqa: E402

HOST = "127.0.0.1"
DEADLINE_S = 5
ICON_PORT = 18003
# ICON opens its port after main-loop creation, bounded by the 120 s HWM wait.
ICON_WAIT_S = 180
# Process names as `ss` prints them (truncated to 15 characters).
ZENOH_CLIENTS = ("world_binary", "motion_planner_", "generic_icon_ma")


def _tcp_connect(port: int, timeout: float = DEADLINE_S) -> None:
  socket.create_connection((HOST, port), timeout=timeout).close()


def _wait_channel_ready(port: int) -> None:
  with grpc.insecure_channel(f"{HOST}:{port}") as channel:
    grpc.channel_ready_future(channel).result(timeout=DEADLINE_S)


def _call(port: int, stub_class, method: str, request):
  with grpc.insecure_channel(f"{HOST}:{port}") as channel:
    return getattr(stub_class(channel), method)(request, timeout=DEADLINE_S)


def _docker(command: str) -> subprocess.CompletedProcess:
  return subprocess.run(
      ["sg", "docker", "-c", command],
      capture_output=True,
      text=True,
      timeout=DEADLINE_S,
  )


def check_zenohd() -> None:
  _tcp_connect(7447)


def check_zenoh_sessions() -> None:
  # Inside the container: the processes run as root there, so a host-side ss cannot name them.
  result = _docker(
      "docker exec intrinsic ss -Htnp state established '( dport = :7447 )'"
  )
  if result.returncode != 0:
    raise RuntimeError(f"ss exited {result.returncode}: {result.stderr}")
  out = result.stdout
  missing = [name for name in ZENOH_CLIENTS if f'"{name}",' not in out]
  if missing:
    raise RuntimeError(f"no established session from {missing}; ss: {out!r}")


def check_localcas() -> None:
  _wait_channel_ready(18020)


def check_stubs() -> None:
  _call(
      18010,
      registry_grpc.ResourceRegistryStub,
      "ListResourceInstances",
      registry_pb.ListResourceInstanceRequest(),
  )


def check_geometry() -> None:
  _wait_channel_ready(18001)


def check_world() -> None:
  response = _call(
      18000,
      world_grpc.ObjectWorldServiceStub,
      "ListObjects",
      world_pb.ListObjectsRequest(world_id="world"),
  )
  if not any(obj.name == "root" for obj in response.objects):
    raise RuntimeError(
        f"no object named root in {[obj.name for obj in response.objects]}"
    )


def check_planner() -> None:
  _call(
      18002,
      planner_grpc.MotionPlannerServiceStub,
      "ClearCache",
      empty_pb2.Empty(),
  )


def check_hwm() -> None:
  socket_test = _docker(
      "docker exec intrinsic test -S /tmp/intrinsic_icon/robot_hwm.sock"
  )
  if socket_test.returncode != 0:
    raise RuntimeError(
        f"socket test exited {socket_test.returncode}: {socket_test.stderr}"
    )
  logs = _docker("docker logs intrinsic")
  if "Hardware Module Runtime started." not in logs.stdout + logs.stderr:
    raise RuntimeError("logs lack 'Hardware Module Runtime started.'")


def check_icon_status() -> None:
  _call(
      ICON_PORT, icon_grpc.IconApiStub, "GetStatus", icon_pb.GetStatusRequest()
  )


def check_icon_parts() -> None:
  _call(
      ICON_PORT, icon_grpc.IconApiStub, "ListParts", icon_pb.ListPartsRequest()
  )


CHECKS = (
    ("zenohd", check_zenohd),
    ("zenoh_sessions", check_zenoh_sessions),
    ("localcas", check_localcas),
    ("stubs", check_stubs),
    ("geometry", check_geometry),
    ("world", check_world),
    ("planner", check_planner),
    ("hwm", check_hwm),
    ("icon_status", check_icon_status),
    ("icon_parts", check_icon_parts),
)


def _describe(error: Exception) -> str:
  if isinstance(error, grpc.RpcError):
    return f"{error.code().name}: {error.details()}"
  return f"{type(error).__name__}: {error}"


def _wait_for_icon_port() -> None:
  deadline = time.monotonic() + ICON_WAIT_S
  while time.monotonic() < deadline:
    try:
      _tcp_connect(ICON_PORT, timeout=1)
      return
    except OSError:
      time.sleep(1)


def main() -> int:
  start = time.monotonic()
  _wait_for_icon_port()
  failures = 0
  for name, check in CHECKS:
    try:
      check()
      print(f"PASS {name}", flush=True)
    except Exception as error:  # pylint: disable=broad-except
      failures += 1
      print(f"FAIL {name}: {_describe(error)}", flush=True)
  print(f"elapsed {time.monotonic() - start:.1f}")
  return 0 if failures == 0 else 1


if __name__ == "__main__":
  sys.exit(main())
