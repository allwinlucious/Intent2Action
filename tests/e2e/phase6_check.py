"""Phase 6 check: the MCP server (mcp_server/server.py) on the running robot.

Syncs the repository to Windows, starts sim/run_scene.py and the container like phase3_cuj_check.py, then
talks to the MCP server over stdio with the MCP Python SDK and prints PASS/FAIL per check (phase_6.md):
  .venv/bin/python tests/e2e/phase6_check.py
Stops the container and Isaac at the end.
"""

import base64
import json
import re
import sys
import time

import anyio
import grpc
from mcp import ClientSession
from mcp import StdioServerParameters
from mcp.client.stdio import stdio_client

import harness
import phase3_hwm_check as hwm  # sets sys.path for gen/
import phase1_check as p1
import phase3_cuj_check as cuj
from robot_service import robot_service_pb2 as api  # noqa: E402

ISAAC_LOG = harness.LOG_DIR / "phase6_isaac.log"
CONTAINER_LOG = harness.LOG_DIR / "phase6_container.log"
MCP_LOG = harness.ROOT / "logs/mcp_server.log"
SERVER = StdioServerParameters(
    command=str(harness.ROOT / ".venv/bin/python"),
    args=[str(harness.ROOT / "mcp_server/server.py")],
    cwd=str(harness.ROOT),
)
EXCLUDED = {"get_point_cloud"}
OFFSET = (0.10, 0.10, -0.15)  # m, from the home TCP pose (CUJ 2's target)
UNREACHABLE = {"x": 2.0, "y": 0.0, "z": 0.5, "qw": 1.0}
OPERATION_NAME = re.compile(r"operations/[\w-]+")


def _snake(name: str) -> str:
  return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def _json(result) -> dict:
  if result.is_error:
    raise RuntimeError(f"tool error: {result.content[0].text}")
  return json.loads(result.content[-1].text)


def _done(reply: dict, what: str) -> None:
  if not reply["done"] or reply["state"] != "DONE" or "error" in reply:
    raise RuntimeError(f"{what}: {reply}")


async def check_tools(session: ClientSession) -> str:
  tools = (await session.list_tools()).tools
  want = {_snake(m.name) for m in api.DESCRIPTOR.services_by_name["RobotService"].methods} - EXCLUDED
  names = [t.name for t in tools]
  if sorted(names) != sorted(want):
    raise RuntimeError(f"tools {sorted(names)}, want {sorted(want)}")
  bad = [t.name for t in tools if not t.description or t.input_schema.get("type") != "object"]
  if bad:
    raise RuntimeError(f"no description or object input schema: {bad}")
  return f"{len(tools)} tools, each with a description and an input schema"


async def check_calls(session: ClientSession) -> str:
  calls_before = MCP_LOG.read_text().count(" tool=") if MCP_LOG.exists() else 0
  details, calls = [], 0

  async def call(name: str, arguments: dict | None = None):
    nonlocal calls
    calls += 1
    return await session.call_tool(name, arguments or {})

  joints = _json(await call("get_state"))["joints"]["positions"]
  if len(joints) != 7:
    raise RuntimeError(f"get_state joints {joints}")
  details.append("get_state 7 joints")

  _done(_json(await call("home")), "home")
  down = _json(await call("get_state"))["tool_pose"]
  target = {**down, "x": down["x"] + OFFSET[0], "y": down["y"] + OFFSET[1], "z": down["z"] + OFFSET[2]}
  moved = _json(await call("move_to_pose", {"pose": target, "wait": True}))
  _done(moved, "move_to_pose")
  details.append(f"move_to_pose done {moved['response']['duration_s']:.2f} s")

  result = await call("get_image", {"kind": "RGB"})
  image = result.content[0]
  if result.is_error or image.type != "image" or image.mime_type != "image/png":
    raise RuntimeError(f"get_image: {result.content}")
  decoded = cuj._png(base64.b64decode(image.data))
  if decoded[:2] != (cuj.WIDTH, cuj.HEIGHT):
    raise RuntimeError(f"get_image PNG {decoded}")
  details.append(f"get_image PNG {decoded[0]}x{decoded[1]}")

  try:
    cuj._call(cuj._robot, "MoveToPose", api.MoveToPoseRequest(pose=api.Pose(**UNREACHABLE), wait=True))
  except grpc.RpcError as error:
    want = f"{error.code().name}: {error.details()}"
  else:
    raise RuntimeError("unreachable MoveToPose over gRPC returned OK")
  result = await call("move_to_pose", {"pose": UNREACHABLE})
  # The message names the operation, which differs between the two calls.
  if not result.is_error or OPERATION_NAME.sub("operations/*", result.content[0].text) != OPERATION_NAME.sub(
      "operations/*", want):
    raise RuntimeError(f"unreachable move_to_pose: is_error={result.is_error} {result.content[0].text!r}, want {want!r}")
  details.append(f"unreachable -> error {want[:80]!r}")

  started = _json(await call("move_to_pose", {"pose": down, "wait": False}))
  waited = _json(await call("wait_operation", {"name": started["name"]}))
  _done(waited, "wait_operation")
  details.append(f"wait=false {started['name']} done={started['done']} -> wait_operation done")

  logged = MCP_LOG.read_text().count(" tool=") - calls_before
  if logged != calls:
    raise RuntimeError(f"{calls} tool calls, {logged} lines in {MCP_LOG}")
  details.append(f"{calls} calls logged")
  return "; ".join(details)


def _run(check) -> str:
  async def session_run():
    async with stdio_client(SERVER) as (read, write):
      async with ClientSession(read, write) as session:
        await session.initialize()
        return await check(session)
  try:
    return anyio.run(session_run)
  except BaseExceptionGroup as group:  # the MCP client's task groups wrap the check's own error
    error = group
    while isinstance(error, BaseExceptionGroup):
      error = error.exceptions[0]
    raise error from None


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
    cuj._wait_ready()
    print(f"robot_service ready after {time.monotonic() - start:.1f} s", flush=True)
    results = [hwm._run_check(name, lambda c=check: _run(c))
               for name, check in (("tools", check_tools), ("calls", check_calls))]
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
