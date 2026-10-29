"""MCP server: one tool per RobotService RPC, generated from robot_service.proto (phase 6).

Started by the MCP client over stdio (see README.md); stdout is the MCP channel. Tool names, descriptions
and input schemas come from gen/robot_service.desc (container/gen_protos.sh); calls go to RobotService on
127.0.0.1:18100. One line per tool call goes to logs/mcp_server.log.
"""

import base64
import json
import logging
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "gen"))

import anyio  # noqa: E402
from google.protobuf import descriptor  # noqa: E402
from google.protobuf import descriptor_pb2  # noqa: E402
from google.protobuf import descriptor_pool  # noqa: E402
from google.protobuf import json_format  # noqa: E402
from google.protobuf import message_factory  # noqa: E402
import grpc  # noqa: E402
import mcp.server.stdio  # noqa: E402
from mcp.server.lowlevel import Server  # noqa: E402
import mcp.types as types  # noqa: E402
from robot_service import robot_service_pb2_grpc  # noqa: E402

ADDRESS = "127.0.0.1:18100"
DESCRIPTOR_SET = ROOT / "gen/robot_service.desc"
LOG_PATH = ROOT / "logs/mcp_server.log"
SERVICE = "robot.v1.RobotService"
EXCLUDED = {"GetPointCloud"}  # 57 600 points are no use to a language model; stays a gRPC API
CALL_TIMEOUT_S = 70.0  # wait calls block <= 60 s server-side
EXTRA_TIMEOUT_S = 10.0  # beyond WaitOperation's timeout and Jog's duration_s
WAIT_DEFAULT_S = 60.0
LOG_ARGS_CHARS = 200
CODES = {code.value[0]: code.name for code in grpc.StatusCode}
OPERATION = "google.longrunning.Operation"
IMAGE = "robot.v1.GetImageResponse"
SCALARS = {
    descriptor.FieldDescriptor.CPPTYPE_DOUBLE: "number",
    descriptor.FieldDescriptor.CPPTYPE_FLOAT: "number",
    descriptor.FieldDescriptor.CPPTYPE_INT32: "integer",
    descriptor.FieldDescriptor.CPPTYPE_INT64: "integer",
    descriptor.FieldDescriptor.CPPTYPE_UINT32: "integer",
    descriptor.FieldDescriptor.CPPTYPE_UINT64: "integer",
    descriptor.FieldDescriptor.CPPTYPE_BOOL: "boolean",
    descriptor.FieldDescriptor.CPPTYPE_STRING: "string",
}
INSTRUCTIONS = """\
Controls a simulated Franka Panda arm with a parallel gripper (opening up to 0.075 m) through Intrinsic's \
robot stack. Units: metres, radians, seconds, newtons, kilograms. Poses are a position (x, y, z) and a unit \
quaternion (qx, qy, qz, qw). Frames: the robot base frame is the world frame (z up); every pose is in it. \
The TCP (tool centre point, the default `tool`) is between the fingertips; FLANGE is the robot flange. \
Object poses from list_objects / detect_objects are the objects' bottom centres. Objects are identified only \
by an `id` (e.g. obj_42) with category, color, size and pose; ids change after a scene reset.
Long-running calls (motions, pick, place, grasp, release, calibrate_camera, detect_objects) take `wait`: \
true (the default) blocks up to 60 s and returns the operation, false returns at once; then poll with \
get_operation or block with wait_operation. A returned operation has `done`, `state` and, on failure, `error`.
Typical pick and place: list_objects, choose the ids whose category, color, size and pose match the \
request, then pick(object_id) -> place(object_id, target_id) (a target with category box: put inside; \
any other object: on top) -> list_objects to check the result. pick and place make one attempt; if one \
fails, read its error and decide what to do (e.g. after "closed on nothing", call release, then pick \
again). Objects inside a box cannot be picked again in this version. The low-level calls (plan_grasp, \
move_to_pose, grasp, release, attach_object, detach_object) remain for special cases.
After a fault (get_state shows faulted), call clear_faults before moving again."""

log = logging.getLogger("mcp_server")


def _snake(name: str) -> str:
  return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def _comments(files: descriptor_pb2.FileDescriptorSet) -> dict[str, str]:
  """Full name of every message, field, enum value (`<enum full name>.<value>`) and method -> its leading and
  trailing comments, whitespace collapsed."""
  out = {}
  for file in files.file:
    locations = {tuple(l.path): l for l in file.source_code_info.location}

    def text(path, locations=locations) -> str:
      location = locations.get(tuple(path))
      return " ".join(f"{location.leading_comments} {location.trailing_comments}".split()) if location else ""

    def enum(proto, path, prefix):
      for i, value in enumerate(proto.value):
        out[f"{prefix}.{proto.name}.{value.name}"] = text(path + [2, i])

    def message(proto, path, prefix):
      name = f"{prefix}.{proto.name}"
      out[name] = text(path)
      for i, field in enumerate(proto.field):
        out[f"{name}.{field.name}"] = text(path + [2, i])
      for i, nested in enumerate(proto.nested_type):
        message(nested, path + [3, i], name)
      for i, nested in enumerate(proto.enum_type):
        enum(nested, path + [4, i], name)

    for i, proto in enumerate(file.message_type):
      message(proto, [4, i], file.package)
    for i, proto in enumerate(file.enum_type):
      enum(proto, [5, i], file.package)
    for i, service in enumerate(file.service):
      for j, method in enumerate(service.method):
        out[f"{file.package}.{service.name}.{method.name}"] = text([6, i, 2, j])
  return out


def _enum_values(enum: descriptor.EnumDescriptor) -> list[str]:
  return [v.name for v in enum.values if not v.name.endswith("_UNSPECIFIED")]


class Tools:
  """The tools of RobotService, built from a descriptor set with source info."""

  def __init__(self, descriptor_set: Path):
    files = descriptor_pb2.FileDescriptorSet.FromString(descriptor_set.read_bytes())
    pool = descriptor_pool.DescriptorPool()
    for file in files.file:
      pool.Add(file)
    self._comments = _comments(files)
    # name -> (RPC name, request full name, MCP tool)
    self.by_name: dict[str, tuple[str, str, types.Tool]] = {}
    for method in pool.FindServiceByName(SERVICE).methods:
      if method.name in EXCLUDED:
        continue
      tool = types.Tool(
          name=_snake(method.name),
          description=self._description(method),
          input_schema=self._object(method.input_type, top=True),
      )
      self.by_name[tool.name] = (method.name, method.input_type.full_name, tool)

  def _type(self, field: descriptor.FieldDescriptor) -> str:
    if field.message_type:
      name = field.message_type.name
    elif field.enum_type:
      name = field.enum_type.name
    else:
      name = descriptor_pb2.FieldDescriptorProto.Type.Name(field.type).removeprefix("TYPE_").lower()
    return f"repeated {name}" if field.is_repeated else name

  def _field_text(self, field: descriptor.FieldDescriptor) -> str:
    parts = [self._comments.get(field.full_name, "")]
    if oneof := field.containing_oneof:
      parts.append(f"oneof `{oneof.name}`: set only one of {', '.join(f.name for f in oneof.fields)}")
    if field.name == "wait":
      parts.append("default true (blocks up to 60 s); false returns at once, then use wait_operation")
    if field.enum_type:
      values = (
          f"{name} ({c})" if (c := self._comments.get(f"{field.enum_type.full_name}.{name}")) else name
          for name in _enum_values(field.enum_type)
      )
      parts.append("one of " + ", ".join(values))
    return "; ".join(p for p in parts if p)

  def _description(self, method: descriptor.MethodDescriptor) -> str:
    lines = [self._comments.get(method.full_name, "")]
    for field in method.input_type.fields:
      lines.append(f"`{field.name}` ({self._type(field)}): {self._field_text(field)}".rstrip())
    return "\n".join(lines)

  def _field_schema(self, field: descriptor.FieldDescriptor) -> dict:
    if field.message_type:
      schema = self._object(field.message_type)
    elif field.enum_type:
      schema = {"type": "string", "enum": _enum_values(field.enum_type)}
    else:
      schema = {"type": SCALARS[field.cpp_type]}
    if field.is_repeated:
      schema = {"type": "array", "items": schema}
    if field.name == "wait":
      schema["default"] = True
    if text := self._field_text(field):
      schema["description"] = " ".join(filter(None, (text, schema.get("description"))))
    return schema

  def _object(self, message: descriptor.Descriptor, top: bool = False) -> dict:
    if message.full_name == "google.protobuf.Duration":
      return {"type": "string", "description": 'duration, e.g. "30s"'}
    schema = {"type": "object", "properties": {f.name: self._field_schema(f) for f in message.fields}}
    if not top and (text := self._comments.get(message.full_name)):
      schema["description"] = text
    return schema


def _message_class(full_name: str):
  return message_factory.GetMessageClass(descriptor_pool.Default().FindMessageTypeByName(full_name))


def _unpack(any_message) -> dict:
  message = _message_class(any_message.TypeName())()
  any_message.Unpack(message)
  return _to_dict(message)


def _to_dict(message) -> dict:
  return json_format.MessageToDict(
      message, preserving_proto_field_name=True, always_print_fields_with_no_presence=True
  )


def _text(data, is_error: bool = False) -> types.CallToolResult:
  text = data if isinstance(data, str) else json.dumps(data)
  return types.CallToolResult(content=[types.TextContent(type="text", text=text)], is_error=is_error)


def _operation(operation) -> dict:
  info = _unpack(operation.metadata) if operation.HasField("metadata") else {}
  result = {
      "name": operation.name,
      "done": operation.done,
      "state": info.get("state", ""),
      "detail": info.get("detail", ""),
      "planned_duration_s": info.get("planned_duration_s", 0.0),
  }
  if operation.HasField("error"):
    result["error"] = {"code": CODES.get(operation.error.code, str(operation.error.code)),
                       "message": operation.error.message}
  if operation.HasField("response"):
    result["response"] = _unpack(operation.response)
  return result


def _image(response) -> types.CallToolResult:
  kind = response.DESCRIPTOR.fields_by_name["kind"].enum_type.values_by_number[response.kind].name
  info = {"kind": kind, "width": response.width, "height": response.height, "sim_time": response.sim_time}
  if kind == "DEPTH":
    info["note"] = "16-bit PNG, depth in mm"
  return types.CallToolResult(content=[
      types.ImageContent(type="image", data=base64.b64encode(response.png).decode(), mime_type="image/png"),
      types.TextContent(type="text", text=json.dumps(info)),
  ])


def result_of(response) -> types.CallToolResult:
  """The MCP result of a successful RPC response."""
  name = response.DESCRIPTOR.full_name
  if name == OPERATION:
    return _text(_operation(response))
  if name == IMAGE:
    return _image(response)
  return _text(_to_dict(response))


def _timeout(rpc: str, request) -> float:
  if rpc == "WaitOperation":
    return (request.timeout.ToTimedelta().total_seconds() if request.HasField("timeout") else WAIT_DEFAULT_S) \
        + EXTRA_TIMEOUT_S
  if rpc == "Jog":
    return request.duration_s + EXTRA_TIMEOUT_S
  return CALL_TIMEOUT_S


def invoke(tools: Tools, stub, name: str, arguments: dict | None) -> tuple[types.CallToolResult, str]:
  """Runs tool `name` on `stub` (blocking); returns the MCP result and a status for the log."""
  if name not in tools.by_name:
    return _text(f"unknown tool: {name}", is_error=True), "UNKNOWN_TOOL"
  rpc, request_name, _ = tools.by_name[name]
  arguments = dict(arguments or {})
  request_class = _message_class(request_name)
  if "wait" in request_class.DESCRIPTOR.fields_by_name:
    arguments.setdefault("wait", True)
  try:
    request = json_format.ParseDict(arguments, request_class(), ignore_unknown_fields=False)
  except json_format.ParseError as error:
    return _text(f"invalid arguments: {error}", is_error=True), "INVALID_ARGUMENTS"
  try:
    response = getattr(stub, rpc)(request, timeout=_timeout(rpc, request))
  except grpc.RpcError as error:
    code, details = error.code(), error.details() or ""
    # gRPC's own connection errors (robot_service also sends UNAVAILABLE, with its own messages).
    if code == grpc.StatusCode.UNAVAILABLE and "failed to connect" in details.lower():
      details = f"robot service not reachable at {ADDRESS}"
    return _text(f"{code.name}: {details}", is_error=True), code.name
  return result_of(response), "OK"


def main() -> None:
  LOG_PATH.parent.mkdir(exist_ok=True)
  handler = logging.FileHandler(LOG_PATH)
  handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s mcp_server %(message)s"))
  log.addHandler(handler)
  log.setLevel(logging.INFO)
  log.propagate = False
  tools = Tools(DESCRIPTOR_SET)
  stub = robot_service_pb2_grpc.RobotServiceStub(grpc.insecure_channel(ADDRESS))

  async def list_tools(ctx, params) -> types.ListToolsResult:
    return types.ListToolsResult(tools=[tool for _, _, tool in tools.by_name.values()])

  async def call_tool(ctx, params: types.CallToolRequestParams) -> types.CallToolResult:
    start = time.monotonic()
    result, status = await anyio.to_thread.run_sync(invoke, tools, stub, params.name, params.arguments)
    log.info("tool=%s args=%s duration=%.3fs status=%s", params.name,
             json.dumps(params.arguments or {})[:LOG_ARGS_CHARS], time.monotonic() - start, status)
    return result

  server = Server(name="robot", version="1.0", instructions=INSTRUCTIONS,
                  on_list_tools=list_tools, on_call_tool=call_tool)

  async def run() -> None:
    async with mcp.server.stdio.stdio_server() as (read, write):
      await server.run(read, write, server.create_initialization_options())

  log.info("start: %d tools, robot service %s", len(tools.by_name), ADDRESS)
  anyio.run(run)


if __name__ == "__main__":
  main()
