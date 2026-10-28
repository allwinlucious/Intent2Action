"""Unit tests of mcp_server/server.py: tools generated from gen/robot_service.desc, results, errors."""

import base64
import json

from google.longrunning import operations_pb2
import grpc
from mcp_server import server
import pytest
from robot_service import robot_service_pb2 as api


@pytest.fixture(scope="module")
def tools() -> server.Tools:
  return server.Tools(server.DESCRIPTOR_SET)


class _RpcError(grpc.RpcError):

  def __init__(self, code: grpc.StatusCode, details: str):
    self._code, self._details = code, details

  def code(self):
    return self._code

  def details(self):
    return self._details


class _Stub:
  """Records the request; returns `reply` or raises it."""

  def __init__(self, reply):
    self.reply, self.calls = reply, []

  def __getattr__(self, rpc):
    def call(request, timeout):
      self.calls.append((rpc, request, timeout))
      if isinstance(self.reply, Exception):
        raise self.reply
      return self.reply
    return call


def test_names_and_tool_set(tools):
  assert "move_to_pose" in tools.by_name
  assert "get_operation" in tools.by_name and "wait_operation" in tools.by_name
  assert "compute_ik" in tools.by_name
  assert "get_point_cloud" not in tools.by_name
  assert len(tools.by_name) == len(api.DESCRIPTOR.services_by_name["RobotService"].methods) - 1
  for _, _, tool in tools.by_name.values():
    assert tool.description and tool.input_schema["type"] == "object"


def test_move_to_pose_schema(tools):
  schema = tools.by_name["move_to_pose"][2].input_schema
  pose = schema["properties"]["pose"]
  assert pose["type"] == "object"
  assert pose["properties"]["qw"] == {"type": "number"}
  assert schema["properties"]["motion"]["enum"] == ["ANY", "LINEAR"]
  assert schema["properties"]["wait"]["type"] == "boolean"
  assert schema["properties"]["wait"]["default"] is True
  assert tools.by_name["get_state"][2].input_schema == {"type": "object", "properties": {}}
  timeout = tools.by_name["wait_operation"][2].input_schema["properties"]["timeout"]
  assert timeout["type"] == "string" and '"30s"' in timeout["description"]
  waypoint = tools.by_name["move_waypoints"][2].input_schema["properties"]["waypoints"]["items"]
  assert "oneof `target`" in waypoint["properties"]["pose"]["description"]


def test_description_has_field_comments(tools):
  description = tools.by_name["move_to_pose"][2].description
  assert description.startswith("Move the tool to a pose")
  assert "`pose` (Pose): target pose of `tool`, in the robot base frame" in description
  assert "LINEAR (straight line of the tool in Cartesian space)" in description
  assert "`wait` (bool): default true" in description
  assert "oneof `target`" in tools.by_name["check_collisions"][2].description


def test_wait_defaults_true_and_bad_arguments(tools):
  stub = _Stub(operations_pb2.Operation(name="op-1"))
  server.invoke(tools, stub, "home", {})
  assert stub.calls[0][1].wait is True
  server.invoke(tools, stub, "home", {"wait": False})
  assert stub.calls[1][1].wait is False
  result, status = server.invoke(tools, stub, "move_to_pose", {"pose": {"x": "far"}})
  assert result.is_error and result.content[0].text.startswith("invalid arguments: ")
  assert status == "INVALID_ARGUMENTS" and len(stub.calls) == 2


def test_operation_json_unpacks_result(tools):
  operation = operations_pb2.Operation(name="op-7", done=True)
  operation.metadata.Pack(api.OperationInfo(kind="MoveToPose", planned_duration_s=1.5, state="DONE", detail="ok"))
  operation.response.Pack(api.MotionResult(final_joints=api.Joints(positions=[0.1] * 7), duration_s=1.4))
  result, status = server.invoke(tools, _Stub(operation), "move_to_pose", {"pose": {"z": 0.4, "qw": 1}})
  reply = json.loads(result.content[0].text)
  assert status == "OK" and not result.is_error
  assert reply["name"] == "op-7" and reply["done"] is True and reply["state"] == "DONE"
  assert reply["planned_duration_s"] == 1.5
  assert reply["response"] == {"final_joints": {"positions": [0.1] * 7}, "duration_s": 1.4}
  failed = operations_pb2.Operation(name="op-8", done=True)
  failed.error.code, failed.error.message = grpc.StatusCode.ABORTED.value[0], "settled before reaching the target"
  reply = json.loads(server.invoke(tools, _Stub(failed), "home", {})[0].content[0].text)
  assert reply["error"] == {"code": "ABORTED", "message": "settled before reaching the target"}


def test_grpc_error_text(tools):
  error = _RpcError(grpc.StatusCode.FAILED_PRECONDITION, "robot faulted: simulation reset")
  result, status = server.invoke(tools, _Stub(error), "home", {})
  assert result.is_error and result.content[0].text == "FAILED_PRECONDITION: robot faulted: simulation reset"
  assert status == "FAILED_PRECONDITION"
  down = _RpcError(grpc.StatusCode.UNAVAILABLE, "failed to connect to all addresses; Connection refused")
  result, _ = server.invoke(tools, _Stub(down), "get_state", {})
  assert result.content[0].text == "UNAVAILABLE: robot service not reachable at 127.0.0.1:18100"


def test_get_image_block(tools):
  png = b"\x89PNG\r\n\x1a\nfake"
  reply = api.GetImageResponse(kind=api.RGB, width=1280, height=720, png=png, sim_time=3.5)
  result, _ = server.invoke(tools, _Stub(reply), "get_image", {"kind": "RGB"})
  image, text = result.content
  assert image.type == "image" and image.mime_type == "image/png"
  assert base64.b64decode(image.data) == png
  assert json.loads(text.text) == {"kind": "RGB", "width": 1280, "height": 720, "sim_time": 3.5}


def test_timeouts(tools):
  stub = _Stub(operations_pb2.Operation(name="op-1"))
  server.invoke(tools, stub, "wait_operation", {"name": "op-1", "timeout": "30s"})
  server.invoke(tools, stub, "wait_operation", {"name": "op-1"})
  server.invoke(tools, stub, "get_state", {})
  assert [call[2] for call in stub.calls] == [40.0, 70.0, 70.0]
