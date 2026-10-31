from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys

from google.longrunning import operations_pb2
from google.rpc import status_pb2
import pytest

from program_runtime.agent import ChatClient, ProgramWriter, RecoveryAgent, ScriptedAgent
from program_runtime.api import (
  FakeBackend, GrpcBackend, Obj, RobotAPI, RobotFailure, classify, size_class,
)
from program_runtime.check import check
from program_runtime.runtime import Budget, run


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "program_runtime" / "examples"


def obj(object_id, category="cube", color="red", x=0.4, y=0.1, z=0.0, sx=0.05, sy=None, sz=None):
  sy = sx if sy is None else sy
  sz = sx if sz is None else sz
  return Obj(object_id, category, color, size_class(sx, sy, sz), x, y, z, 0.0, sx, sy, sz, False)


def box(object_id="box", color="red", x=0.45, y=-0.22):
  return Obj(object_id, "box", color, "large", x, y, 0.0, 0.0, 0.25, 0.21, 0.08, False)


@pytest.mark.parametrize("name", ["cubes_in_box", "sort_by_color", "yellow_on_red"])
def test_check_accepts_examples(name):
  assert check((EXAMPLES / f"{name}.py").read_text()) == []


@pytest.mark.parametrize("source", [
  "import os\nverify(lambda: True)",
  "__import__('os')\nverify(lambda: True)",
  "open('x')\nverify(lambda: True)",
  "x = find(category='cube')\nx.__class__\nverify(lambda: True)",
  "def f():\n  pass\nverify(lambda: True)",
  "eval('1')\nverify(lambda: True)",
  "unknown()\nverify(lambda: True)",
  "x = find(category='cube')\nx._y\nverify(lambda: True)",
  "verify(True)",
  "report('done')",
])
def test_check_rejects_unsafe_sources(source):
  assert check(source)


@pytest.mark.parametrize("literal", ["'obj_12'", "0.45", "f'obj_{12}'"])
def test_check_rejects_ids_and_floats_for_patches(literal):
  assert check(f"x = {literal}\nverify(lambda: True)", patch=True)


def test_obj_from_world_object_and_size_boundaries():
  value = {
    "id": "a", "category": "cube", "color": "red",
    "pose": {"x": 1, "y": 2, "z": 3, "qz": 2 ** -0.5, "qw": 2 ** -0.5},
    "size_x": 0.03, "size_y": 0.02, "size_z": 0.01, "attached_to": "arm",
  }
  actual = Obj.from_world_object(value)
  assert actual.yaw == pytest.approx(1.57079632679)
  assert actual.size == "small" and actual.attached
  assert [size_class(value, value, value) for value in (0.0399, 0.04, 0.06)] == [
    "small", "medium", "large",
  ]


def test_inside_rotated_box():
  target = replace(box(), yaw=1.57079632679, x=0, y=0)
  cube = obj("cube", x=0.0, y=0.115, z=0.01)
  assert RobotAPI(FakeBackend([cube, target])).inside(cube, target)
  unrotated = replace(target, yaw=0.0)
  assert not RobotAPI(FakeBackend([cube, unrotated])).inside(cube, unrotated)


@pytest.mark.parametrize("name,objects", [
  ("cubes_in_box", [obj("red"), obj("blue", color="blue", x=0.55), box()]),
  ("sort_by_color", [obj("red"), obj("blue", color="blue", x=0.55), box(), box("blue_box", "blue", 0.65)]),
  ("yellow_on_red", [obj("red"), obj("yellow", color="yellow", x=0.55, sx=0.035)]),
])
def test_examples_run(name, objects):
  api = RobotAPI(FakeBackend(objects))
  result = run((EXAMPLES / f"{name}.py").read_text(), api)
  assert result.ok, result.error


class NoMoveBackend(FakeBackend):
  def place(self, object_id, target_id):
    del target_id
    self.objects[object_id] = replace(self.objects[object_id], attached=False)


def test_postcondition_violation_is_classified():
  backend = NoMoveBackend([obj("cube"), box()])
  source = "cube = find(category='cube')\nbox = find(category='box')\npick(cube)\nplace(cube, box)\nverify(lambda: inside(cube, box))"
  result = run(source, RobotAPI(backend))
  assert not result.ok
  assert result.failures[-1]["kind"] == "postcondition"


@pytest.mark.parametrize("code,message,kind", [
  ("DEADLINE_EXCEEDED", "late", "timeout"),
  ("UNKNOWN", "robot faulted", "fault"),
  ("NOT_FOUND", "no object with id x", "perception"),
  ("FAILED_PRECONDITION", "not holding x", "precondition"),
  ("UNKNOWN", "grasp failed", "grasp"),
  ("UNKNOWN", "box is full", "place"),
  ("UNKNOWN", "IK failed", "reachability"),
  ("UNKNOWN", "collision", "collision"),
  ("UNKNOWN", "different", "other"),
])
def test_classify(code, message, kind):
  assert classify(code, message) == kind


def one_cube_source():
  return "cube = find(category='cube')\npick(cube)\nverify(lambda: held(cube))"


def test_handoff_retry_succeeds():
  backend = FakeBackend([obj("cube")])
  backend.inject = {"pick": [RobotFailure("grasp", "blocked")]}
  result = run(one_cube_source(), RobotAPI(backend), ScriptedAgent([
    {"rung": "retry_step", "reason": "transient"},
  ]))
  assert result.ok and len(result.handoffs) == 1
  assert result.handoffs[0]["outcome"] == "resolved"


def test_handoff_budget_exhaustion():
  backend = FakeBackend([obj("cube")])
  backend.inject = {"pick": [RobotFailure("grasp", "blocked") for _ in range(4)]}
  decisions = [{"rung": "retry_step", "reason": "again"} for _ in range(4)]
  result = run(one_cube_source(), RobotAPI(backend), ScriptedAgent(decisions))
  assert not result.ok and len(result.handoffs) == 3


def test_skip_continues():
  backend = FakeBackend([obj("cube")])
  source = "cube = find(category='cube')\npick(cube)\nverify(lambda: True)"
  backend.inject = {"pick": [RobotFailure("grasp", "blocked")]}
  result = run(source, RobotAPI(backend), ScriptedAgent([{"rung": "skip", "reason": "optional"}]))
  assert result.ok


def test_invalid_patch_is_rejected():
  backend = FakeBackend([obj("cube")])
  backend.inject = {"pick": [RobotFailure("grasp", "blocked")]}
  agent = ScriptedAgent([{
    "rung": "patch", "reason": "replace", "patch": "x = 0.45\nverify(lambda: True)",
  }])
  result = run(one_cube_source(), RobotAPI(backend), agent)
  assert not result.ok and result.handoffs[0]["outcome"] == "rejected"


def test_valid_patch_becomes_result():
  backend = FakeBackend([obj("cube")])
  backend.inject = {"pick": [RobotFailure("grasp", "blocked")]}
  patch = "cube = find(category='cube')\npick(cube)\nverify(lambda: held(cube))"
  result = run(one_cube_source(), RobotAPI(backend), ScriptedAgent([{
    "rung": "patch", "reason": "replace", "patch": patch,
  }]))
  assert result.ok and result.patches == 1


def test_ask_user_once_without_second_handoff():
  backend = FakeBackend([obj("cube")])
  backend.inject = {"pick": [RobotFailure("grasp", "blocked")]}
  asked = []
  agent = ScriptedAgent([
    {"rung": "ask_user", "reason": "confirm", "question": "retry?"},
    {"rung": "retry_step", "reason": "confirmed"},
  ])
  result = run(one_cube_source(), RobotAPI(backend), agent, ask=lambda question: asked.append(question) or "yes")
  assert result.ok and asked == ["retry?"] and len(result.handoffs) == 1


def test_step_budget_stops_loop():
  source = "while True:\n  pass\nverify(lambda: True)"
  result = run(source, RobotAPI(FakeBackend([])), budget=Budget(steps=10))
  assert not result.ok and "step budget exceeded" in result.error


class GrowingBackend(FakeBackend):
  def list_objects(self):
    listed = super().list_objects()
    self.objects.update({f"c{number}": obj(f"c{number}") for number in range(200)})
    return listed


def test_step_budget_in_verify_is_not_handed_off():
  # The dry run sees one cube and passes; the real run loops over 200 inside the verify lambda.
  source = "verify(lambda: all(c.x >= 0 for c in find_all(category='cube')))"
  agent = ScriptedAgent([{"rung": "retry_step", "reason": "again"}])
  result = run(source, RobotAPI(GrowingBackend([obj("a")])), agent, budget=Budget(steps=50))
  assert not result.ok and "step budget exceeded" in result.error and result.handoffs == []


def test_dry_run_ignores_box_full():
  small_box = replace(box(), sx=0.09, sy=0.09)
  events = []
  result = run((EXAMPLES / "cubes_in_box.py").read_text(),
               RobotAPI(FakeBackend([obj("a"), obj("b", x=0.55), small_box])), log=events.append)
  dry = [event for event in events if event["kind"] == "dry_run"]
  assert dry[0]["ok"] and dry[0]["error"].startswith("ignored: place: box is full")
  assert not result.ok and result.error.startswith("place: box is full")


def test_dry_run_catches_ambiguous_find_before_real_pick():
  backend = FakeBackend([obj("a"), obj("b", x=0.5)])
  source = "cube = find(category='cube', color='red')\npick(cube)\nverify(lambda: held(cube))"
  result = run(source, RobotAPI(backend))
  assert not result.ok and result.error.startswith("dry run: perception")
  assert not any(value.attached for value in backend.list_objects())


class FakeStub:
  def __init__(self, operation):
    self.operation = operation
    self.waits = 0

  def Pick(self, request, timeout):
    del request, timeout
    return self.operation

  def WaitOperation(self, request, timeout):
    del request, timeout
    self.waits += 1
    return operations_pb2.Operation(name="op", done=True)


def test_grpc_backend_waits_for_operation():
  stub = FakeStub(operations_pb2.Operation(name="op", done=False))
  GrpcBackend(stub=stub).pick("cube")
  assert stub.waits == 1


def test_grpc_backend_operation_error():
  operation = operations_pb2.Operation(
    name="op", done=True, error=status_pb2.Status(code=10, message="grasp collision"),
  )
  with pytest.raises(RobotFailure) as caught:
    GrpcBackend(stub=FakeStub(operation)).pick("cube")
  assert caught.value.kind == "grasp"


def test_grpc_backend_injection_skips_rpc():
  stub = FakeStub(operations_pb2.Operation(name="op", done=True))
  backend = GrpcBackend(stub=stub)
  backend.inject = {"pick": [RobotFailure("grasp", "injected")]}
  with pytest.raises(RobotFailure, match="injected"):
    backend.pick("cube")


def test_chat_client_request(monkeypatch):
  captured = {}

  class Response:
    def __enter__(self):
      return self

    def __exit__(self, *args):
      pass

    def read(self):
      return json.dumps({"choices": [{"message": {"content": "ok"}}]}).encode()

  def open_request(request, timeout):
    captured["request"] = request
    captured["timeout"] = timeout
    return Response()

  monkeypatch.setattr("urllib.request.urlopen", open_request)
  message = ChatClient("http://model", "secret", "test").chat([{"role": "user", "content": "hi"}])
  body = json.loads(captured["request"].data)
  assert message == {"content": "ok"}
  assert body["model"] == "test" and body["temperature"] == 0 and body["max_tokens"] == 8192
  assert captured["request"].get_header("Authorization") == "Bearer secret"


def test_chat_client_missing_environment(monkeypatch):
  monkeypatch.delenv("LLM_BASE_URL", raising=False)
  with pytest.raises(ValueError, match="LLM_BASE_URL is not set"):
    ChatClient()


class WriterClient:
  def __init__(self, replies):
    self.replies = iter(replies)

  def chat(self, messages):
    del messages
    return {"content": next(self.replies)}


def test_program_writer_extracts_and_retries():
  good = "verify(lambda: True)"
  client = WriterClient(["```python\nopen('x')\n```", f"text\n```python\n{good}\n```"])
  writer = ProgramWriter(client)
  assert writer.write({"rules": []}, []) == good + "\n"
  assert writer.last_attempts == 2


def test_program_writer_retries_after_dry_run_error():
  ambiguous = "cube = find(category='cube')\nverify(lambda: held(cube))"
  good = "cubes = find_all(category='cube')\nverify(lambda: len(cubes) == 2)"
  writer = ProgramWriter(WriterClient([ambiguous, good]))
  assert writer.write({"rules": []}, [obj("a"), obj("b", x=0.5)]) == good + "\n"
  assert writer.last_attempts == 2


class Tool:
  description = "test tool"
  input_schema = {"type": "object", "properties": {}}


class ToolSet:
  by_name = {"home": ("Home", "request", Tool())}


class RecoveryClient:
  def __init__(self, replies):
    self.replies = iter(replies)
    self.calls = []

  def chat(self, messages, tools=None):
    self.calls.append((messages, tools))
    return next(self.replies)


def tool_message(name="home", arguments="{}", call_id="one"):
  return {"tool_calls": [{"id": call_id, "function": {"name": name, "arguments": arguments}}]}


def test_recovery_agent_tool_then_decision(monkeypatch):
  from mcp.types import CallToolResult, TextContent

  monkeypatch.setattr(
    "mcp_server.server.invoke",
    lambda tools, stub, name, arguments: (CallToolResult(content=[TextContent(type="text", text="done")]), "OK"),
  )
  client = RecoveryClient([tool_message(), {"content": '{"rung":"retry_step","reason":"done"}'}])
  decision = RecoveryAgent(client, object(), ToolSet()).recover({"budget": {}})
  assert decision["rung"] == "retry_step"
  assert client.calls[1][0][-1]["role"] == "tool"


@pytest.mark.parametrize("message,expected", [
  (tool_message("missing"), "unknown tool"),
  (tool_message(arguments="{"), "invalid arguments"),
])
def test_recovery_agent_tool_errors(message, expected):
  client = RecoveryClient([message, {"content": '{"rung":"stop","reason":"done"}'}])
  RecoveryAgent(client, object(), ToolSet()).recover({"budget": {}})
  assert expected in client.calls[1][0][-1]["content"]


def test_recovery_agent_twelve_call_limit():
  calls = [
    {"id": str(number), "function": {"name": "missing", "arguments": "{}"}}
    for number in range(13)
  ]
  client = RecoveryClient([{"tool_calls": calls}, {"content": '{"rung":"stop","reason":"limit"}'}])
  decision = RecoveryAgent(client, object(), ToolSet()).recover({"budget": {}})
  assert decision["reason"] == "limit"
  assert client.calls[-1][1] is None
  replies = [message for message in client.calls[-1][0] if message.get("role") == "tool"]
  assert len(replies) == 13 and replies[-1]["content"] == "not run: tool budget used"


def test_recovery_agent_invalid_decision_stops():
  client = RecoveryClient([{"content": "not json"}])
  assert RecoveryAgent(client, object(), ToolSet()).recover({"budget": {}})["rung"] == "stop"


def test_cli_fake_scene(tmp_path):
  scene = tmp_path / "scene.json"
  scene.write_text(json.dumps([value.__dict__ for value in [obj("red"), obj("blue", color="blue"), box()]]))
  log = tmp_path / "run.jsonl"
  result = subprocess.run([
    sys.executable, "-m", "program_runtime.cli", "run",
    "--program", str(EXAMPLES / "cubes_in_box.py"),
    "--intent", str(EXAMPLES / "cubes_in_box.json"),
    "--fake-scene", str(scene), "--log", str(log),
  ], cwd=ROOT, text=True, capture_output=True)
  assert result.returncode == 0, result.stdout + result.stderr
  assert result.stdout.rstrip().endswith("OK")
  assert [json.loads(line) for line in log.read_text().splitlines()][-1]["kind"] == "done"


def test_obj_equality_is_by_id():
  from dataclasses import replace
  a = Obj(id="obj_1", category="cube", color="blue", size="medium", x=0.4, y=0.0, z=0.0, yaw=0.0,
          sx=0.05, sy=0.05, sz=0.05, attached=False)
  moved = replace(a, x=0.6, z=0.01)
  assert a == moved and hash(a) == hash(moved) and a != replace(a, id="obj_2")
