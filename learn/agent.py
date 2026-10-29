"""One agent loop for every policy (Qwen raw / LoRA / action head / scripted expert), and the recorder.

The tools, their schemas, the system prompt and every call go through mcp_server/server.py's own code
(Tools, invoke, INSTRUCTIONS), so a policy sees exactly what an MCP client sees. The vision tools are hidden
(physics-only experiments). One JSONL line per episode in logs/learn/<run>.jsonl:
  {task, policy, messages, calls: [{name, arguments, status, result, seconds}], final, success, detail, ...}
"""

import dataclasses
import json
import sys
import time

import env  # sets sys.path
import grpc

sys.path.insert(0, str(env.ROOT))
from mcp_server import server as mcp  # noqa: E402
from robot_service import robot_service_pb2_grpc  # noqa: E402

import tasks as tasks_lib  # noqa: E402

HIDDEN = {"get_image", "detect_objects", "calibrate_camera"}
MAX_CALLS = 40
RESULT_CHARS = 4000
SYSTEM = mcp.INSTRUCTIONS + "\n\nWhen the task is done, or cannot be done, stop calling tools and say so in one or two sentences."

_tools = mcp.Tools(mcp.DESCRIPTOR_SET)
_stub = robot_service_pb2_grpc.RobotServiceStub(grpc.insecure_channel(mcp.ADDRESS))


def tool_specs() -> list[dict]:
  """The visible tools in the OpenAI/Qwen function format."""
  return [{"type": "function", "function": {"name": t.name, "description": t.description, "parameters": t.input_schema}}
          for _, _, t in _tools.by_name.values() if t.name not in HIDDEN]


def call(name: str, arguments) -> tuple[str, str]:
  """Runs one tool; returns (status, result text). Operations that finished with an error count as errors."""
  if name in HIDDEN:
    return "UNKNOWN_TOOL", f"unknown tool: {name}"
  if not isinstance(arguments, dict):
    return "INVALID_ARGUMENTS", "invalid arguments: arguments must be a JSON object"
  result, status = mcp.invoke(_tools, _stub, name, arguments)
  text = " ".join(c.text for c in result.content if c.type == "text")
  if status == "OK":
    try:
      data = json.loads(text)
      if isinstance(data, dict) and isinstance(data.get("error"), dict):
        status = data["error"].get("code", "OPERATION_FAILED")
    except json.JSONDecodeError:
      pass
  return status, text[:RESULT_CHARS]


def run_episode(policy, task: tasks_lib.Task, ids: dict, record: dict) -> dict:
  """Runs `policy` on `task` in the current scene; fills and returns `record`."""
  messages = [{"role": "system", "content": SYSTEM}, {"role": "user", "content": task.prompt}]
  specs = tool_specs()
  calls, final = [], None
  policy.reset(task)
  while len(calls) < MAX_CALLS:
    t0 = time.monotonic()
    text, requested = policy.act(messages, specs)
    gen_s = time.monotonic() - t0
    if not requested:
      final = text
      messages.append({"role": "assistant", "content": text})
      break
    messages.append({"role": "assistant", "content": text,
                     "tool_calls": [{"type": "function", "function": {"name": r["name"], "arguments": r["arguments"]}}
                                    for r in requested]})
    for r in requested:
      t1 = time.monotonic()
      if r.get("parse_error"):
        status, result = "INVALID_ARGUMENTS", f"could not parse the tool call: {r['parse_error']}"
      else:
        status, result = call(r["name"], r["arguments"])
      calls.append({"name": r["name"], "arguments": r["arguments"], "status": status, "result": result,
                    "seconds": round(time.monotonic() - t1, 2), "gen_seconds": round(gen_s, 2)})
      messages.append({"role": "tool", "content": result})
      print(f"    {r['name']}({json.dumps(r['arguments'])[:150]}) -> {status} {result[:120]!r}", flush=True)
  record.update(messages=messages, calls=calls, final=final, hit_call_limit=final is None)
  return record


def episode(policy, task: tasks_lib.Task, out) -> dict:
  """A whole episode: random scene, run, check, write one JSONL line."""
  import random
  start = time.monotonic()
  ids = env.new_scene(random.Random(task.seed), task.objects)
  before = tasks_lib.snapshot(ids)
  record = {"task": dataclasses.asdict(task), "policy": policy.name, "ids": ids}
  try:
    run_episode(policy, task, ids, record)
    ok, detail = tasks_lib.check(task, ids, before)
  except Exception as error:  # pylint: disable=broad-except
    ok, detail = False, f"episode error: {error!r}"
    record.setdefault("calls", [])
  record.update(success=ok, detail=detail, seconds=round(time.monotonic() - start, 1))
  out.write(json.dumps(record) + "\n")
  out.flush()
  errors = sum(c["status"] != "OK" for c in record["calls"])
  print(f"{'PASS' if ok else 'FAIL'} {task.id} [{policy.name}] {len(record['calls'])} calls, {errors} errors, "
        f"{record['seconds']:.0f} s: {detail}; final {str(record.get('final'))[:150]!r}", flush=True)
  return record
