"""Program writing and recovery clients for OpenAI-compatible chat endpoints."""

import json
import math
import os
from pathlib import Path
import re
import urllib.request

from .api import FakeBackend, Obj, RobotAPI
from .check import BUILTINS, check
from .runtime import dry_run


ROOT = Path(__file__).resolve().parents[1]
EXCLUDED_TOOLS = {"get_image", "detect_objects", "calibrate_camera"}
RUNGS = {"reobserve", "retry_step", "fix_world", "skip", "patch", "ask_user", "stop"}


class ChatClient:
  """Minimal client for an OpenAI-compatible chat completions endpoint. A temperature in LLM_EXTRA_BODY overrides
  the `temperature` argument."""

  def __init__(self, base_url: str | None = None, api_key: str | None = None, model: str | None = None,
               temperature: float = 0.0):
    values = {
      "LLM_BASE_URL": base_url if base_url is not None else os.environ.get("LLM_BASE_URL"),
      "LLM_API_KEY": api_key if api_key is not None else os.environ.get("LLM_API_KEY"),
      "LLM_MODEL": model if model is not None else os.environ.get("LLM_MODEL"),
    }
    for name, value in values.items():
      if not value:
        raise ValueError(f"{name} is not set")
    self.base_url = values["LLM_BASE_URL"].rstrip("/")
    self.api_key = values["LLM_API_KEY"]
    self.model = values["LLM_MODEL"]
    self.temperature = temperature
    # Endpoint-specific request fields, e.g. {"chat_template_kwargs": {"reasoning_strength": "low"}} for Glimmer.
    self.extra = json.loads(os.environ.get("LLM_EXTRA_BODY") or "{}")

  def chat(self, messages: list[dict], tools=None, tool_choice=None) -> dict:
    """Returns the first assistant message from one chat completion."""
    body = {
      "model": self.model,
      "messages": messages,
      "temperature": self.temperature,
      "max_tokens": 8192,  # reasoning models spend part of it before the answer
      **self.extra,
    }
    if tools is not None:
      body["tools"] = tools
    if tool_choice is not None:
      body["tool_choice"] = tool_choice
    request = urllib.request.Request(
      f"{self.base_url}/chat/completions",
      data=json.dumps(body).encode(),
      headers={"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"},
      method="POST",
    )
    # LLM_TIMEOUT_S: a self-hosted server that answers one request at a time needs longer than a hosted endpoint.
    with urllib.request.urlopen(request, timeout=float(os.environ.get("LLM_TIMEOUT_S", "120"))) as response:
      result = json.loads(response.read())
    return result["choices"][0]["message"]


def _api_reference() -> str:
  lines = []
  for name in ("find", "find_all", "pick", "place", "held", "inside", "on_top", "verify", "report"):
    doc = " ".join((getattr(RobotAPI, name).__doc__ or "").split())
    lines.append(f"{name}: {doc}")
  return "\n".join(lines)


def _examples() -> str:
  directory = ROOT / "program_runtime" / "examples"
  sections = []
  for name in ("cubes_in_box", "sort_by_color", "yellow_on_red"):
    sections.append(f"{name}:\n```python\n{(directory / f'{name}.py').read_text().rstrip()}\n```")
  return "\n\n".join(sections)


def _program_text(content: str) -> str:
  match = re.search(r"```(?:python)?\s*\n(.*?)```", content, re.DOTALL | re.IGNORECASE)
  return (match.group(1) if match else content).strip() + "\n"


def _scene_line(obj: Obj) -> str:
  size = "x".join(str(round(value * 1000)) for value in (obj.sx, obj.sy, obj.sz))
  line = (f"{obj.id} {obj.category} {obj.color} {obj.size} x={round(obj.x * 1000)} y={round(obj.y * 1000)} "
          f"z={round(obj.z * 1000)} yaw={round(math.degrees(obj.yaw))} size={size}")
  return line + " held" if obj.attached else line


def program_messages(intent: dict, objects: list[Obj]) -> list[dict]:
  """The program writer's prompt (also the training prompt: one source for both)."""
  system = (
    "Write a short Python program using only this API and the listed builtins. Do not import or define "
    "functions. Resolve objects by attributes, never by ids or coordinates. End with verify(lambda: ...). "
    "Coordinates are in millimetres in the robot's base frame; programs read them from object attributes (x, y, "
    "sx, ...) in metres.\n\n"
    f"Builtins: {', '.join(sorted(BUILTINS))}\n\nProgram API:\n{_api_reference()}\n\n"
    f"Examples:\n{_examples()}"
  )
  scene = "\n".join(_scene_line(obj) for obj in objects)
  return [
    {"role": "system", "content": system},
    {"role": "user", "content": f"Intent:\n{json.dumps(intent)}\nScene:\n{scene}"},
  ]


class ProgramWriter:
  """Writes policy-checked programs from intents and perceived scenes."""

  def __init__(self, client: ChatClient):
    self.client = client
    self.last_attempts = 0

  def write(self, intent: dict, objects: list[Obj]) -> str:
    """Returns a program, retrying once after static-check problems or a dry-run error."""
    messages = program_messages(intent, objects)
    last = ""
    for attempt in range(1, 3):
      self.last_attempts = attempt
      last = _program_text(self.client.chat(messages).get("content") or "")
      problems = check(last)
      if not problems:
        error = dry_run(last, RobotAPI(FakeBackend(objects)))
        if error is None:
          return last
        problems = [f"dry run: {error}"]
      if attempt == 1:
        messages.append({"role": "assistant", "content": last})
        messages.append({
          "role": "user",
          "content": "The program failed its checks:\n" + "\n".join(problems) + "\nReturn a corrected program.",
        })
    return last


def _invalid(reason: str) -> dict:
  return {"rung": "stop", "reason": f"invalid decision: {reason}"}


def _parse_decision(content) -> dict:
  if not isinstance(content, str):
    return _invalid("content is not a string")
  decoder = json.JSONDecoder()
  decision = None
  for index, character in enumerate(content):
    if character != "{":
      continue
    try:
      decision, _ = decoder.raw_decode(content[index:])
      break
    except json.JSONDecodeError:
      continue
  if not isinstance(decision, dict):
    return _invalid("no JSON object")
  if decision.get("rung") not in RUNGS:
    return _invalid("unknown rung")
  if not isinstance(decision.get("reason"), str) or not decision["reason"]:
    return _invalid("reason must be a non-empty string")
  if "patch" in decision and decision["patch"] is not None and not isinstance(decision["patch"], str):
    return _invalid("patch must be a string or null")
  if "question" in decision and decision["question"] is not None and not isinstance(decision["question"], str):
    return _invalid("question must be a string or null")
  return decision


def tool_specs(tools) -> list[dict]:
  """The chat tool specs of the recovery agent's visible robot tools (also the training rows' tools)."""
  output = []
  for name, (_, _, tool) in tools.by_name.items():
    if name not in EXCLUDED_TOOLS:
      output.append({
        "type": "function",
        "function": {"name": name, "description": tool.description, "parameters": tool.input_schema},
      })
  return output


def recovery_messages(context: dict) -> list[dict]:
  """The recovery agent's prompt for one handoff context (also the training prompt: one source for both)."""
  system = (
    "Choose the lowest recovery rung that can work and always give a reason. Rungs: reobserve retries "
    "the failed call with fresh objects; retry_step retries it; fix_world uses robot tools then retries; "
    "skip continues; patch restarts a checked replacement program; ask_user asks one question; stop fails. "
    "Patches may use only the program API and builtins, must resolve objects with find, contain no object "
    "ids or float literals, and end with verify(lambda: ...). Respect the remaining budget. You may call "
    "visible robot tools before deciding. Finish with one decision JSON object."
  )
  public_context = {key: value for key, value in context.items() if key != "_backend"}
  return [
    {"role": "system", "content": system},
    {"role": "user", "content": json.dumps(public_context)},
  ]


class RecoveryAgent:
  """Climbs the budgeted recovery ladder, optionally using robot tools first."""

  def __init__(self, client: ChatClient, stub, tools):
    self.client = client
    self.stub = stub
    self.tools = tools

  def _specs(self) -> list[dict]:
    return tool_specs(self.tools)

  def _tool_text(self, call: dict, names: set[str]) -> str:
    from mcp_server import server

    function = call.get("function", {})
    name = function.get("name", "")
    try:
      arguments = json.loads(function.get("arguments", ""))
    except json.JSONDecodeError as error:
      return f"invalid arguments: {error}"
    if name not in names:
      return f"unknown tool: {name}"
    result, status = server.invoke(self.tools, self.stub, name, arguments)
    text = "\n".join(item.text for item in result.content if hasattr(item, "text"))
    return text if status == "OK" else f"ERROR {status}: {text}"

  def recover(self, context: dict) -> dict:
    """Returns one validated recovery decision after at most twelve tool calls."""
    messages = recovery_messages(context)
    specs = self._specs()
    names = {spec["function"]["name"] for spec in specs}
    used = 0
    while used < 12:
      message = self.client.chat(messages, tools=specs)
      calls = message.get("tool_calls") or []
      if not calls:
        return self._decide(messages, message)
      messages.append(message)
      for call in calls:
        # Chat endpoints reject a reply whose tool calls are not all answered, so calls past the limit get one.
        text = self._tool_text(call, names) if used < 12 else "not run: tool budget used"
        used += 1
        messages.append({"role": "tool", "tool_call_id": call.get("id", ""), "content": text[:4000]})
    messages.append({
      "role": "user",
      "content": "Tool budget used. Answer now with the decision JSON only.",
    })
    return self._decide(messages, self.client.chat(messages))

  def _decide(self, messages: list[dict], message: dict) -> dict:
    """Parses the reply; one corrective turn if it is not a valid decision (models drift into prose)."""
    decision = _parse_decision(message.get("content"))
    if not decision["reason"].startswith("invalid decision"):
      return decision
    messages = [*messages, {"role": "assistant", "content": message.get("content") or ""}, {
      "role": "user",
      "content": f"That reply is not a valid decision ({decision['reason']}). Reply with only one JSON object: "
                 '{"rung": ..., "reason": ...} plus "patch" or "question" when the rung needs it.',
    }]
    reply = self.client.chat(messages).get("content")
    decision = _parse_decision(reply)
    if decision["reason"].startswith("invalid decision"):
      decision["reason"] += f"; reply: {str(reply)[:300]!r}"
    return decision


class ScriptedAgent:
  """Deterministic recovery decisions and optional backend calls for tests."""

  def __init__(self, decisions: list[dict], tool_calls: dict[int, list[tuple[str, dict]]] | None = None):
    self.decisions = list(decisions)
    self.tool_calls = tool_calls or {}
    self.used_tool_handoffs: set[int] = set()

  def recover(self, context: dict) -> dict:
    handoff = len(context.get("history", [])) + 1
    backend = context.get("_backend")
    if handoff not in self.used_tool_handoffs:
      for name, kwargs in self.tool_calls.get(handoff, []):
        getattr(backend, name)(**kwargs)
      self.used_tool_handoffs.add(handoff)
    if not self.decisions:
      return {"rung": "stop", "reason": "scripted decisions exhausted"}
    return dict(self.decisions.pop(0))
