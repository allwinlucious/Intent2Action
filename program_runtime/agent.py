"""Program writing and recovery clients for OpenAI-compatible chat endpoints."""

import json
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
  """Minimal client for an OpenAI-compatible chat completions endpoint."""

  def __init__(self, base_url: str | None = None, api_key: str | None = None, model: str | None = None):
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

  def chat(self, messages: list[dict], tools=None, tool_choice=None) -> dict:
    """Returns the first assistant message from one chat completion."""
    body = {
      "model": self.model,
      "messages": messages,
      "temperature": 0,
      "max_tokens": 2048,
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
    with urllib.request.urlopen(request, timeout=120) as response:
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


class ProgramWriter:
  """Writes policy-checked programs from intents and perceived scenes."""

  def __init__(self, client: ChatClient):
    self.client = client
    self.last_attempts = 0

  def write(self, intent: dict, objects: list[Obj]) -> str:
    """Returns a program, retrying once after static-check problems or a dry-run error."""
    system = (
      "Write a short Python program using only this API and the listed builtins. Do not import or define "
      "functions. Resolve objects by attributes, never by ids or coordinates. End with verify(lambda: ...).\n\n"
      f"Builtins: {', '.join(sorted(BUILTINS))}\n\nProgram API:\n{_api_reference()}\n\n"
      f"Examples:\n{_examples()}"
    )
    scene = "\n".join(f"{obj.id} {obj.category} {obj.color} {obj.size}" for obj in objects)
    messages = [
      {"role": "system", "content": system},
      {"role": "user", "content": f"Intent:\n{json.dumps(intent)}\nScene:\n{scene}"},
    ]
    last = ""
    for attempt in range(1, 3):
      self.last_attempts = attempt
      last = _program_text(self.client.chat(messages)["content"])
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


class RecoveryAgent:
  """Climbs the budgeted recovery ladder, optionally using robot tools first."""

  def __init__(self, client: ChatClient, stub, tools):
    self.client = client
    self.stub = stub
    self.tools = tools

  def _specs(self) -> list[dict]:
    output = []
    for name, (_, _, tool) in self.tools.by_name.items():
      if name not in EXCLUDED_TOOLS:
        output.append({
          "type": "function",
          "function": {"name": name, "description": tool.description, "parameters": tool.input_schema},
        })
    return output

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
    system = (
      "Choose the lowest recovery rung that can work and always give a reason. Rungs: reobserve retries "
      "the failed call with fresh objects; retry_step retries it; fix_world uses robot tools then retries; "
      "skip continues; patch restarts a checked replacement program; ask_user asks one question; stop fails. "
      "Patches may use only the program API and builtins, must resolve objects with find, contain no object "
      "ids or float literals, and end with verify(lambda: ...). Respect the remaining budget. You may call "
      "visible robot tools before deciding. Finish with one decision JSON object."
    )
    public_context = {key: value for key, value in context.items() if key != "_backend"}
    messages = [
      {"role": "system", "content": system},
      {"role": "user", "content": json.dumps(public_context)},
    ]
    specs = self._specs()
    names = {spec["function"]["name"] for spec in specs}
    used = 0
    while used < 12:
      message = self.client.chat(messages, tools=specs)
      calls = message.get("tool_calls") or []
      if not calls:
        return _parse_decision(message.get("content"))
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
    return _parse_decision(self.client.chat(messages).get("content"))


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
