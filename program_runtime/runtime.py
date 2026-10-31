"""Checked dry-run and fail-fast execution of generated robot programs."""

import builtins
from dataclasses import asdict, dataclass, replace
import sys
import time
from typing import Callable

from .api import FakeBackend, Obj, RobotAPI, RobotFailure
from .check import API_FUNCTIONS, BUILTINS, check


@dataclass
class Budget:
  handoffs: int = 3
  patches: int = 1
  steps: int = 20_000
  calls: int = 200


@dataclass
class RunResult:
  ok: bool
  report: list[str]
  failures: list[dict]
  handoffs: list[dict]
  patches: int
  error: str | None


class _Patch(Exception):
  def __init__(self, source: str, entry: dict):
    super().__init__(source)
    self.source = source
    self.entry = entry


def _failure_dict(failure: RobotFailure) -> dict:
  return {
    "kind": failure.kind,
    "message": failure.message,
    "call": failure.call,
    "line": failure.line,
  }


def _error_text(failure: RobotFailure) -> str:
  suffix = f" (line {failure.line})" if failure.line is not None else ""
  return f"{failure.kind}: {failure.message}{suffix}"


def _numbered(source: str) -> str:
  return "\n".join(f"{number}: {line}" for number, line in enumerate(source.splitlines(), 1))


def _arg_text(value) -> str:
  if isinstance(value, Obj):
    return value.id
  return str(value)


class _Session:
  def __init__(self, api: RobotAPI, agent, log, budget: Budget, ask):
    self.api = api
    self.agent = agent
    self.log = log
    self.budget = budget
    self.ask = ask
    self.source = ""
    self.line = None
    self.last_logged_line = None
    self.failures: list[dict] = []
    self.handoffs: list[dict] = []
    self.patch_count = 0

  def emit(self, kind: str, **fields) -> None:
    self.log({"t": time.time(), "kind": kind, **fields})

  def _trace(self, frame, event, arg):
    del arg
    # Returning None leaves API and library frames untraced, so only program lines pay the trace cost.
    if frame.f_code.co_filename != "<program>":
      return None
    if event == "line":
      self.budget.steps -= 1
      self.line = frame.f_lineno
      if self.line != self.last_logged_line:
        self.emit("line", line=self.line)
        self.last_logged_line = self.line
      if self.budget.steps < 0:
        raise RobotFailure("other", "step budget exceeded", line=self.line)
    return self._trace

  def _resolve(self, value):
    if not isinstance(value, Obj):
      return value
    fresh = {obj.id: obj for obj in self.api.backend.list_objects()}
    if value.id not in fresh:
      raise RobotFailure("perception", f"object {value.id} no longer exists")
    return fresh[value.id]

  def _record_failure(self, failure: RobotFailure) -> dict:
    if failure.line is None:
      failure.line = self.line
    data = _failure_dict(failure)
    self.failures.append(data)
    self.emit(
      "failure", failure_kind=failure.kind, message=failure.message,
      call=failure.call, line=failure.line,
    )
    return data

  def _context(self, failure: RobotFailure) -> dict:
    context = {
      "program": _numbered(self.source),
      "line": failure.line,
      "kind": failure.kind,
      "message": failure.message,
      "call": failure.call,
      "objects": [asdict(obj) for obj in self.api.backend.list_objects()],
      "state": self.api.backend.state(),
      "history": list(self.handoffs),
      "budget": {"handoffs": self.budget.handoffs, "patches": self.budget.patches},
    }
    if self.agent is not None and self.agent.__class__.__name__ == "ScriptedAgent":
      context["_backend"] = self.api.backend
    return context

  def _decision(self, context: dict) -> dict:
    decision = self.agent.recover(context)
    if not isinstance(decision, dict):
      return {"rung": "stop", "reason": "invalid decision: expected an object"}
    return decision

  def _recover(self, failure: RobotFailure, retry: Callable[[], object]):
    data = self._record_failure(failure)
    if self.agent is None or self.budget.handoffs <= 0:
      raise failure
    number = len(self.handoffs) + 1
    self.emit("handoff", n=number)
    context = self._context(failure)
    decision = self._decision(context)
    self.budget.handoffs -= 1
    asked = False
    while decision.get("rung") == "ask_user" and not asked:
      self.emit(
        "decision", rung="ask_user", reason=decision.get("reason", ""),
        patch=decision.get("patch"), question=decision.get("question"),
      )
      asked = True
      if self.ask is None:
        raise RobotFailure("other", "ask_user is not available", line=self.line)
      context["user_answer"] = self.ask(decision.get("question") or "")
      decision = self._decision(context)
    rung = decision.get("rung", "stop")
    self.emit(
      "decision", rung=rung, reason=decision.get("reason", ""),
      patch=decision.get("patch"), question=decision.get("question"),
    )
    entry = {"failure": data, "decision": decision, "outcome": "failed"}
    self.handoffs.append(entry)
    if rung in {"reobserve", "retry_step", "fix_world"}:
      result = retry()
      entry["outcome"] = "resolved"
      return result
    if rung == "skip":
      entry["outcome"] = "resolved"
      return None
    if rung == "patch":
      patch_source = decision.get("patch")
      problems = check(patch_source, patch=True) if isinstance(patch_source, str) else ["patch is not a string"]
      accepted = self.budget.patches > 0 and not problems
      if self.budget.patches <= 0:
        problems = ["patch budget exhausted", *problems]
      self.emit("patch", accepted=accepted, problems=problems)
      if not accepted:
        entry["outcome"] = "rejected"
        raise failure
      self.budget.patches -= 1
      self.patch_count += 1
      raise _Patch(patch_source, entry)
    raise failure

  def _postcondition_retry(self, name: str, args: tuple, failure: RobotFailure):
    self.budget.calls -= 1
    if self.budget.calls < 0:
      raise RobotFailure("other", "call budget exceeded", line=self.line)
    try:
      resolved = tuple(self._resolve(value) for value in args)
    except RobotFailure as retry_failure:
      retry_failure.call = failure.call
      return self._recover(
        retry_failure, lambda: self._postcondition_retry(name, args, retry_failure)
      )
    self.emit("call", fn="postcondition", args=[_arg_text(value) for value in resolved])
    if not self.api._postcondition(name, resolved):
      self.emit("result", fn="postcondition", ok=False, message=failure.message)
      repeated = RobotFailure("postcondition", failure.message, failure.call, self.line)
      return self._recover(repeated, lambda: self._postcondition_retry(name, args, repeated))
    self.emit("result", fn="postcondition", ok=True, message="")
    return None

  def _retry_call(self, name: str, args: tuple, kwargs: dict):
    try:
      resolved = tuple(self._resolve(value) for value in args)
    except RobotFailure as failure:
      failure.call = name
      return self._recover(failure, lambda: self._retry_call(name, args, kwargs))
    return self.call(name, *resolved, **kwargs)

  def call(self, name: str, *args, **kwargs):
    self.budget.calls -= 1
    if self.budget.calls < 0:
      raise RobotFailure("other", "call budget exceeded", call=name, line=self.line)
    texts = [_arg_text(value) for value in args]
    texts.extend(f"{key}={_arg_text(value)}" for key, value in kwargs.items())
    self.emit("call", fn=name, args=texts)
    method = getattr(self.api, name)
    try:
      result = method(*args, **kwargs)
    except RobotFailure as failure:
      # A step-budget failure (from a verify lambda) is final: Python also unsets the trace after it raises.
      if self.budget.steps < 0:
        raise
      if not failure.call:
        failure.call = name
      self.emit("result", fn=name, ok=False, message=failure.message)
      if failure.kind == "postcondition" and name in {"pick", "place"}:
        retry = lambda: self._postcondition_retry(name, args, failure)
      elif name == "verify":
        retry = lambda: self.call(name, *args, **kwargs)
      else:
        retry = lambda: self._retry_call(name, args, kwargs)
      return self._recover(failure, retry)
    self.emit("result", fn=name, ok=True, message="")
    if name == "report":
      self.emit("report", text=str(args[0]))
    return result

  def execute(self, source: str) -> None:
    self.source = source
    self.last_logged_line = None

    def bound(name: str):
      return lambda *args, **kwargs: self.call(name, *args, **kwargs)

    namespace = {
      "__builtins__": {name: getattr(builtins, name) for name in BUILTINS},
      **{name: bound(name) for name in API_FUNCTIONS},
    }
    code = compile(source, "<program>", "exec")
    previous = sys.gettrace()
    sys.settrace(self._trace)
    try:
      exec(code, namespace, namespace)
    finally:
      sys.settrace(previous)


def dry_run(source: str, api: RobotAPI, log=lambda event: None, budget: Budget | None = None) -> str | None:
  """Runs *source* on a FakeBackend copy of *api*'s scene; returns the error text, or None if it passed."""
  # Only the dry run's summary event is logged: its line/call events would mix with the real run's in the UI.
  session = _Session(None, None, lambda event: None, replace(budget or Budget(), handoffs=0, patches=0), None)
  summary = lambda **fields: log({"t": time.time(), "kind": "dry_run", **fields})
  failure = None
  try:
    session.api = RobotAPI(FakeBackend(api.backend.list_objects()))
    session.execute(source)
  except RobotFailure as error:
    failure = error
  except Exception as error:
    failure = RobotFailure("other", str(error))
  if failure is None:
    summary(ok=True, error=None)
    return None
  if failure.line is None:
    failure.line = session.line
  text = _error_text(failure)
  if failure.kind == "place" and failure.message == "box is full":
    # The fake's slot grid cannot model a real box with more free space; real execution decides.
    summary(ok=True, error=f"ignored: {text}")
    return None
  summary(ok=False, error=text)
  return text


def run(
    source: str,
    api: RobotAPI,
    agent=None,
    log=lambda event: None,
    budget: Budget | None = None,
    ask: Callable[[str], str] | None = None,
) -> RunResult:
  """Checks, dry-runs, and executes a program with budgeted recovery."""
  remaining = replace(budget or Budget())
  report_start = len(api.report_lines)
  session = _Session(api, agent, log, remaining, ask)
  current = source
  pending_patch = None
  error = None
  ok = False
  while True:
    problems = check(current)
    session.emit("check", problems=problems)
    if problems:
      error = "check: " + "; ".join(problems)
      if pending_patch:
        pending_patch["outcome"] = "failed"
      break
    dry_error = dry_run(current, api, session.log, remaining)
    if dry_error:
      error = "dry run: " + dry_error
      if pending_patch:
        pending_patch["outcome"] = "failed"
      break
    try:
      session.execute(current)
    except _Patch as patch:
      current = patch.source
      pending_patch = patch.entry
      continue
    except RobotFailure as failure:
      if failure.line is None:
        failure.line = session.line
      if _failure_dict(failure) not in session.failures:
        session._record_failure(failure)
      error = _error_text(failure)
      if pending_patch:
        pending_patch["outcome"] = "failed"
      break
    except Exception as exception:
      error = f"other: {exception}"
      if pending_patch:
        pending_patch["outcome"] = "failed"
      break
    if pending_patch:
      pending_patch["outcome"] = "resolved"
    ok = True
    break
  result = RunResult(
    ok=ok,
    report=list(api.report_lines[report_start:]),
    failures=session.failures,
    handoffs=session.handoffs,
    patches=session.patch_count,
    error=error,
  )
  session.emit(
    "done", ok=result.ok, error=result.error, handoffs=len(result.handoffs), patches=result.patches,
  )
  return result
