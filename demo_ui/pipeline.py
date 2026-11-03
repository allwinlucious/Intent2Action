"""Thread-safe state and execution pipeline behind the demo page."""

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
import logging
from pathlib import Path
import queue
import threading
import time
import uuid

from google.protobuf import empty_pb2

from learn import families
from program_runtime.agent import ProgramWriter
from program_runtime.api import RobotAPI
from program_runtime.rules import check_intent, truth_snapshot
from program_runtime.rules import validate_intent
from program_runtime.runtime import run as runtime_run

from .scenes import BY_ID, PRESETS, Preset, fake_objects

LOG = logging.getLogger("demo_ui")


ROOT = Path(__file__).resolve().parents[1]
STOP_SENTINEL = object()


class PipelineError(Exception):
  def __init__(self, status: int, message: str):
    super().__init__(message)
    self.status = status


@dataclass
class Pending:
  kind: str
  run_id: str
  question_id: str
  response: queue.Queue = field(default_factory=lambda: queue.Queue(maxsize=1))


@dataclass
class RunState:
  id: str
  attempt_id: str
  intent: dict
  program: str
  cancel: threading.Event = field(default_factory=threading.Event)
  events: list[dict] = field(default_factory=list)
  condition: threading.Condition = field(default_factory=threading.Condition)
  final_sent: bool = False
  thread: threading.Thread | None = None
  log_stream: object | None = None


class Pipeline:
  def __init__(
      self,
      backend,
      reader,
      writer,
      agent=None,
      *,
      fake: bool = False,
      robot_stub=None,
      sim_stub=None,
      truth=truth_snapshot,
      checker=check_intent,
      runner=runtime_run,
      untrained_model: str | None = None,
      transcriber=None,
  ):
    self.backend = backend
    # The writer for the side-by-side comparison: the same endpoint, the base model without the fine-tuning.
    self.untrained_model = untrained_model
    # Speech in a clip (a spoken instruction) -> text, added to the clip's note; None: frames and typed note only.
    self.transcriber = transcriber
    self.api = RobotAPI(backend)
    self.reader = reader
    self.writer = writer
    self.agent = agent
    self.fake = fake
    self.robot_stub = robot_stub
    self.sim_stub = sim_stub
    self.truth = truth
    self.checker = checker
    self.runner = runner
    self.lock = threading.Lock()
    self.reading = None
    self.round = 0
    self.attempt_id = None
    self.confirmed_intent = None
    self.program = None
    self.program_id = None
    self.program_revisions = None
    self.scene_rev = 0
    self.intent_rev = 0
    self.scene_loading = False
    self.current_scene = PRESETS[0]
    self.active: RunState | None = None
    self.last_run: RunState | None = None
    self.pending: Pending | None = None
    self.trail_events = []
    self.model_status = {"status": "unknown", "message": ""}
    self.robot_status = {"status": "ok" if fake else "unknown", "message": ""}
    self.robot_probe_at = 0.0
    self.robot_probe_running = False
    self.attempt_log_lock = threading.Lock()

  @staticmethod
  def _identifier() -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{stamp}-{uuid.uuid4().hex[:8]}"

  def _heard(self, path: Path) -> str:
    """The clip's speech as text; a failure is logged and the clip is read from frames and the typed note."""
    if self.transcriber is None:
      return ""
    try:
      return self.transcriber(path)
    except Exception as error:  # pylint: disable=broad-except
      LOG.warning("transcription failed for %s: %s", path.name, error)
      return ""

  @staticmethod
  def _reading_output(reading) -> dict:
    return {"seen": reading.seen, "intent": reading.intent}

  def _attempt_record(self, kind: str, inputs: dict, outputs: dict, attempt_id: str | None = None) -> None:
    attempt_id = attempt_id or self.attempt_id
    if not attempt_id:
      return
    path = ROOT / "logs" / "demo_ui" / "attempts" / f"{attempt_id}.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    record = {
      "kind": kind,
      "time": datetime.now(timezone.utc).isoformat(),
      "inputs": inputs,
      "outputs": outputs,
    }
    with self.attempt_log_lock, path.open("a") as stream:
      stream.write(json.dumps(record, separators=(",", ":")) + "\n")

  def _set_model(self, error=None) -> None:
    with self.lock:
      self.model_status = {
        "status": "error" if error else "ok",
        "message": str(error) if error else "",
      }

  def _model_call(self, call):
    try:
      value = call()
    except Exception as error:
      self._set_model(error)
      raise
    self._set_model()
    return value

  def _stage_trail(self, name: str, state: str, detail: str = "") -> None:
    self.trail_events.append({"event": "stage", "data": {"name": name, "state": state, "detail": detail}})

  def scenes(self) -> list[dict]:
    return [preset.public() for preset in PRESETS]

  def load_scene(self, scene_id: str) -> list[dict]:
    preset = BY_ID.get(scene_id)
    if preset is None:
      raise PipelineError(400, "unknown scene")
    with self.lock:
      if self.active is not None:
        raise PipelineError(409, "a run is active")
      if self.scene_loading:
        raise PipelineError(409, "a scene is already loading")
      self.scene_loading = True
      self.scene_rev += 1
      self.confirmed_intent = None
      self.program = None
      self.program_id = None
      self.program_revisions = None
    try:
      if self.fake:
        objects = fake_objects(preset)
        self.backend.objects = {obj.id: obj for obj in objects}
      else:
        families.load_placements(preset.placements, self.sim_stub, self.robot_stub)
      objects = self.backend.list_objects()
    except Exception:
      with self.lock:
        self.scene_loading = False
      raise
    with self.lock:
      self.current_scene = preset
      self.scene_loading = False
    return [asdict(obj) for obj in objects]

  def read_text(self, text: str) -> dict:
    if not isinstance(text, str) or not text.strip():
      raise PipelineError(400, "text must not be empty")
    if len(text) > 1000:
      raise PipelineError(400, "text is longer than 1000 characters")
    attempt_id = self._identifier()
    with self.lock:
      if self.active is not None:
        raise PipelineError(409, "a run is active")
      self.intent_rev += 1
      revision = self.intent_rev
      self.attempt_id = attempt_id
      self.reading = None
      self.round = 0
      self.confirmed_intent = None
      self.program = None
      self.program_id = None
      self.program_revisions = None
      self.trail_events = []
      self._stage_trail("read", "start")
    try:
      reading = self._model_call(lambda: self.reader.read_text(text))
    except Exception as error:
      with self.lock:
        self._stage_trail("read", "failed", str(error))
      self._attempt_record("reading", {"text": text}, {"error": str(error)}, attempt_id)
      raise
    with self.lock:
      if self.intent_rev != revision or self.attempt_id != attempt_id:
        raise PipelineError(409, "the intent changed while reading")
      self.reading = reading
      self.round = 1
      self._stage_trail("read", "done")
    output = {**self._reading_output(reading), "round": 1, "attempt_id": attempt_id}
    self._attempt_record("reading", {"text": text}, output, attempt_id)
    return output

  def read_clip(self, path: Path, note: str | None) -> dict:
    if note is not None and not isinstance(note, str):
      raise PipelineError(400, "note must be text")
    if note is not None and len(note) > 1000:
      raise PipelineError(400, "note is longer than 1000 characters")
    attempt_id = self._identifier()
    with self.lock:
      if self.active is not None:
        raise PipelineError(409, "a run is active")
      self.intent_rev += 1
      revision = self.intent_rev
      self.attempt_id = attempt_id
      self.reading = None
      self.round = 0
      self.confirmed_intent = None
      self.program = None
      self.program_id = None
      self.program_revisions = None
      self.trail_events = []
      self._stage_trail("read", "start")
    try:
      heard = self._heard(path)
      combined = "; ".join(part for part in (note, heard) if part)[:1000]
      reading = self._model_call(lambda: self.reader.read_clip(path, note=combined or None))
    except Exception as error:
      with self.lock:
        self._stage_trail("read", "failed", str(error))
      self._attempt_record("reading", {"clip": path.name, "note": note}, {"error": str(error)}, attempt_id)
      raise
    with self.lock:
      if self.intent_rev != revision or self.attempt_id != attempt_id:
        raise PipelineError(409, "the intent changed while reading")
      self.reading = reading
      self.round = 1
      self._stage_trail("read", "done")
    frames = ["data:image/jpeg;base64," + __import__("base64").b64encode(value).decode("ascii")
              for value in reading.frames]
    output = {
      **self._reading_output(reading), "round": 1, "attempt_id": attempt_id,
      "note": note or "", "heard": heard, "frames": frames,
    }
    self._attempt_record("reading", {"clip": path.name, "note": note, "heard": heard}, output, attempt_id)
    return output

  def clarify(self, correction: str) -> dict:
    if not isinstance(correction, str) or not correction.strip():
      raise PipelineError(400, "clarification must not be empty")
    if len(correction) > 1000:
      raise PipelineError(400, "clarification is longer than 1000 characters")
    with self.lock:
      if self.active is not None:
        raise PipelineError(409, "a run is active")
      if self.reading is None:
        raise PipelineError(409, "there is no reading to clarify")
      previous = self.reading
      attempt_id = self.attempt_id
      self.intent_rev += 1
      revision = self.intent_rev
      self.confirmed_intent = None
      self.program = None
      self.program_id = None
      self.program_revisions = None
    try:
      reading = self._model_call(lambda: self.reader.clarify(previous, correction))
    except Exception as error:
      self._attempt_record(
        "clarification", {"correction": correction}, {"error": str(error)}, attempt_id,
      )
      raise
    with self.lock:
      if self.intent_rev != revision or self.reading is not previous:
        raise PipelineError(409, "the intent changed while clarifying")
      self.reading = reading
      self.round += 1
      round_number = self.round
    output = {**self._reading_output(reading), "round": round_number, "attempt_id": attempt_id}
    self._attempt_record("clarification", {"correction": correction}, output, attempt_id)
    return output

  def write(self, intent: dict, confirm: bool, writer: str = "fine_tuned") -> dict:
    if confirm is not True:
      raise PipelineError(400, "confirm must be true")
    if writer not in ("fine_tuned", "untrained"):
      raise PipelineError(400, "writer must be fine_tuned or untrained")
    program_writer = self.writer
    if writer == "untrained":
      program_writer = ProgramWriter(self.writer.client, model=self.untrained_model)
    problems = validate_intent(intent)
    if problems:
      raise PipelineError(400, "intent: " + "; ".join(problems))
    with self.lock:
      if self.active is not None:
        raise PipelineError(409, "a run is active")
      if self.scene_loading:
        raise PipelineError(409, "the scene is loading")
      if self.reading is None:
        raise PipelineError(409, "there is no reading to confirm")
      was_rewrite = self.program is not None
      self.intent_rev += 1
      revisions = (self.scene_rev, self.intent_rev)
      attempt_id = self.attempt_id
      self.confirmed_intent = json.loads(json.dumps(intent))
      self.program = None
      self.program_id = None
      self.program_revisions = None
      self._stage_trail("write", "start")
    self._attempt_record("confirm", {"intent": intent}, {"confirmed": True}, attempt_id)
    try:
      objects = self.backend.list_objects()
      program = self._model_call(lambda: program_writer.write(intent, objects))
      attempts = program_writer.last_attempts
    except Exception as error:
      with self.lock:
        self._stage_trail("write", "failed", str(error))
      raise
    with self.lock:
      if revisions != (self.scene_rev, self.intent_rev):
        raise PipelineError(409, "the scene or intent changed while writing")
      program_id = uuid.uuid4().hex
      self.program = program
      self.program_id = program_id
      self.program_revisions = revisions
      self._stage_trail("write", "done")
    output = {"program": program, "program_id": program_id, "attempts": attempts, "writer": writer}
    self._attempt_record("rewrite" if was_rewrite else "program", {"intent": intent}, output, attempt_id)
    return output

  def start_run(self, program_id: str) -> str:
    with self.lock:
      if self.active is not None:
        raise PipelineError(409, "a run is already active")
      if not program_id:
        raise PipelineError(409, "program_id is required")
      if program_id != self.program_id or self.program is None:
        raise PipelineError(409, "the program id is stale or unknown")
      if self.program_revisions != (self.scene_rev, self.intent_rev):
        raise PipelineError(409, "the program is stale for the scene or intent")
      if self.confirmed_intent is None:
        raise PipelineError(409, "the intent is not confirmed")
      run_id = self._identifier()
      state = RunState(
        run_id, self.attempt_id, self.confirmed_intent, self.program,
        events=list(self.trail_events),
      )
      log_path = ROOT / "logs" / "demo_ui" / "runs" / f"{run_id}.jsonl"
      log_path.parent.mkdir(parents=True, exist_ok=True)
      state.log_stream = log_path.open("w")
      state.log_stream.write(json.dumps({
        "t": time.time(), "kind": "attempt", "attempt_id": self.attempt_id,
      }, separators=(",", ":")) + "\n")
      state.log_stream.flush()
      self.active = state
      self.last_run = state
      thread = threading.Thread(target=self._run, args=(state,), daemon=True, name=f"demo-run-{run_id}")
      state.thread = thread
      thread.start()
    self._attempt_record("run start", {"program_id": program_id}, {"run_id": run_id}, state.attempt_id)
    return run_id

  def _append(self, state: RunState, event: str, data: dict) -> None:
    envelope = {"event": event, "data": data}
    with state.condition:
      state.events.append(envelope)
      state.condition.notify_all()
    if state.log_stream is not None:
      record = data if event == "runtime" else {"t": time.time(), "kind": event, **data}
      state.log_stream.write(json.dumps(record, separators=(",", ":")) + "\n")
      state.log_stream.flush()

  def _stage(self, state: RunState, name: str, stage_state: str, detail: str = "") -> None:
    self._append(state, "stage", {"name": name, "state": stage_state, "detail": detail})

  def _ask(self, state: RunState, text: str) -> str:
    pending = Pending("question", state.id, uuid.uuid4().hex)
    with self.lock:
      if state.cancel.is_set():
        return ""
      self.pending = pending
    self._append(state, "question", {
      "text": text, "run_id": state.id, "question_id": pending.question_id,
    })
    try:
      value = pending.response.get(timeout=120)
    except queue.Empty:
      with self.lock:
        if self.pending is pending:
          self.pending = None
      return ""
    return "" if value is STOP_SENTINEL else str(value)

  def _approve(self, state: RunState, patch: str | None, reason: str, tool: str | None = None,
               args: dict | None = None) -> tuple[bool, str]:
    """Waits for the operator on a recovery patch, or (with `tool`) on a robot move the model wants to make."""
    pending = Pending("approval", state.id, uuid.uuid4().hex)
    with self.lock:
      if state.cancel.is_set():
        return False, ""
      self.pending = pending
    payload = {"patch": patch, "reason": reason, "run_id": state.id, "question_id": pending.question_id}
    if tool is not None:
      payload.update(tool=tool, args=args or {})
    self._append(state, "approval", payload)
    value = pending.response.get()
    if value is STOP_SENTINEL:
      return False, ""
    approved, note = value
    kind = "move" if tool is not None else "patch"
    self._attempt_record(
      f"{kind} approval" if approved else f"{kind} decline",
      {"run_id": state.id, "question_id": pending.question_id},
      {"approved": approved, "note": note},
      state.attempt_id,
    )
    return approved, note

  def _run(self, state: RunState) -> None:
    final = None
    stage = "run"
    before = None
    runtime_result = None
    try:
      self._stage(state, "run", "start")
      if not self.fake:
        before = self.truth(self.sim_stub)
      if self.agent is not None and hasattr(self.agent, "approve_tool"):
        # Robot moves the recovery agent issues itself pass the same gate as patches (one run at a time).
        self.agent.approve_tool = lambda name, args: self._approve(
            state, None, "The model wants to move the robot", tool=name, args=args)
      runtime_result = self.runner(
        state.program,
        self.api,
        agent=self.agent,
        log=lambda event: self._append(state, "runtime", event),
        ask=lambda text: self._ask(state, text),
        cancel=state.cancel,
        approve=lambda patch, reason: self._approve(state, patch, reason),
      )
      if not runtime_result.ok and runtime_result.error == "stopped by the operator":
        final = {"ok": False, "error": "stopped by the operator", "rules": None}
      else:
        self._stage(state, "run", "done" if runtime_result.ok else "failed", runtime_result.error or "")
        if self.fake:
          final = {"ok": runtime_result.ok, "error": runtime_result.error, "rules": None}
        else:
          stage = "check"
          self._stage(state, "check", "start")
          after = self.truth(self.sim_stub)
          rules = self.checker(state.intent, before, after)
          self._stage(state, "check", "done" if rules.ok else "failed", rules.summary)
          final = {
            "ok": runtime_result.ok and rules.ok,
            "error": runtime_result.error,
            "rules": asdict(rules),
          }
    except Exception as error:
      self._stage(state, stage, "failed", str(error))
      final = {"ok": False, "error": f"{stage}: {error}", "rules": None}
    finally:
      if not state.final_sent:
        self._append(state, "final", final or {
          "ok": False, "error": f"{stage}: run ended without a result", "rules": None,
        })
        state.final_sent = True
      self._attempt_record("final", {"run_id": state.id}, final or {}, state.attempt_id)
      with self.lock:
        pending = self.pending if self.pending and self.pending.run_id == state.id else None
        if pending is not None:
          self.pending = None
        if self.active is state:
          self.active = None
      if pending is not None:
        try:
          pending.response.put_nowait(STOP_SENTINEL)
        except queue.Full:
          pass
      with state.condition:
        state.condition.notify_all()
      if state.log_stream is not None:
        state.log_stream.close()
        state.log_stream = None

  def answer(self, run_id: str, question_id: str, text: str) -> None:
    with self.lock:
      pending = self.pending
      if pending is None or pending.kind != "question" \
          or pending.run_id != run_id or pending.question_id != question_id:
        raise PipelineError(409, "the question is stale, duplicate, or missing")
      self.pending = None
    pending.response.put(text)

  def approve(self, run_id: str, question_id: str, approved: bool, note: str = "") -> None:
    if not isinstance(approved, bool):
      raise PipelineError(400, "approved must be a boolean")
    with self.lock:
      pending = self.pending
      if pending is None or pending.kind != "approval" \
          or pending.run_id != run_id or pending.question_id != question_id:
        raise PipelineError(409, "the approval is stale, duplicate, or missing")
      self.pending = None
    pending.response.put((approved, str(note or "")))

  def stop(self) -> None:
    with self.lock:
      state = self.active
      if state is None:
        raise PipelineError(409, "there is no active run")
      state.cancel.set()
      pending = self.pending if self.pending and self.pending.run_id == state.id else None
      if pending is not None:
        self.pending = None
    if pending is not None:
      try:
        pending.response.put_nowait(STOP_SENTINEL)
      except queue.Full:
        pass
    if not self.fake and self.robot_stub is not None:
      self.robot_stub.Stop(empty_pb2.Empty(), timeout=2)

  def event_run(self) -> RunState | None:
    with self.lock:
      return self.active or self.last_run

  def _probe_robot(self) -> None:
    error = None
    try:
      self.robot_stub.GetState(empty_pb2.Empty(), timeout=2)
    except Exception as caught:
      error = caught
    with self.lock:
      self.robot_status = {
        "status": "error" if error else "ok",
        "message": str(error) if error else "",
      }
      self.robot_probe_at = time.monotonic()
      self.robot_probe_running = False

  def status(self, frame_age: float | None) -> dict:
    with self.lock:
      if not self.fake and not self.robot_probe_running \
          and time.monotonic() - self.robot_probe_at >= 2:
        self.robot_probe_running = True
        threading.Thread(target=self._probe_robot, daemon=True, name="demo-robot-status").start()
      run_status = "waiting" if self.pending is not None else "running" if self.active is not None else "idle"
      return {
        "robot": dict(self.robot_status),
        "model": dict(self.model_status),
        "frames": frame_age,
        "run": run_status,
      }
