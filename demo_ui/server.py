"""Standard-library local HTTP server for the demo page."""

import argparse
from datetime import datetime, timezone
import json
import logging
import os
from pathlib import Path
import re
import secrets
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import cv2
import grpc
import numpy as np

from mcp_server.server import DESCRIPTOR_SET, Tools
from program_runtime.agent import ChatClient, ProgramWriter, RecoveryAgent
from program_runtime.api import FakeBackend, GrpcBackend
from program_runtime.transcribe import transcribe
from program_runtime.reader import IntentReader
from program_runtime.rules import validate_intent

from .pipeline import Pipeline, PipelineError
from .replay import ReplayModel
from .scenes import PRESETS, fake_objects


ROOT = Path(__file__).resolve().parents[1]
STATIC = Path(__file__).resolve().parent / "static"
LOGGER = logging.getLogger("demo_ui")
JSON_LIMIT = 64 * 1024
CLIP_LIMIT = 200 * 1024 * 1024
CLIP_TYPES = {"video/webm": "webm", "video/mp4": "mp4", "video/quicktime": "mov"}
STATIC_FILES = {
  "/": ("index.html", "text/html; charset=utf-8"),
  "/index.html": ("index.html", "text/html; charset=utf-8"),
  "/app.js": ("app.js", "text/javascript; charset=utf-8"),
  "/style.css": ("style.css", "text/css; charset=utf-8"),
}
EXAMPLES = ROOT / "program_runtime" / "examples"


def load_dotenv(path: Path) -> None:
  if not path.exists():
    return
  for raw in path.read_text().splitlines():
    line = raw.strip()
    if not line or line.startswith("#") or "=" not in line:
      continue
    key, value = line.split("=", 1)
    key, value = key.strip(), value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
      value = value[1:-1]
    if key and key not in os.environ:
      os.environ[key] = value


class FakeModel:
  def __init__(self):
    self.intents = {
      name: json.loads((EXAMPLES / f"{name}.json").read_text())
      for name in ("cubes_in_box", "sort_by_color", "yellow_on_red")
    }
    self.programs = {
      name: (EXAMPLES / f"{name}.py").read_text()
      for name in ("cubes_in_box", "sort_by_color", "yellow_on_red")
    }

  @staticmethod
  def _text(messages: list[dict]) -> str:
    values = []
    for message in messages:
      if message.get("role") != "user":
        continue
      content = message.get("content", "")
      if isinstance(content, str):
        values.append(content)
      elif isinstance(content, list):
        values.extend(part.get("text", "") for part in content if part.get("type") == "text")
    return "\n".join(values).lower()

  @staticmethod
  def _task(text: str) -> str:
    if "sort" in text or "same colour" in text or "same color" in text:
      return "sort_by_color"
    if "yellow" in text and "red" in text and (" on " in text or "stack" in text):
      return "yellow_on_red"
    return "cubes_in_box"

  def chat(self, messages: list[dict], tools=None, tool_choice=None, model=None) -> dict:
    del tools, tool_choice, model
    text = self._text(messages)
    system = str(messages[0].get("content", "")) if messages else ""
    if system.startswith("Write a short Python program"):
      return {"content": self.programs[self._task(text)]}
    if system.startswith("Choose the lowest recovery rung"):
      return {"content": '{"rung":"stop","reason":"fake recovery stops"}'}
    task = self._task(text)
    seen = {
      "cubes_in_box": "cubes are placed into the red box",
      "sort_by_color": "red and blue cubes are sorted into matching boxes",
      "yellow_on_red": "the yellow cube is placed on the red cube",
    }[task]
    return {"content": f"SEEN: {seen}\n```json\n{json.dumps(self.intents[task])}\n```"}


class FrameWatcher:
  def __init__(self, directory: Path, fake: bool = False):
    self.directory = Path(directory)
    self.fake = fake
    self._fake_image = None
    self.condition = threading.Condition()
    self.generation = 0
    self.jpeg = None
    self.updated = None
    self.stopped = threading.Event()
    self.last_index = None
    self.last_mtime_ns = None
    self.last_scan = 0.0
    self.thread = threading.Thread(target=self._watch, daemon=True, name="demo-frame-watcher")
    self.thread.start()

  def age(self) -> float | None:
    with self.condition:
      return None if self.updated is None else max(0.0, time.monotonic() - self.updated)

  def close(self) -> None:
    self.stopped.set()
    with self.condition:
      self.condition.notify_all()
    self.thread.join(timeout=2)

  def _publish(self, image) -> None:
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
      return
    with self.condition:
      self.jpeg = encoded.tobytes()
      self.generation += 1
      self.updated = time.monotonic()
      self.condition.notify_all()

  def _fake_frame(self, tick: int):
    # A real Isaac viewer frame (scene cubes_in_box), so fake mode looks like the product.
    if self._fake_image is None:
      self._fake_image = cv2.imread(str(Path(__file__).resolve().parent / "fake_view.jpg"))
    return self._fake_image

  def _scan(self) -> Path | None:
    newest = None
    try:
      entries = os.scandir(self.directory)
    except OSError:
      return None
    with entries:
      for entry in entries:
        match = re.fullmatch(r"frame_(\d+)\.png", entry.name)
        if match and entry.is_file():
          index = int(match.group(1))
          if newest is None or index > newest[0]:
            newest = (index, Path(entry.path))
    if newest is None:
      return None
    index, path = newest
    try:
      mtime_ns = path.stat().st_mtime_ns
    except OSError:
      return None
    if self.last_index is None or index != self.last_index or mtime_ns != self.last_mtime_ns:
      return path
    return None

  def _candidate(self) -> Path | None:
    now = time.monotonic()
    if self.last_index is None:
      self.last_scan = now
      return self._scan()
    candidate = self.directory / f"frame_{self.last_index + 1:05d}.png"
    if candidate.exists():
      return candidate
    if now - self.last_scan >= 5:
      self.last_scan = now
      return self._scan()
    return None

  def _watch(self) -> None:
    tick = 0
    while not self.stopped.is_set():
      if self.fake:
        self._publish(self._fake_frame(tick))
        tick += 1
      else:
        path = self._candidate()
        if path is not None:
          image = cv2.imread(str(path), cv2.IMREAD_COLOR)
          if image is not None:
            match = re.fullmatch(r"frame_(\d+)\.png", path.name)
            if match:
              self.last_index = int(match.group(1))
              try:
                self.last_mtime_ns = path.stat().st_mtime_ns
              except OSError:
                self.last_mtime_ns = None
            self._publish(image)
      self.stopped.wait(1 / 15)


class DemoHTTPServer(ThreadingHTTPServer):
  daemon_threads = True

  def __init__(self, port: int, pipeline: Pipeline, watcher: FrameWatcher):
    self.pipeline = pipeline
    self.watcher = watcher
    super().__init__(("127.0.0.1", port), Handler)

  def server_close(self):
    self.watcher.close()
    super().server_close()


class Handler(BaseHTTPRequestHandler):
  server: DemoHTTPServer
  protocol_version = "HTTP/1.1"

  def log_message(self, format, *args):
    del format, args

  def _record(self, status: int, started: float) -> None:
    LOGGER.info(
      "%s %s %d %.1fms", self.command, urlsplit(self.path).path, status,
      (time.monotonic() - started) * 1000,
    )

  def _send(self, status: int, body: bytes, content_type: str, extra=None) -> int:
    self.send_response(status)
    self.send_header("Content-Type", content_type)
    self.send_header("Content-Length", str(len(body)))
    self.send_header("Cache-Control", "no-store")
    for key, value in extra or []:
      self.send_header(key, value)
    self.end_headers()
    self.wfile.write(body)
    return status

  def _json(self, status: int, value: dict | list) -> int:
    return self._send(status, json.dumps(value, separators=(",", ":")).encode(), "application/json")

  def _error(self, status: int, message: str) -> int:
    return self._json(status, {"error": str(message)})

  def _origin(self) -> None:
    origin = self.headers.get("Origin")
    port = self.server.server_port
    if origin and origin not in {f"http://127.0.0.1:{port}", f"http://localhost:{port}"}:
      raise PipelineError(403, "foreign origin")

  def _length(self, maximum: int) -> int:
    raw = self.headers.get("Content-Length")
    if raw is None:
      raise PipelineError(411, "Content-Length is required")
    try:
      length = int(raw)
    except ValueError as error:
      raise PipelineError(400, "invalid Content-Length") from error
    if length < 0:
      raise PipelineError(400, "invalid Content-Length")
    if length > maximum:
      raise PipelineError(413, "request body is too large")
    return length

  def _read_exact(self, length: int) -> bytes:
    chunks = []
    remaining = length
    while remaining:
      chunk = self.rfile.read(min(1024 * 1024, remaining))
      if not chunk:
        raise PipelineError(400, "request body ended early")
      chunks.append(chunk)
      remaining -= len(chunk)
    return b"".join(chunks)

  def _json_body(self) -> dict:
    if self.headers.get_content_type() != "application/json":
      raise PipelineError(415, "Content-Type must be application/json")
    data = self._read_exact(self._length(JSON_LIMIT))
    if not data:
      raise PipelineError(400, "JSON body is required")
    try:
      value = json.loads(data)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
      raise PipelineError(400, "invalid JSON") from error
    if not isinstance(value, dict):
      raise PipelineError(400, "JSON body must be an object")
    return value

  def do_GET(self):
    started = time.monotonic()
    status = 500
    try:
      status = self._get()
    except (BrokenPipeError, ConnectionResetError):
      status = 200
    except PipelineError as error:
      status = self._error(error.status, str(error))
    except Exception as error:
      LOGGER.error("GET %s failed: %s", urlsplit(self.path).path, error)
      try:
        status = self._error(502, str(error))
      except (BrokenPipeError, ConnectionResetError):
        status = 502
    finally:
      self._record(status, started)

  def _get(self) -> int:
    path = urlsplit(self.path).path
    if path in STATIC_FILES:
      name, content_type = STATIC_FILES[path]
      return self._send(200, (STATIC / name).read_bytes(), content_type)
    if path == "/api/scenes":
      return self._json(200, self.server.pipeline.scenes())
    if path == "/api/status":
      return self._json(200, self.server.pipeline.status(self.server.watcher.age()))
    if path == "/api/events":
      return self._events()
    if path == "/api/sim.mjpg":
      return self._mjpeg()
    return self._error(404, "not found")

  def do_POST(self):
    started = time.monotonic()
    status = 500
    try:
      self._origin()
      status = self._post()
    except PipelineError as error:
      status = self._error(error.status, str(error))
    except (BrokenPipeError, ConnectionResetError):
      status = 400
    except Exception as error:
      LOGGER.error("POST %s failed: %s", urlsplit(self.path).path, error)
      status = self._error(502, str(error))
    finally:
      self._record(status, started)

  def _post(self) -> int:
    parsed = urlsplit(self.path)
    path = parsed.path
    if path == "/api/read_clip":
      return self._clip(parsed.query)
    if path == "/api/transcribe":
      return self._transcribe()
    body = self._json_body()
    pipeline = self.server.pipeline
    if path == "/api/scene":
      return self._json(200, {"objects": pipeline.load_scene(body.get("id"))})
    if path == "/api/read":
      return self._json(200, pipeline.read_text(body.get("text")))
    if path == "/api/clarify":
      return self._json(200, pipeline.clarify(body.get("text")))
    if path == "/api/write":
      intent = body.get("intent")
      problems = validate_intent(intent)
      if problems:
        raise PipelineError(400, "intent: " + "; ".join(problems))
      return self._json(200, pipeline.write(intent, body.get("confirm"), body.get("writer", "fine_tuned")))
    if path == "/api/run":
      return self._json(200, {"run_id": pipeline.start_run(body.get("program_id"))})
    if path == "/api/answer":
      pipeline.answer(body.get("run_id"), body.get("question_id"), body.get("text", ""))
      return self._json(200, {"ok": True})
    if path == "/api/approve":
      pipeline.approve(
        body.get("run_id"), body.get("question_id"), body.get("approved"), body.get("note", ""),
      )
      return self._json(200, {"ok": True})
    if path == "/api/stop":
      pipeline.stop()
      return self._json(200, {"ok": True})
    return self._error(404, "not found")

  def _clip(self, query: str) -> int:
    content_type = self.headers.get_content_type()
    if content_type not in CLIP_TYPES:
      raise PipelineError(415, "unsupported video Content-Type")
    length = self._length(CLIP_LIMIT)
    note = parse_qs(query, keep_blank_values=True).get("note", [""])[0]
    if len(note) > 1000:
      raise PipelineError(400, "note is longer than 1000 characters")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    directory = ROOT / "logs" / "demo_ui" / "clips"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{stamp}-{secrets.token_hex(3)}.{CLIP_TYPES[content_type]}"
    try:
      remaining = length
      with path.open("wb") as stream:
        while remaining:
          chunk = self.rfile.read(min(1024 * 1024, remaining))
          if not chunk:
            raise PipelineError(400, "request body ended early")
          stream.write(chunk)
          remaining -= len(chunk)
      result = self.server.pipeline.read_clip(path, note or None)
    except Exception:
      try:
        path.unlink()
      except OSError:
        pass
      raise
    return self._json(200, result)

  def _transcribe(self) -> int:
    """Speech for the request box (Muse Voice Transcribe); the text is read like a typed request."""
    if self.headers.get_content_type() not in ("audio/webm", "audio/ogg", "audio/mp4"):
      raise PipelineError(415, "unsupported audio Content-Type")
    length = self._length(CLIP_LIMIT)
    directory = ROOT / "logs" / "demo_ui" / "clips"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{secrets.token_hex(3)}.webm"
    path.write_bytes(self.rfile.read(length))
    try:
      text = transcribe(path)
    finally:
      path.unlink(missing_ok=True)
    return self._json(200, {"text": text})

  def _events(self) -> int:
    state = self.server.pipeline.event_run()
    self.send_response(200)
    self.send_header("Content-Type", "text/event-stream")
    self.send_header("Cache-Control", "no-store")
    self.send_header("Connection", "close")
    self.end_headers()
    if state is None:
      return 200
    index = 0
    while True:
      with state.condition:
        while index >= len(state.events) and not state.final_sent:
          state.condition.wait(timeout=15)
        events = list(state.events[index:])
        index += len(events)
        done = state.final_sent and index >= len(state.events)
      for event in events:
        payload = json.dumps(event["data"], separators=(",", ":"))
        self.wfile.write(f"event: {event['event']}\ndata: {payload}\n\n".encode())
        self.wfile.flush()
      if done:
        return 200

  def _mjpeg(self) -> int:
    watcher = self.server.watcher
    boundary = "demo-frame"
    self.send_response(200)
    self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
    self.send_header("Cache-Control", "no-store")
    self.send_header("Connection", "close")
    self.end_headers()
    generation = 0
    while not watcher.stopped.is_set():
      with watcher.condition:
        while watcher.generation <= generation and not watcher.stopped.is_set():
          watcher.condition.wait(timeout=2)
        if watcher.stopped.is_set():
          return 200
        generation = watcher.generation
        jpeg = watcher.jpeg
      if jpeg is None:
        continue
      header = (
        f"--{boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n"
      ).encode()
      self.wfile.write(header + jpeg + b"\r\n")
      self.wfile.flush()
    return 200


def build_server(
    *, port: int = 0, fake: bool = True, frames: Path | str | None = None,
    address: str = "127.0.0.1:18100", sim_address: str | None = None,
    reader_model: str = "G0", writer_model: str = "G1d", recovery_model: str = "G1d",
    replay: Path | str | None = None, replay_default: str | None = None, staged_program: Path | str | None = None,
) -> DemoHTTPServer:
  frames = Path(frames or "/mnt/c/isaac-sim-work/viewer_frames")
  if fake:
    model = ReplayModel(replay, replay_default, Path(staged_program).read_text() if staged_program else None) if replay else FakeModel()
    backend = FakeBackend(fake_objects(PRESETS[0]))
    reader = IntentReader(model, model=reader_model)
    writer = ProgramWriter(model, model=writer_model)
    agent = RecoveryAgent(model, None, Tools(DESCRIPTOR_SET), model=recovery_model)
    pipeline = Pipeline(backend, reader, writer, agent, fake=True, untrained_model=reader_model)
  else:
    load_dotenv(ROOT / ".env")
    model = ChatClient()
    backend = GrpcBackend(address)
    sys.path.insert(0, str(ROOT / "gen"))
    from robot_service import robot_service_pb2_grpc
    robot_stub = backend.stub
    sim_channel = grpc.insecure_channel(sim_address or address)
    sim_stub = robot_service_pb2_grpc.SimServiceStub(sim_channel)
    reader = IntentReader(model, model=reader_model)
    writer = ProgramWriter(model, model=writer_model)
    agent = RecoveryAgent(model, robot_stub, Tools(DESCRIPTOR_SET), model=recovery_model)
    pipeline = Pipeline(
      backend, reader, writer, agent, fake=False, robot_stub=robot_stub, sim_stub=sim_stub,
      untrained_model=reader_model,
      transcriber=transcribe if os.environ.get("META_API_KEY") else None,
    )
  return DemoHTTPServer(port, pipeline, FrameWatcher(frames, fake=fake))


def _parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(prog="python -m demo_ui.server")
  parser.add_argument("--port", type=int, default=8765)
  parser.add_argument("--address", default="127.0.0.1:18100")
  parser.add_argument("--sim-address")
  parser.add_argument("--frames", type=Path, default=Path("/mnt/c/isaac-sim-work/viewer_frames"))
  parser.add_argument("--reader-model", default="G0")
  parser.add_argument("--writer-model", default="G1d")
  parser.add_argument("--recovery-model", default="G1d")
  parser.add_argument("--fake", action="store_true")
  parser.add_argument("--replay", type=Path, help="with --fake: answer model calls with the replies logged in this transcript")
  parser.add_argument("--staged-program", type=Path, help="with --replay: every program-writer call returns this hand-written program")
  parser.add_argument("--replay-default", help="with --replay: any unlogged reading returns the logged reading of this request")
  return parser


def main(argv: list[str] | None = None) -> int:
  args = _parser().parse_args(argv)
  logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
  try:
    server = build_server(
      port=args.port, fake=args.fake, frames=args.frames, address=args.address,
      sim_address=args.sim_address, reader_model=args.reader_model,
      writer_model=args.writer_model, recovery_model=args.recovery_model, replay=args.replay, replay_default=args.replay_default, staged_program=args.staged_program,
    )
  except Exception as error:
    print(f"FAILED: {error}", file=sys.stderr)
    return 1
  host, port = server.server_address
  print(f"demo UI on http://{host}:{port}", flush=True)
  try:
    server.serve_forever()
  except KeyboardInterrupt:
    pass
  finally:
    server.server_close()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
