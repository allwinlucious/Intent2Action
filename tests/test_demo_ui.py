import http.client
import json
from pathlib import Path
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

import cv2
import numpy as np
import pytest

from demo_ui.server import DemoHTTPServer, FrameWatcher, build_server
from program_runtime.runtime import RunResult


def start(server):
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()
  host, port = server.server_address
  return f"http://{host}:{port}", thread


def stop(server, thread):
  server.shutdown()
  server.server_close()
  thread.join(2)


@pytest.fixture
def demo(tmp_path):
  server = build_server(port=0, fake=True, frames=tmp_path)
  base, thread = start(server)
  try:
    yield base, server
  finally:
    stop(server, thread)


def get(base, path):
  with urllib.request.urlopen(base + path, timeout=5) as response:
    return response.status, response.headers, response.read()


def post(base, path, value, headers=None):
  request = urllib.request.Request(
    base + path,
    data=json.dumps(value).encode(),
    headers={"Content-Type": "application/json", **(headers or {})},
    method="POST",
  )
  with urllib.request.urlopen(request, timeout=5) as response:
    return response.status, json.load(response)


def post_error(base, path, value, headers=None):
  with pytest.raises(urllib.error.HTTPError) as caught:
    post(base, path, value, headers)
  return caught.value.code, json.loads(caught.value.read())


def typed_program(base, text="put all the cubes in the red box"):
  _, reading = post(base, "/api/read", {"text": text})
  _, written = post(base, "/api/write", {"intent": reading["intent"], "confirm": True})
  return reading, written


def test_page_static_files_and_sixteen_scenes(demo):
  base, _ = demo
  for path, content_type in (("/", "text/html"), ("/app.js", "text/javascript"),
                             ("/style.css", "text/css")):
    status, headers, body = get(base, path)
    assert status == 200 and headers.get_content_type() == content_type and body
  _, _, body = get(base, "/api/scenes")
  scenes = json.loads(body)
  assert len(scenes) == 16
  assert set(scenes[0]) == {"id", "label", "request"}


def test_typed_read_write_run_streams_runtime_in_order(demo):
  base, _ = demo
  reading, written = typed_program(base)
  assert reading["attempt_id"] and written["program_id"]
  _, started = post(base, "/api/run", {"program_id": written["program_id"]})
  assert started["run_id"]
  _, _, body = get(base, "/api/events")
  text = body.decode()
  names = [line.removeprefix("event: ") for line in text.splitlines() if line.startswith("event: ")]
  for required in ("stage", "runtime", "final"):
    assert required in names
  kinds = [json.loads(line.removeprefix("data: "))["kind"]
           for line in text.splitlines() if line.startswith("data: ") and '"kind"' in line]
  assert kinds.index("line") < kinds.index("call") < kinds.index("result") < kinds.index("done")
  assert names[-1] == "final"


def test_run_gate_rejects_missing_stale_and_scene_revision(demo):
  base, _ = demo
  assert post_error(base, "/api/run", {})[0] == 409
  _, written = typed_program(base)
  assert post_error(base, "/api/run", {"program_id": "old"})[0] == 409
  post(base, "/api/scene", {"id": "yellow_on_red:0"})
  status, body = post_error(base, "/api/run", {"program_id": written["program_id"]})
  assert status == 409 and "stale" in body["error"]


def test_second_run_is_busy_with_deterministic_hold(demo):
  base, server = demo
  started = threading.Event()
  release = threading.Event()

  def held_runner(source, api, **kwargs):
    del source, api
    started.set()
    assert release.wait(2)
    kwargs["log"]({"t": time.time(), "kind": "line", "line": 1})
    return RunResult(True, [], [], [], 0, None)

  server.pipeline.runner = held_runner
  _, written = typed_program(base)
  post(base, "/api/run", {"program_id": written["program_id"]})
  assert started.wait(2)
  assert post_error(base, "/api/run", {"program_id": written["program_id"]})[0] == 409
  release.set()
  deadline = time.monotonic() + 2
  while server.pipeline.active is not None and time.monotonic() < deadline:
    time.sleep(0.01)
  assert server.pipeline.active is None


def test_answer_without_question_and_clarify_without_reading_are_conflicts(demo):
  base, _ = demo
  assert post_error(base, "/api/answer", {"run_id": "x", "question_id": "y", "text": "no"})[0] == 409
  assert post_error(base, "/api/clarify", {"text": "red only"})[0] == 409


def test_question_answer_round_trip_and_stop_while_waiting(demo):
  base, server = demo
  answers = []

  def asking_runner(source, api, **kwargs):
    del source, api
    answer = kwargs["ask"]("Continue?")
    answers.append(answer)
    stopped = kwargs["cancel"].is_set()
    return RunResult(not stopped, [], [], [], 0, "stopped by the operator" if stopped else None)

  server.pipeline.runner = asking_runner
  _, written = typed_program(base)
  post(base, "/api/run", {"program_id": written["program_id"]})
  deadline = time.monotonic() + 2
  while server.pipeline.pending is None and time.monotonic() < deadline:
    time.sleep(0.01)
  pending = server.pipeline.pending
  assert pending and server.pipeline.status(0)["run"] == "waiting"
  post(base, "/api/answer", {"run_id": pending.run_id, "question_id": pending.question_id, "text": "yes"})
  deadline = time.monotonic() + 2
  while server.pipeline.active is not None and time.monotonic() < deadline:
    time.sleep(0.01)
  assert answers == ["yes"]
  assert post_error(base, "/api/answer", {"run_id": pending.run_id, "question_id": pending.question_id,
                                          "text": "again"})[0] == 409

  _, written = typed_program(base)
  post(base, "/api/run", {"program_id": written["program_id"]})
  deadline = time.monotonic() + 2
  while server.pipeline.pending is None and time.monotonic() < deadline:
    time.sleep(0.01)
  post(base, "/api/stop", {})
  deadline = time.monotonic() + 2
  while server.pipeline.active is not None and time.monotonic() < deadline:
    time.sleep(0.01)
  assert server.pipeline.active is None and answers[-1] == ""


def test_approval_round_trip_stale_and_stop(demo):
  base, server = demo
  decisions = []

  def approval_runner(source, api, **kwargs):
    del api
    value = kwargs["approve"](source + "\n", "repair")
    decisions.append(value)
    stopped = kwargs["cancel"].is_set()
    return RunResult(not stopped, [], [], [], 0, "stopped by the operator" if stopped else None)

  server.pipeline.runner = approval_runner
  _, written = typed_program(base)
  post(base, "/api/run", {"program_id": written["program_id"]})
  deadline = time.monotonic() + 2
  while server.pipeline.pending is None and time.monotonic() < deadline:
    time.sleep(0.01)
  pending = server.pipeline.pending
  post(base, "/api/approve", {
    "run_id": pending.run_id, "question_id": pending.question_id, "approved": False, "note": "not yet",
  })
  deadline = time.monotonic() + 2
  while server.pipeline.active is not None and time.monotonic() < deadline:
    time.sleep(0.01)
  assert decisions == [(False, "not yet")]
  assert post_error(base, "/api/approve", {
    "run_id": pending.run_id, "question_id": pending.question_id, "approved": True,
  })[0] == 409

  _, written = typed_program(base)
  post(base, "/api/run", {"program_id": written["program_id"]})
  deadline = time.monotonic() + 2
  while server.pipeline.pending is None and time.monotonic() < deadline:
    time.sleep(0.01)
  post(base, "/api/stop", {})
  deadline = time.monotonic() + 2
  while server.pipeline.active is not None and time.monotonic() < deadline:
    time.sleep(0.01)
  assert server.pipeline.active is None


def test_scripted_and_heldout_scene_load_without_rpc(demo):
  base, server = demo
  for scene_id in ("sort_by_color:1", "cluster:100001"):
    _, body = post(base, "/api/scene", {"id": scene_id})
    assert body["objects"]
  assert server.pipeline.current_scene.id == "cluster:100001"


@pytest.mark.parametrize("body,content_type,status", [
  (b"", "application/json", 400),
  (b"{", "application/json", 400),
  (b"{}", "text/plain", 415),
])
def test_body_errors_are_json_without_tracebacks(demo, body, content_type, status):
  base, _ = demo
  request = urllib.request.Request(
    base + "/api/read", data=body, headers={"Content-Type": content_type}, method="POST",
  )
  with pytest.raises(urllib.error.HTTPError) as caught:
    urllib.request.urlopen(request, timeout=5)
  text = caught.value.read().decode()
  assert caught.value.code == status and "Traceback" not in text and json.loads(text)["error"]


def test_foreign_origin_and_oversized_clip_headers(demo):
  base, _ = demo
  assert post_error(base, "/api/read", {"text": "hello"}, {"Origin": "https://foreign.invalid"})[0] == 403
  parsed = urllib.parse.urlsplit(base)
  connection = http.client.HTTPConnection(parsed.hostname, parsed.port, timeout=5)
  connection.putrequest("POST", "/api/read_clip")
  connection.putheader("Content-Type", "video/webm")
  connection.putheader("Content-Length", str(200 * 1024 * 1024 + 1))
  connection.endheaders()
  response = connection.getresponse()
  body = response.read()
  connection.close()
  assert response.status == 413 and b"Traceback" not in body


def test_cleanup_after_truth_snapshot_failure_allows_next_run(demo):
  base, server = demo
  server.pipeline.fake = False
  server.pipeline.truth = lambda stub: (_ for _ in ()).throw(RuntimeError("snapshot unavailable"))
  _, written = typed_program(base)
  post(base, "/api/run", {"program_id": written["program_id"]})
  _, _, body = get(base, "/api/events")
  assert body.decode().count("event: final") == 1
  assert server.pipeline.active is None
  server.pipeline.fake = True
  _, written = typed_program(base)
  assert post(base, "/api/run", {"program_id": written["program_id"]})[0] == 200


def test_mjpeg_two_frames_half_write_and_restart(tmp_path):
  pipeline_server = build_server(port=0, fake=True, frames=tmp_path)
  pipeline_server.watcher.close()
  watcher = FrameWatcher(tmp_path, fake=False)
  pipeline_server.watcher = watcher
  image0 = np.full((24, 32, 3), (0, 0, 180), dtype=np.uint8)
  assert cv2.imwrite(str(tmp_path / "frame_00000.png"), image0)
  base, thread = start(pipeline_server)
  response = urllib.request.urlopen(base + "/api/sim.mjpg", timeout=8)
  received = bytearray()

  def read_until(parts):
    deadline = time.monotonic() + 7
    while received.count(b"Content-Type: image/jpeg") < parts and time.monotonic() < deadline:
      received.extend(response.read(1))
    assert received.count(b"Content-Type: image/jpeg") >= parts

  try:
    read_until(1)
    (tmp_path / "frame_00001.png").write_bytes(b"not complete")
    time.sleep(0.15)
    image1 = np.full((24, 32, 3), (0, 180, 0), dtype=np.uint8)
    assert cv2.imwrite(str(tmp_path / "frame_00001.png"), image1)
    read_until(2)
    (tmp_path / "frame_00000.png").unlink()
    (tmp_path / "frame_00001.png").unlink()
    watcher.last_scan = time.monotonic() - 5
    image2 = np.full((24, 32, 3), (180, 0, 0), dtype=np.uint8)
    assert cv2.imwrite(str(tmp_path / "frame_00000.png"), image2)
    read_until(3)
  finally:
    response.close()
    stop(pipeline_server, thread)
  assert received.count(b"Content-Type: image/jpeg") >= 3


def test_app_run_endpoint_only_occurs_in_run_click_handler():
  source = (Path(__file__).resolve().parents[1] / "demo_ui" / "static" / "app.js").read_text()
  assert source.count('jsonRequest("/api/run"') == 1
  handler = source.index('$("#run-btn").addEventListener("click"')
  assert source.index('jsonRequest("/api/run"') > handler


def test_writer_switch_routes_to_the_untrained_model(demo):
  base, server = demo
  client = server.pipeline.writer.client
  models = []
  original = client.chat
  client.chat = lambda messages, *a, **k: (models.append(k.get("model")), original(messages, *a, **k))[1]
  _, reading = post(base, "/api/read", {"text": "put all the cubes in the red box"})
  _, written = post(base, "/api/write", {"intent": reading["intent"], "confirm": True, "writer": "untrained"})
  assert written["writer"] == "untrained" and models[-1] == "G0"
  _, written = post(base, "/api/write", {"intent": reading["intent"], "confirm": True})
  assert written["writer"] == "fine_tuned" and models[-1] == "G1d"
  assert post_error(base, "/api/write", {"intent": reading["intent"], "confirm": True, "writer": "big"})[0] == 400


def _silent_clip(path):
  writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 10, (64, 48))
  for value in range(10):
    writer.write(np.full((48, 64, 3), value * 20, dtype=np.uint8))
  writer.release()
  return path


def test_clip_speech_joins_the_note_and_a_failure_does_not_block(demo, tmp_path):
  _, server = demo
  pipeline, seen = server.pipeline, []
  original = pipeline.reader.read_clip
  pipeline.reader.read_clip = lambda path, **k: (seen.append(k.get("note")), original(path, **k))[1]
  clip = _silent_clip(tmp_path / "clip.avi")
  pipeline.transcriber = lambda path: "only the red ones"
  output = pipeline.read_clip(clip, "into the blue box")
  assert output["heard"] == "only the red ones" and seen[-1] == "into the blue box; only the red ones"
  pipeline.transcriber = lambda path: (_ for _ in ()).throw(RuntimeError("asr down"))
  output = pipeline.read_clip(clip, None)
  assert output["heard"] == "" and seen[-1] is None


def test_transcribe_returns_empty_for_a_clip_without_audio(tmp_path, monkeypatch):
  from program_runtime import transcribe as module
  monkeypatch.setattr(module.urllib.request, "urlopen", lambda *a, **k: pytest.fail("no audio: no API call"))
  assert module.transcribe(_silent_clip(tmp_path / "silent.avi"), key="test") == ""
  body, content_type = module.multipart([("request", "", "application/json", b"{}"), ("audio", "a.wav", "audio/wav", b"RIFF")])
  boundary = content_type.split("boundary=")[1]
  assert body.count(f"--{boundary}".encode()) == 3 and b'name="audio"; filename="a.wav"' in body
