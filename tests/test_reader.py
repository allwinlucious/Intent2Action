import json
import logging

import cv2
import numpy as np
import pytest

from program_runtime import cli
from program_runtime.agent import ChatClient, ProgramWriter, RecoveryAgent
from program_runtime.reader import IntentReader, ReadError, Reading, frames


INTENT = {
  "description": "the yellow cube goes on the red cube",
  "rules": [{
    "select": {"category": "cube", "color": "yellow"},
    "relation": "on",
    "target": {"category": "cube", "color": "red"},
  }],
  "others": "stay",
}
VALID = "SEEN: a yellow cube was placed on a red cube\n```json\n" + json.dumps(INTENT) + "\n```"


class FakeClient:
  def __init__(self, replies):
    self.replies = iter(replies)
    self.calls = []

  def chat(self, messages, tools=None, tool_choice=None, model=None):
    self.calls.append({"messages": messages, "tools": tools, "tool_choice": tool_choice, "model": model})
    reply = next(self.replies)
    if isinstance(reply, Exception):
      raise reply
    return {"content": reply}


def test_frames_include_ends_and_resize(tmp_path):
  path = tmp_path / "colors.avi"
  writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 10, (80, 40))
  assert writer.isOpened()
  for index in range(20):
    writer.write(np.full((40, 80, 3), (index * 10, index * 5, index * 2), dtype=np.uint8))
  writer.release()

  images = frames(path, count=6, max_side=32)
  decoded = [cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR) for image in images]
  assert len(decoded) == 6
  assert decoded[0].shape[:2] == (16, 32)
  assert np.mean(decoded[0], axis=(0, 1)) == pytest.approx((0, 0, 0), abs=4)
  assert np.mean(decoded[-1], axis=(0, 1)) == pytest.approx((190, 95, 38), abs=4)


def test_reader_accepts_valid_first_reply():
  client = FakeClient([VALID])
  reading = IntentReader(client).read_text("put the yellow cube on the red cube")
  assert reading.intent == INTENT
  assert reading.seen == "a yellow cube was placed on a red cube"
  assert reading.raw == VALID
  assert len(client.calls) == 1


def test_reader_corrects_invalid_reply_and_lists_problems():
  client = FakeClient(["SEEN:\n```json\n{}\n```", VALID])
  reading = IntentReader(client, model="reader-model").read_text("stack the cubes")
  assert reading.intent == INTENT
  assert [call["model"] for call in client.calls] == ["reader-model", "reader-model"]
  correction = client.calls[1]["messages"][-1]["content"]
  assert "missing or empty SEEN account" in correction
  assert "rules must be a list" in correction
  assert "reply again in the same format" in correction.lower()


def test_reader_two_invalid_replies_reports_problems_and_raw_replies():
  client = FakeClient(["first invalid", "SEEN: still invalid"])
  with pytest.raises(ReadError) as caught:
    IntentReader(client).read_text("a task")
  assert caught.value.problems == ["missing ```json fenced block"]
  assert caught.value.replies == ["first invalid", "SEEN: still invalid"]


def test_reader_clip_sends_requested_image_parts_before_text(monkeypatch):
  counts = []

  def fake_frames(path, count=6, max_side=768):
    del path, max_side
    counts.append(count)
    return [bytes([index]) for index in range(count)]

  monkeypatch.setattr("program_runtime.reader.frames", fake_frames)
  client = FakeClient([VALID])
  IntentReader(client).read_clip("demo.mp4", count=4)
  parts = client.calls[0]["messages"][1]["content"]
  assert counts == [4]
  assert [part["type"] for part in parts] == ["image_url"] * 4 + ["text"]
  assert all(part["image_url"]["url"].startswith("data:image/jpeg;base64,") for part in parts[:-1])


def test_reader_clip_note_is_after_images_and_frames_are_kept(monkeypatch):
  monkeypatch.setattr("program_runtime.reader.frames", lambda path, count=6, max_side=768: [b"jpeg"] * count)
  client = FakeClient([VALID])
  reading = IntentReader(client).read_clip("demo.mp4", note="only the yellow one")
  parts = client.calls[0]["messages"][1]["content"]
  assert parts[-1]["type"] == "text"
  assert 'The person also wrote: "only the yellow one"' in parts[-1]["text"]
  assert len(reading.frames) == 6


def test_clarify_keeps_old_reading_immutable_and_includes_earlier_turns():
  client = FakeClient([VALID, VALID])
  reader = IntentReader(client)
  first = reader.read_text("put yellow on red")
  old_messages = json.loads(json.dumps(first.messages))
  second = reader.clarify(first, "only the yellow cube")
  assert first.messages == old_messages and second is not first
  sent = client.calls[1]["messages"]
  assert sent[:len(old_messages)] == old_messages
  assert "That is not what I meant: only the yellow cube" in sent[-1]["content"]


def test_failed_clarify_does_not_mutate_input():
  reading = Reading(INTENT, "seen", VALID, [{"role": "user", "content": "old"}], [])
  before = json.loads(json.dumps(reading.messages))
  with pytest.raises(ReadError):
    IntentReader(FakeClient(["bad", "bad again"])).clarify(reading, "correction")
  assert reading.messages == before


def test_oversized_reported_dimensions_fail_before_retrieve(monkeypatch):
  class Capture:
    retrieved = 0

    def isOpened(self): return True
    def get(self, prop): return 5000 if prop == cv2.CAP_PROP_FRAME_WIDTH else 10
    def grab(self): return True
    def retrieve(self):
      self.retrieved += 1
      return True, np.zeros((2, 2, 3), dtype=np.uint8)
    def release(self): pass

  capture = Capture()
  monkeypatch.setattr(cv2, "VideoCapture", lambda path: capture)
  with pytest.raises(ValueError, match="oversized"):
    frames("large.mp4")
  assert capture.retrieved == 0


def test_chat_client_call_model_wins_over_extra_body(monkeypatch):
  captured = {}

  class Response:
    def __enter__(self):
      return self

    def __exit__(self, *args):
      pass

    def read(self):
      return b'{"choices":[{"message":{"content":"ok"}}]}'

  def urlopen(request, timeout):
    del timeout
    captured.update(json.loads(request.data))
    return Response()

  monkeypatch.setenv("LLM_EXTRA_BODY", '{"model":"extra-model","custom":true}')
  monkeypatch.setattr("urllib.request.urlopen", urlopen)
  ChatClient("http://model", "key", "default-model").chat([], model="call-model")
  assert captured["model"] == "call-model" and captured["custom"] is True


class RoutingClient:
  def __init__(self, replies):
    self.replies = iter(replies)
    self.models = []

  def chat(self, messages, tools=None, model=None):
    del messages, tools
    self.models.append(model)
    return next(self.replies)


class EmptyTools:
  by_name = {}


def test_writer_and_recovery_route_their_models():
  writer_client = RoutingClient([{"content": "verify(lambda: True)"}])
  ProgramWriter(writer_client, model="writer-model").write({"rules": []}, [])
  recovery_client = RoutingClient([{"content": '{"rung":"stop","reason":"done"}'}])
  RecoveryAgent(recovery_client, object(), EmptyTools(), model="recovery-model").recover({"budget": {}})
  assert writer_client.models == ["writer-model"]
  assert recovery_client.models == ["recovery-model"]


def test_cli_read_text_writes_output(monkeypatch, tmp_path, capsys):
  monkeypatch.setattr(cli, "ChatClient", lambda: FakeClient([VALID]))
  output = tmp_path / "intent.json"
  assert cli.main(["read", "--text", "stack the cubes", "--out", str(output)]) == 0
  captured = capsys.readouterr()
  assert captured.out.startswith("SEEN: a yellow cube")
  assert json.loads(output.read_text()) == INTENT
  assert captured.err == ""


def test_cli_unreadable_clip_has_clean_error(monkeypatch, tmp_path, capsys):
  monkeypatch.setattr(cli, "ChatClient", lambda: FakeClient([]))
  assert cli.main(["read", "--clip", str(tmp_path / "missing.mp4")]) == 1
  captured = capsys.readouterr()
  assert "missing.mp4" in captured.err
  assert "Traceback" not in captured.err


def test_cli_client_exception_has_clean_error(monkeypatch, capsys):
  monkeypatch.setattr(cli, "ChatClient", lambda: FakeClient([TimeoutError("late")]))
  assert cli.main(["read", "--text", "stack them"]) == 1
  captured = capsys.readouterr()
  assert "TimeoutError: late" in captured.err
  assert "Traceback" not in captured.err


def test_cli_rejects_one_frame(capsys):
  with pytest.raises(SystemExit) as caught:
    cli.main(["read", "--text", "stack them", "--frames", "1"])
  assert caught.value.code == 2
  assert "must be at least 2" in capsys.readouterr().err


def test_cli_frame_count_reaches_reader_frames(monkeypatch, capsys):
  counts = []

  def fake_frames(path, count=6, max_side=768):
    del path, max_side
    counts.append(count)
    return [b"jpeg"] * count

  monkeypatch.setattr("program_runtime.reader.frames", fake_frames)
  monkeypatch.setattr(cli, "ChatClient", lambda: FakeClient([VALID]))
  assert cli.main(["read", "--clip", "demo.mp4", "--frames", "4"]) == 0
  assert counts == [4]
  capsys.readouterr()


def test_cli_note_is_forwarded_and_rejected_with_text(monkeypatch, capsys):
  monkeypatch.setattr("program_runtime.reader.frames", lambda path, count=6, max_side=768: [b"jpeg"] * count)
  client = FakeClient([VALID])
  monkeypatch.setattr(cli, "ChatClient", lambda: client)
  assert cli.main(["read", "--clip", "demo.mp4", "--note", "red only"]) == 0
  assert "red only" in client.calls[0]["messages"][1]["content"][-1]["text"]
  capsys.readouterr()
  with pytest.raises(SystemExit) as caught:
    cli.main(["read", "--text", "stack", "--note", "red only"])
  assert caught.value.code == 2


def test_reader_logs_one_success_record(caplog):
  caplog.set_level(logging.INFO, logger="program_runtime.reader")
  IntentReader(FakeClient([VALID])).read_text("stack them")
  records = [record for record in caplog.records if record.name == "program_runtime.reader"]
  assert len(records) == 1
  record = records[0]
  assert record.source == "text" and record.frame_count == 0
  assert record.latency >= 0 and record.ok is True


def test_reader_logs_one_failure_record(caplog):
  caplog.set_level(logging.INFO, logger="program_runtime.reader")
  with pytest.raises(ReadError):
    IntentReader(FakeClient(["bad", "also bad"])).read_text("stack them")
  records = [record for record in caplog.records if record.name == "program_runtime.reader"]
  assert len(records) == 1
  record = records[0]
  assert record.source == "text" and record.frame_count == 0
  assert record.latency >= 0 and record.ok is False
  assert record.problems
