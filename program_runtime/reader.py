"""Read task intents from demonstration clips or typed sentences."""

import base64
import copy
from dataclasses import dataclass, field
import json
import logging
from pathlib import Path
import re
import time

import cv2

from .rules import validate_intent


ROOT = Path(__file__).resolve().parents[1]
LOGGER = logging.getLogger("program_runtime.reader")
MAX_CLIP_FRAMES = 3600
MAX_CLIP_SIDE = 4096


def _examples() -> str:
  directory = ROOT / "program_runtime" / "examples"
  sections = []
  for name in ("cubes_in_box", "sort_by_color", "yellow_on_red"):
    sections.append(f"{name}:\n```json\n{(directory / f'{name}.json').read_text().strip()}\n```")
  return "\n\n".join(sections)


SYSTEM_PROMPT = (
  "Read the demonstrated or described tabletop task and return its intent. The intent is one JSON object with "
  "only the root keys rules, description, and others. rules is required and contains 1 to 6 rules. description "
  "and others are optional in the schema, but include both: description is a short string and others is exactly "
  "\"stay\". Each rule has exactly the keys select, relation, and target. "
  "select and target are selector objects whose only allowed keys are category, color, and size. category and "
  "color values are non-empty strings. size is one of small, medium, or large. select may be empty; target must "
  "not be empty. relation is either \"in\" or \"on\". Do not repeat a rule. Name selected objects, boxes, and "
  "targets by attributes, never by position or id.\n\n"
  "Reply with a line starting with SEEN: followed by a concise account of what was demonstrated or described, "
  "then put the intent in one ```json fenced block.\n\n"
  f"Worked examples:\n{_examples()}"
)


@dataclass
class Reading:
  intent: dict
  seen: str
  raw: str
  messages: list[dict] = field(default_factory=list)
  frames: list[bytes] = field(default_factory=list)


class ReadError(Exception):
  def __init__(self, message: str, problems: list[str], replies: list[str]):
    super().__init__(message)
    self.problems = problems
    self.replies = replies


def frames(path, count: int = 6, max_side: int = 768) -> list[bytes]:
  """Returns evenly spaced JPEG frames, including the clip's first and last frames."""
  path = Path(path)
  if count < 2:
    raise ValueError(f"{path}: frame count must be at least 2")
  capture = cv2.VideoCapture(str(path))
  if not capture.isOpened():
    capture.release()
    raise ValueError(f"{path}: unreadable video")
  width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
  height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
  if width <= 0 or height <= 0 or max(width, height) > MAX_CLIP_SIDE:
    capture.release()
    raise ValueError(f"{path}: invalid or oversized video dimensions {width:g}x{height:g}")
  frame_count = 0
  try:
    while capture.grab():
      frame_count += 1
      if frame_count > MAX_CLIP_FRAMES:
        raise ValueError(f"{path}: video has more than {MAX_CLIP_FRAMES} frames")
  finally:
    capture.release()
  if frame_count < 2:
    raise ValueError(f"{path}: video has fewer than 2 frames")

  indexes = [round(index * (frame_count - 1) / (count - 1)) for index in range(count)]
  selected = set(indexes)
  encoded_by_index = {}
  capture = cv2.VideoCapture(str(path))
  if not capture.isOpened():
    capture.release()
    raise ValueError(f"{path}: unreadable video on second pass")
  width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
  height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
  if width <= 0 or height <= 0 or max(width, height) > MAX_CLIP_SIDE:
    capture.release()
    raise ValueError(f"{path}: invalid or oversized video dimensions {width:g}x{height:g}")
  try:
    for index in range(frame_count):
      if not capture.grab():
        raise ValueError(f"{path}: video ended during second pass")
      if index not in selected:
        continue
      ok, frame = capture.retrieve()
      if not ok:
        raise ValueError(f"{path}: could not retrieve frame {index}")
      frame_height, frame_width = frame.shape[:2]
      if max(frame_height, frame_width) > MAX_CLIP_SIDE:
        raise ValueError(f"{path}: frame {index} is larger than {MAX_CLIP_SIDE}px")
      if max(frame_height, frame_width) > max_side:
        scale = max_side / max(frame_height, frame_width)
        frame = cv2.resize(
          frame, (max(1, round(frame_width * scale)), max(1, round(frame_height * scale))),
          interpolation=cv2.INTER_AREA,
        )
      ok, encoded = cv2.imencode(".jpg", frame)
      if not ok:
        raise ValueError(f"{path}: could not encode frame {index}")
      encoded_by_index[index] = encoded.tobytes()
  finally:
    capture.release()
  try:
    return [encoded_by_index[index] for index in indexes]
  except KeyError as error:
    raise ValueError(f"{path}: could not sample frame {error.args[0]}") from error


def _parse(content) -> tuple[Reading | None, list[str]]:
  if not isinstance(content, str):
    return None, ["assistant content must be a string"]
  problems = []
  blocks = re.findall(r"```json\s*(.*?)```", content, re.DOTALL | re.IGNORECASE)
  intent = None
  if not blocks:
    problems.append("missing ```json fenced block")
  else:
    try:
      intent = json.loads(blocks[-1])
    except json.JSONDecodeError as error:
      problems.append(f"invalid JSON: {error}")
  seen_match = re.search(r"^SEEN:\s*(.*?)(?=```json|\Z)", content, re.MULTILINE | re.DOTALL)
  seen = seen_match.group(1).strip() if seen_match else ""
  if not seen:
    problems.append("missing or empty SEEN account")
  if intent is not None:
    problems.extend(validate_intent(intent))
  if problems:
    return None, problems
  return Reading(intent, seen, content), []


class IntentReader:
  def __init__(self, client, model: str = "G0"):
    self.client = client
    self.model = model

  def _reply(self, messages: list[dict], replies: list[str]) -> str:
    try:
      message = self.client.chat(copy.deepcopy(messages), model=self.model)
      content = message.get("content")
    except Exception as error:
      problem = f"{type(error).__name__}: {error}"
      raise ReadError("intent reader request failed", [problem], replies) from error
    if not isinstance(content, str):
      return ""
    return content

  def _complete(self, messages: list[dict], images: list[bytes] | None = None) -> Reading:
    messages = copy.deepcopy(messages)
    replies = []
    first = self._reply(messages, replies)
    replies.append(first)
    reading, problems = _parse(first)
    messages.append({"role": "assistant", "content": first})
    if reading is not None:
      reading.messages = messages
      reading.frames = list(images or [])
      return reading
    messages.append({
      "role": "user",
      "content": "The reply has these problems:\n" + "\n".join(f"- {problem}" for problem in problems)
                 + "\nReply again in the same format.",
    })
    second = self._reply(messages, replies)
    replies.append(second)
    reading, problems = _parse(second)
    if reading is None:
      raise ReadError("intent reader returned two invalid replies", problems, replies)
    messages.append({"role": "assistant", "content": second})
    reading.messages = messages
    reading.frames = list(images or [])
    return reading

  def _read(self, user_content, images: list[bytes] | None = None) -> Reading:
    return self._complete([
      {"role": "system", "content": SYSTEM_PROMPT},
      {"role": "user", "content": user_content},
    ], images)

  def _log(self, source: str, frame_count: int, started: float, error=None, note: bool = False) -> None:
    values = {
      "source": source,
      "frame_count": frame_count,
      "latency": time.monotonic() - started,
      "ok": error is None,
      "note": note,
    }
    if error is not None:
      values["error"] = str(error)
      values["problems"] = error.problems if isinstance(error, ReadError) else [str(error)]
      LOGGER.error("intent read failed", extra=values)
    else:
      LOGGER.info("intent read succeeded", extra=values)

  def read_clip(self, path, count: int = 6, note: str | None = None) -> Reading:
    started = time.monotonic()
    source = str(path)
    try:
      try:
        images = frames(path, count=count)
      except ValueError as error:
        raise ReadError("could not read clip", [str(error)], []) from error
      parts = [{
        "type": "image_url",
        "image_url": {"url": "data:image/jpeg;base64," + base64.b64encode(image).decode("ascii")},
      } for image in images]
      text = (
        "Frames of a person demonstrating a tabletop task, in order; the first is before, the last "
        "after. Read the task intent and reply in the required format."
      )
      if note:
        text += (
          f' The person also wrote: "{note}". Use it together with the frames; where they disagree, '
          "the note wins."
        )
      parts.append({
        "type": "text",
        "text": text,
      })
      reading = self._read(parts, images)
    except Exception as error:
      self._log(source, count, started, error=error, note=bool(note))
      raise
    self._log(source, count, started, note=bool(note))
    return reading

  def read_text(self, sentence: str) -> Reading:
    started = time.monotonic()
    try:
      reading = self._read(sentence)
    except Exception as error:
      self._log("text", 0, started, error=error)
      raise
    self._log("text", 0, started)
    return reading

  def clarify(self, reading: Reading, correction: str) -> Reading:
    started = time.monotonic()
    messages = copy.deepcopy(reading.messages)
    messages.append({
      "role": "user",
      "content": f"That is not what I meant: {correction}. Read the task again with this correction and "
                 "reply in the same format.",
    })
    try:
      updated = self._complete(messages, reading.frames)
    except Exception as error:
      self._log("clarify", 0, started, error=error)
      raise
    self._log("clarify", 0, started)
    return updated
