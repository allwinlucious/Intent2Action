"""Read task intents from demonstration clips or typed sentences."""

import base64
from dataclasses import dataclass
import json
import logging
from pathlib import Path
import re
import time

import cv2

from .rules import validate_intent


ROOT = Path(__file__).resolve().parents[1]
LOGGER = logging.getLogger("program_runtime.reader")


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
  decoded = []
  try:
    while True:
      ok, frame = capture.read()
      if not ok:
        break
      decoded.append(frame)
  finally:
    capture.release()
  if len(decoded) < 2:
    raise ValueError(f"{path}: video has fewer than 2 frames")

  indexes = [round(index * (len(decoded) - 1) / (count - 1)) for index in range(count)]
  output = []
  for index in indexes:
    frame = decoded[index]
    height, width = frame.shape[:2]
    if max(height, width) > max_side:
      scale = max_side / max(height, width)
      frame = cv2.resize(frame, (max(1, round(width * scale)), max(1, round(height * scale))),
                         interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".jpg", frame)
    if not ok:
      raise ValueError(f"{path}: could not encode frame {index}")
    output.append(encoded.tobytes())
  return output


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
      message = self.client.chat(messages, model=self.model)
      content = message.get("content")
    except Exception as error:
      problem = f"{type(error).__name__}: {error}"
      raise ReadError("intent reader request failed", [problem], replies) from error
    if not isinstance(content, str):
      return ""
    return content

  def _read(self, user_content) -> Reading:
    messages = [
      {"role": "system", "content": SYSTEM_PROMPT},
      {"role": "user", "content": user_content},
    ]
    replies = []
    first = self._reply(messages, replies)
    replies.append(first)
    reading, problems = _parse(first)
    if reading is not None:
      return reading
    messages.extend([
      {"role": "assistant", "content": first},
      {
        "role": "user",
        "content": "The reply has these problems:\n" + "\n".join(f"- {problem}" for problem in problems)
                   + "\nReply again in the same format.",
      },
    ])
    second = self._reply(messages, replies)
    replies.append(second)
    reading, problems = _parse(second)
    if reading is None:
      raise ReadError("intent reader returned two invalid replies", problems, replies)
    return reading

  def _log(self, source: str, frame_count: int, started: float, error=None) -> None:
    values = {
      "source": source,
      "frame_count": frame_count,
      "latency": time.monotonic() - started,
      "ok": error is None,
    }
    if error is not None:
      values["error"] = str(error)
      values["problems"] = error.problems if isinstance(error, ReadError) else [str(error)]
      LOGGER.error("intent read failed", extra=values)
    else:
      LOGGER.info("intent read succeeded", extra=values)

  def read_clip(self, path, count: int = 6) -> Reading:
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
      parts.append({
        "type": "text",
        "text": "Frames of a person demonstrating a tabletop task, in order; the first is before, the last "
                "after. Read the task intent and reply in the required format.",
      })
      reading = self._read(parts)
    except Exception as error:
      self._log(source, count, started, error=error)
      raise
    self._log(source, count, started)
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
