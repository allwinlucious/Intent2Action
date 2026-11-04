"""Replay model for the demo panel: answers a model call with the reply logged in a transcript (LLM_TRANSCRIPT,
e.g. demo_runs/transcript.jsonl) when the call matches a logged one exactly, so a screen recording shows real model
output without a served model. A call that was never logged fails, unless a default request is given: then any
unlogged reading returns the logged reading of that request (a staged recording; the reply is real, but not for the
typed text). A staged program, if given, answers every program-writer call (hand-written, not model output)."""

import json
from pathlib import Path

from program_runtime.agent import _without_images


def _key(model, messages) -> str:
  return json.dumps([model, _without_images(messages)], sort_keys=True)


class ReplayModel:
  def __init__(self, transcript: Path | str, default: str | None = None, program: str | None = None):
    self.program = program
    self.replies = {}
    self.fallback = {}  # (model, system prompt) -> the logged reply for the default request
    for line in Path(transcript).read_text().splitlines():
      record = json.loads(line)
      messages = record["request"]["messages"]
      self.replies[_key(record["endpoint"], messages)] = record["reply"]
      if default is not None and messages[-1].get("content") == default:
        self.fallback[(record["endpoint"], messages[0].get("content"))] = record["reply"]
    if default is not None and not self.fallback:
      raise ValueError(f"replay: no logged request {default!r}")

  def chat(self, messages: list[dict], tools=None, tool_choice=None, model=None) -> dict:
    del tools, tool_choice
    if self.program is not None and str(messages[0].get("content", "")).startswith("Write a short Python program"):
      return {"content": self.program}
    reply = self.replies.get(_key(model, messages)) or self.fallback.get((model, messages[0].get("content")))
    if reply is None:
      raise RuntimeError(f"replay: no logged {model} reply for this request (only logged requests can be replayed)")
    return reply
