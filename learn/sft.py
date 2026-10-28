"""Training examples in the agent's chat format: (messages, indices of the assistant turns to learn).

Stage 1 (from stage1.py records): the situation is shown as the model's own earlier list_objects / get_state
calls and their results (not trained); then
  positive:   the right call (trained)
  recovery:   a wrong call + the stack's error (not trained), then the right call (trained)
  infeasible: the call + its error (call trained), then a short refusal (trained)
Stage 2 (from agent.py episode records of successful expert runs): every assistant turn is trained.
Tokenisation renders the conversation with Qwen's chat template; only the trained turns get labels.
"""

import json
import random

import agent

IGNORE = -100


def _assistant(name: str, arguments) -> dict:
  return {"role": "assistant", "content": "", "tool_calls": [{"type": "function", "function": {"name": name, "arguments": arguments}}]}


def _tool(text: str) -> dict:
  return {"role": "tool", "content": text}


def stage1_examples(record: dict) -> list[tuple[list[dict], list[int]]]:
  head = [{"role": "system", "content": agent.SYSTEM}, {"role": "user", "content": record["intent"]},
          _assistant("list_objects", {}), _tool(json.dumps(record["situation"]["list_objects"])),
          _assistant("get_state", {}), _tool(json.dumps(record["situation"]["get_state"]))]
  valid, wrong = record["valid"], record["wrong"]
  right = _assistant(valid["name"], valid["arguments"])
  out = []
  if record["infeasible"]:
    if valid["status"] != "OK":
      messages = head + [right, _tool(valid["result"]),
                         {"role": "assistant", "content": f"I can't do that: {valid['result'][:200]}"}]
      out.append((messages, [len(head), len(head) + 2]))
    return out
  if valid["status"] != "OK":
    return out  # the stack refused the intended call itself: ambiguous, not used
  out.append((head + [right], [len(head)]))
  if wrong and wrong["status"] != "OK":
    messages = head + [_assistant(wrong["name"], wrong["arguments"]), _tool(wrong["result"]), right]
    out.append((messages, [len(messages) - 1]))
  return out


def stage2_examples(record: dict) -> list[tuple[list[dict], list[int]]]:
  if not record.get("success") or any(c["status"] != "OK" for c in record["calls"]):
    return []
  messages = record["messages"]
  return [(messages, [i for i, m in enumerate(messages) if m["role"] == "assistant"])]


def load(path, builder) -> list:
  out = []
  with open(path) as f:
    for line in f:
      out += builder(json.loads(line))
  return out


def tokenize(tokenizer, messages: list[dict], trained: list[int], tools: list[dict]) -> tuple[list[int], list[int]]:
  """(input ids, labels) of the whole conversation; labels only on the trained assistant turns."""
  def render(upto: int, gen: bool) -> list[int]:
    return tokenizer.apply_chat_template(messages[:upto], tools=tools, add_generation_prompt=gen,
                                         enable_thinking=False, tokenize=True, return_dict=False)
  ids = render(len(messages), False)
  labels = [IGNORE] * len(ids)
  for i in trained:
    start, end = len(render(i, True)), len(render(i + 1, False))
    assert ids[:start] == render(i, True), "chat template prefix mismatch"
    labels[start:end] = ids[start:end]
  return ids, labels


def mix(stage2: list, stage1: list, replay: float, rng: random.Random) -> list:
  """stage2 plus enough stage-1 examples to make `replay` of the result."""
  n = int(len(stage2) * replay / (1 - replay))
  return rng.sample(stage1, min(n, len(stage1))) + stage2
