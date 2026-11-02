"""Local OpenAI-compatible server for base Qwen3-8B (4-bit, beside Isaac on the 10 GB GPU): rehearses the demo flow
without a cloud GPU. Text only (no image input); not trained on this task, so expect zero-shot quality.

  .venv-ml/bin/python learn/qwen_server.py [--port 8000]
  then LLM_BASE_URL=http://127.0.0.1:8000/v1 LLM_MODEL=qwen LLM_API_KEY=local
"""

import argparse
import json
from pathlib import Path
import sys
import threading
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parent))
import chat_server  # noqa: E402
import qwen  # noqa: E402

MAX_NEW_TOKENS = 2048


def to_openai(raw: str) -> dict:
  """Qwen's reply -> an OpenAI assistant message (tool calls as JSON-string arguments)."""
  text, calls = qwen.parse(raw)
  message = {"role": "assistant", "content": text or None}
  good = [c for c in calls if "parse_error" not in c]
  if good:
    message["tool_calls"] = [{"id": f"call_{uuid.uuid4().hex[:12]}", "type": "function",
                              "function": {"name": c["name"], "arguments": json.dumps(c["arguments"])}}
                             for c in good]
  return message


def text_messages(messages: list[dict]) -> list[dict]:
  for message in messages:
    if isinstance(message.get("content"), list):
      raise ValueError("this local model is text-only: image input is not supported")
  return chat_server.template_messages(messages)


class QwenModel:
  def __init__(self):
    import torch
    self.torch = torch
    self.tokenizer, self.lm = qwen.load()
    self.lock = threading.Lock()

  def chat(self, request: dict) -> dict:
    ids = self.tokenizer.apply_chat_template(text_messages(request["messages"]), tools=request.get("tools"),
                                             add_generation_prompt=True, enable_thinking=False,
                                             return_tensors="pt", return_dict=True)["input_ids"].to("cuda:0")
    with self.lock, self.torch.no_grad():
      output = self.lm.generate(ids, max_new_tokens=min(int(request.get("max_tokens") or MAX_NEW_TOKENS),
                                                        MAX_NEW_TOKENS), do_sample=False,
                                pad_token_id=self.tokenizer.eos_token_id)
    raw = self.tokenizer.decode(output[0][ids.shape[1]:], skip_special_tokens=True)
    return {"id": f"chatcmpl-{uuid.uuid4().hex[:12]}", "object": "chat.completion", "model": "qwen",
            "choices": [{"index": 0, "message": to_openai(raw), "finish_reason": "stop"}]}


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--port", type=int, default=8000)
  args = parser.parse_args()
  chat_server.serve(QwenModel(), ["qwen"], args.port, tag="qwen_server")


if __name__ == "__main__":
  main()
