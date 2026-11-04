"""A small OpenAI-compatible chat server for Glimmer with fine-tuned adapters (runs on the GPU host; vLLM's Glimmer
image cannot serve adapters). The model is loaded the way the cookbook evaluates adapters: transformers + PEFT on an
NF4 base. `"model": "G0"` is the base (adapters disabled), any other name an adapter from <root>/runs/<name>.

  python chat_server.py --root /workspace/train --adapters G1d --port 8000

POST /v1/chat/completions {model, messages, tools?, max_tokens?, temperature?, chat_template_kwargs?} ->
{"choices": [{"message": {"role": "assistant", "content", "tool_calls"?}}]}. Assistant tool calls are rendered with
dict arguments (Glimmer's template requires them) and parsed back from its ATEM blocks. One request at a time.
"""

import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
import os
from pathlib import Path
import re
import threading
import uuid

from PIL import Image

MODEL = "meta-models/Muse-Glimmer-30B"
INVOKE = re.compile(r'<atem:invoke name="([^"]+)">(.*?)</atem:invoke>', re.DOTALL)
PARAMETER = re.compile(r'<atem:parameter name="([^"]+)">(.*?)</atem:parameter>', re.DOTALL)
DATA_IMAGE = re.compile(r"^data:(image/[^;,]+);base64,(.*)$", re.DOTALL)


def _image(url: str) -> Image.Image:
  if not isinstance(url, str) or not url.startswith("data:"):
    raise ValueError("only data image URLs are accepted; network URLs are not fetched")
  match = DATA_IMAGE.fullmatch(url)
  if not match:
    raise ValueError("image URL must be data:image/...;base64,...")
  try:
    data = base64.b64decode(match.group(2), validate=True)
    image = Image.open(BytesIO(data))
    image.load()
  except Exception as error:
    raise ValueError(f"invalid data image URL: {error}") from error
  return image


def template_messages(messages: list[dict]) -> list[dict]:
  """OpenAI messages -> Glimmer's template: tool-call arguments as dicts, not JSON strings."""
  out = []
  for message in messages:
    content = message.get("content")
    if isinstance(content, list):
      parts = []
      for part in content:
        if part.get("type") == "text":
          parts.append({"type": "text", "text": part.get("text", "")})
        elif part.get("type") == "image_url":
          parts.append({"type": "image", "image": _image((part.get("image_url") or {}).get("url"))})
        else:
          raise ValueError(f"unsupported content part type {part.get('type')!r}")
      message = {**message, "content": parts}
    if message.get("tool_calls"):
      calls = []
      for call in message["tool_calls"]:
        arguments = call["function"].get("arguments") or {}
        if isinstance(arguments, str):
          try:
            arguments = json.loads(arguments) if arguments.strip() else {}
          except json.JSONDecodeError:
            arguments = {}
        calls.append({"id": call.get("id"), "type": "function",
                      "function": {"name": call["function"]["name"], "arguments": arguments}})
      message = {**message, "tool_calls": calls}
    out.append(message)
  return out


def _value(text: str):
  try:
    return json.loads(text)
  except json.JSONDecodeError:
    return text


def parse(raw: str) -> dict:
  """The generated turns (after <|start|>assistant) -> an OpenAI assistant message. Reasoning (to=self) is dropped;
  the user-directed message is the content; any other recipient is a tool call."""
  content, calls = None, []
  for turn in raw.split("<|start|>assistant"):
    match = re.match(r"\s*to=([^<\s]+)\s*<\|message\|>(.*)", turn, re.DOTALL)
    if not match:
      continue
    recipient, body = match.group(1), re.split(r"<\|eom\|>|<\|eot\|>|<\|end_of_text\|>", match.group(2))[0]
    if recipient == "self":
      continue
    if recipient == "user":
      content = body
      continue
    for name, params in INVOKE.findall(body) or [(recipient, "")]:
      arguments = {k: _value(v) for k, v in PARAMETER.findall(params)}
      calls.append({"id": f"call_{uuid.uuid4().hex[:12]}", "type": "function",
                    "function": {"name": name, "arguments": json.dumps(arguments)}})
  message = {"role": "assistant", "content": content}
  if calls:
    message["tool_calls"] = calls
  return message


class Model:
  def __init__(self, root: str, adapters: list[str]):
    import peft
    import torch
    import transformers
    self.torch = torch
    self.processor = transformers.AutoProcessor.from_pretrained(MODEL)
    self.tokenizer = self.processor.tokenizer
    quantization = transformers.BitsAndBytesConfig(
        load_in_4bit=True, bnb_4bit_quant_type="nf4", bnb_4bit_use_double_quant=True,
        bnb_4bit_compute_dtype=torch.bfloat16, bnb_4bit_quant_storage=torch.bfloat16)
    base = transformers.AutoModelForImageTextToText.from_pretrained(MODEL, device_map="cuda", dtype=torch.bfloat16,
                                                                    quantization_config=quantization)
    self.model = peft.PeftModel.from_pretrained(base, f"{root}/runs/{adapters[0]}", adapter_name=adapters[0])
    for name in adapters[1:]:
      self.model.load_adapter(f"{root}/runs/{name}", adapter_name=name)
    self.model.eval()
    self.adapters = set(adapters)
    self.stop = [self.tokenizer.eos_token_id, self.tokenizer.convert_tokens_to_ids("<|eot|>")]
    self.lock = threading.Lock()

  def chat(self, request: dict) -> dict:
    name = request.get("model", "G0")
    if name != "G0" and name not in self.adapters:
      raise ValueError(f"unknown model {name!r}; served: G0, {', '.join(sorted(self.adapters))}")
    kwargs = {"reasoning_strength": "low", **(request.get("chat_template_kwargs") or {})}
    messages = template_messages(request["messages"])
    has_image = any(
      isinstance(message.get("content"), list)
      and any(part.get("type") == "image" for part in message["content"])
      for message in messages
    )
    if has_image:
      inputs = self.processor.apply_chat_template(
        messages, tools=request.get("tools"), add_generation_prompt=True, tokenize=True, return_dict=True,
        return_tensors="pt", **kwargs,
      ).to("cuda")
    else:
      prompt = self.processor.apply_chat_template(messages, tools=request.get("tools"),
                                                  add_generation_prompt=True, tokenize=False, **kwargs)
      inputs = self.tokenizer(prompt, return_tensors="pt", add_special_tokens=False).to("cuda")
    temperature = float(request.get("temperature") or 0.0)
    sampling = {"do_sample": True, "temperature": temperature} if temperature > 0 else {"do_sample": False}
    with self.lock, self.torch.no_grad():
      if name == "G0":
        with self.model.disable_adapter():
          output = self.model.generate(**inputs, max_new_tokens=int(request.get("max_tokens") or 2048),
                                       eos_token_id=self.stop, pad_token_id=self.tokenizer.pad_token_id, **sampling)
      else:
        self.model.set_adapter(name)
        output = self.model.generate(**inputs, max_new_tokens=int(request.get("max_tokens") or 2048),
                                     eos_token_id=self.stop, pad_token_id=self.tokenizer.pad_token_id, **sampling)
    raw = self.tokenizer.decode(output[0][inputs["input_ids"].shape[1]:], skip_special_tokens=False)
    return {"id": f"chatcmpl-{uuid.uuid4().hex[:12]}", "object": "chat.completion", "model": name,
            "choices": [{"index": 0, "message": parse(raw), "finish_reason": "stop"}]}


def serve(model, names: list[str], port: int, api_key: str = "", tag: str = "chat_server", host: str = "127.0.0.1") -> None:
  """OpenAI-compatible HTTP front for any `model` with chat(request) -> completion; local only."""

  class Handler(BaseHTTPRequestHandler):
    def _send(self, code: int, body: dict) -> None:
      data = json.dumps(body).encode()
      self.send_response(code)
      self.send_header("Content-Type", "application/json")
      self.send_header("Content-Length", str(len(data)))
      self.end_headers()
      self.wfile.write(data)

    def _authorized(self) -> bool:
      return not api_key or self.headers.get("Authorization") == f"Bearer {api_key}"

    def do_GET(self):  # noqa: N802
      if not self._authorized():
        return self._send(401, {"error": "unauthorized"})
      if self.path.rstrip("/").endswith("/models"):
        return self._send(200, {"data": [{"id": n} for n in names]})
      self._send(404, {"error": "not found"})

    def do_POST(self):  # noqa: N802
      if not self._authorized():
        return self._send(401, {"error": "unauthorized"})
      try:
        request = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
        self._send(200, model.chat(request))
      except Exception as error:  # pylint: disable=broad-except
        self._send(400, {"error": f"{type(error).__name__}: {error}"})

    def log_message(self, fmt, *a):
      print(f"[{tag}] {self.address_string()} {fmt % a}", flush=True)

  print(f"serving {names} on :{port}", flush=True)
  ThreadingHTTPServer((host, port), Handler).serve_forever()


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--root", default="/workspace/train")
  parser.add_argument("--adapters", nargs="+", required=True)
  parser.add_argument("--port", type=int, default=8000)
  parser.add_argument("--api-key", default=os.environ.get("I2A_API_KEY", ""),
                      help="callers must send it as a Bearer token (default: $I2A_API_KEY)")
  parser.add_argument("--host", default="127.0.0.1", help="0.0.0.0 behind a proxy that needs it (Modal); then set --api-key")
  args = parser.parse_args()
  model = Model(args.root, args.adapters)
  serve(model, ["G0", *sorted(model.adapters)], args.port, args.api_key, host=args.host)


if __name__ == "__main__":
  main()
