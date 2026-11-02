import base64
from contextlib import nullcontext
from io import BytesIO
import json
import threading

import numpy as np
from PIL import Image
import pytest

from learn import chat_server


def test_parse_tool_call_and_reasoning():
  raw = (' to=self<|message|>the centre cube is blocked<|eom|><|start|>assistant to=pick<|message|>'
         '<atem:function_calls>\n<atem:invoke name="pick">\n<atem:parameter name="object_id">obj_12</atem:parameter>\n'
         '<atem:parameter name="wait">true</atem:parameter>\n</atem:invoke>\n</atem:function_calls><|eot|>')
  message = chat_server.parse(raw)
  assert message["content"] is None
  (call,) = message["tool_calls"]
  assert call["function"]["name"] == "pick"
  assert json.loads(call["function"]["arguments"]) == {"object_id": "obj_12", "wait": True}


def test_parse_user_content():
  message = chat_server.parse(' to=user<|message|>{"rung": "skip", "reason": "blocked"}<|eot|>')
  assert message == {"role": "assistant", "content": '{"rung": "skip", "reason": "blocked"}'}


def test_template_messages_turns_string_arguments_into_dicts():
  messages = [{"role": "assistant", "content": None, "tool_calls": [
      {"id": "c1", "type": "function", "function": {"name": "pick", "arguments": '{"object_id": "obj_1"}'}}]}]
  assert chat_server.template_messages(messages)[0]["tool_calls"][0]["function"]["arguments"] == {"object_id": "obj_1"}


def image_url() -> str:
  stream = BytesIO()
  Image.new("RGB", (2, 3), (10, 20, 30)).save(stream, format="PNG")
  return "data:image/png;base64," + base64.b64encode(stream.getvalue()).decode()


def test_template_messages_decodes_data_image_and_keeps_part_order():
  messages = [{"role": "user", "content": [
    {"type": "text", "text": "before"},
    {"type": "image_url", "image_url": {"url": image_url()}},
    {"type": "text", "text": "after"},
  ]}]
  parts = chat_server.template_messages(messages)[0]["content"]
  assert [part["type"] for part in parts] == ["text", "image", "text"]
  assert isinstance(parts[1]["image"], Image.Image)
  assert parts[1]["image"].size == (2, 3)


@pytest.mark.parametrize("url", ["https://example.invalid/image.png", "file:///tmp/image.png"])
def test_template_messages_rejects_non_data_image_urls(url):
  messages = [{"role": "user", "content": [
    {"type": "image_url", "image_url": {"url": url}},
  ]}]
  with pytest.raises(ValueError, match="only data image URLs"):
    chat_server.template_messages(messages)


class Inputs(dict):
  def to(self, device):
    self.device = device
    return self


class FakeProcessor:
  def __init__(self):
    self.calls = []

  def apply_chat_template(self, messages, **kwargs):
    self.calls.append((messages, kwargs))
    if kwargs["tokenize"]:
      return Inputs(input_ids=np.array([[1, 2]]), pixel_values="image tensors")
    return "rendered prompt"


class FakeTokenizer:
  pad_token_id = 0

  def __init__(self):
    self.calls = []
    self.decoded = []

  def __call__(self, prompt, **kwargs):
    self.calls.append((prompt, kwargs))
    return Inputs(input_ids=np.array([[1, 2]]))

  def decode(self, tokens, skip_special_tokens):
    self.decoded.append((tokens.tolist(), skip_special_tokens))
    return ' to=user<|message|>{"ok": true}<|eot|>'


class FakeGenerator:
  def __init__(self):
    self.calls = []

  def disable_adapter(self):
    return nullcontext()

  def generate(self, **kwargs):
    self.calls.append(kwargs)
    return np.array([[1, 2, 3]])


class FakeTorch:
  @staticmethod
  def no_grad():
    return nullcontext()


def fake_model():
  value = object.__new__(chat_server.Model)
  value.processor = FakeProcessor()
  value.tokenizer = FakeTokenizer()
  value.model = FakeGenerator()
  value.torch = FakeTorch()
  value.adapters = set()
  value.stop = [7, 8]
  value.lock = threading.Lock()
  return value


def test_model_chat_image_uses_processor_tensorization():
  model = fake_model()
  result = model.chat({"model": "G0", "messages": [{"role": "user", "content": [
    {"type": "image_url", "image_url": {"url": image_url()}},
    {"type": "text", "text": "what changed?"},
  ]}]})
  messages, kwargs = model.processor.calls[0]
  assert kwargs["tokenize"] is True
  assert kwargs["return_dict"] is True and kwargs["return_tensors"] == "pt"
  assert isinstance(messages[0]["content"][0]["image"], Image.Image)
  assert model.tokenizer.calls == []
  assert model.model.calls[0]["pixel_values"] == "image tensors"
  assert result["choices"][0]["message"]["content"] == '{"ok": true}'


def test_model_chat_text_keeps_prompt_tokenizer_path_and_slicing():
  model = fake_model()
  result = model.chat({"model": "G0", "messages": [{"role": "user", "content": "hello"}]})
  _, kwargs = model.processor.calls[0]
  assert kwargs["tokenize"] is False
  assert model.tokenizer.calls == [(
    "rendered prompt", {"return_tensors": "pt", "add_special_tokens": False},
  )]
  assert model.tokenizer.decoded == [([3], False)]
  assert "pixel_values" not in model.model.calls[0]
  assert result["choices"][0]["message"]["content"] == '{"ok": true}'
