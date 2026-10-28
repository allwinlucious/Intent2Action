"""Qwen3 policy (versions A and B): transformers + bitsandbytes 4-bit, optional LoRA adapter, greedy decoding.

Thinking is off (enable_thinking=False). The KV cache of the previous turn is kept and cropped to the common
token prefix, so each turn only prefills the new tool results.
"""

import json
import re

import torch
import transformers

MODEL = "Qwen/Qwen3-8B"
MAX_NEW_TOKENS = 768
TOOL_CALL = re.compile(r"<tool_call>(.*?)(?:</tool_call>|$)", re.S)
THINK = re.compile(r"<think>.*?</think>", re.S)


def load(adapter: str | None = None, model: str = MODEL, embed_on_cpu: bool = True):
  quant = transformers.BitsAndBytesConfig(load_in_4bit=True, bnb_4bit_quant_type="nf4",
                                          bnb_4bit_compute_dtype=torch.bfloat16, bnb_4bit_use_double_quant=True,
                                          llm_int8_enable_fp32_cpu_offload=True)
  tokenizer = transformers.AutoTokenizer.from_pretrained(model)
  # The embedding table (1.2 GB in bf16) stays in CPU memory: Isaac shares the 10 GB GPU, and a lookup is cheap.
  config = transformers.AutoConfig.from_pretrained(model)
  device_map = {"model.embed_tokens": "cpu" if embed_on_cpu else 0, "model.norm": 0, "model.rotary_emb": 0, "lm_head": 0,
                **{f"model.layers.{i}": 0 for i in range(config.num_hidden_layers)}}
  lm = transformers.AutoModelForCausalLM.from_pretrained(model, quantization_config=quant, device_map=device_map,
                                                         dtype=torch.bfloat16)
  if adapter:
    from peft import PeftModel
    lm = PeftModel.from_pretrained(lm, adapter)
  lm.eval()
  return tokenizer, lm


def parse(text: str) -> tuple[str, list[dict]]:
  """(text without tool calls, calls [{name, arguments} or {name, arguments, parse_error}])."""
  text = THINK.sub("", text)
  calls = []
  for body in TOOL_CALL.findall(text):
    try:
      data = json.loads(body.strip())
      calls.append({"name": data["name"], "arguments": data.get("arguments", {})})
    except (json.JSONDecodeError, KeyError, TypeError) as error:
      calls.append({"name": "?", "arguments": {}, "parse_error": f"{error}: {body.strip()[:200]}"})
  return TOOL_CALL.sub("", text).strip(), calls


class Policy:
  def __init__(self, name: str, adapter: str | None = None, thinking: bool = False):
    self.name, self.thinking = name, thinking
    self.tokenizer, self.lm = load(adapter)
    self._cache, self._cached_ids = None, None

  def reset(self, task) -> None:
    self._cache, self._cached_ids = None, None

  def prompt_ids(self, messages, tools) -> torch.Tensor:
    return self.tokenizer.apply_chat_template(messages, tools=tools, add_generation_prompt=True,
                                              enable_thinking=self.thinking, return_tensors="pt",
                                              return_dict=True)["input_ids"]

  @torch.no_grad()
  def act(self, messages, tools) -> tuple[str, list[dict]]:
    ids = self.prompt_ids(messages, tools)
    cache = None
    if self._cached_ids is not None:
      n = min(self._cached_ids.shape[1], ids.shape[1] - 1)
      common = int((self._cached_ids[0, :n] == ids[0, :n]).cumprod(0).sum())
      if common > 0:
        if self._cache.get_seq_length() > common:
          self._cache.crop(common - self._cache.get_seq_length())  # negative: tokens to drop
        cache = self._cache
    out = self.lm.generate(ids, attention_mask=torch.ones_like(ids), past_key_values=cache,
                           max_new_tokens=MAX_NEW_TOKENS, do_sample=False, return_dict_in_generate=True,
                           temperature=None, top_p=None, top_k=None)
    self._cache, self._cached_ids = out.past_key_values, out.sequences.cpu()
    text = self.tokenizer.decode(out.sequences[0, ids.shape[1]:], skip_special_tokens=True)
    return parse(text)


if __name__ == "__main__":
  assert parse('ok <tool_call>\n{"name": "get_state", "arguments": {}}\n</tool_call>') == \
      ("ok", [{"name": "get_state", "arguments": {}}])
  assert parse("<tool_call>{bad</tool_call>")[1][0]["parse_error"]
  print("parse ok")
