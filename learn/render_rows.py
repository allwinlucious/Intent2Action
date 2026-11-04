"""Renders the message rows of learn/sft_data.py build with Glimmer's chat template, for Axolotl's template-free
`input_output` format: the prompt is not trained on, the assistant's reply is. Runs in .venv-ml (transformers).

  .venv-ml/bin/python learn/render_rows.py --data logs/sft/G1

Writes DIR/rendered/{train,validation}.jsonl, one `{"segments": [{"label": false, "text": prompt}, {"label": true,
"text": reply}]}` per row, and DIR/rendered/manifest.json. Rows over 4096 tokens (Axolotl's sequence_len) are
dropped and counted. Only the tokenizer files of the model are downloaded.
"""

import argparse
import json
from pathlib import Path

import transformers

MODEL = "meta-models/Muse-Glimmer-30B"
MAX_TOKENS = 4096  # Axolotl's sequence_len; --max-tokens for longer rows (recovery prompts with the tool list)


def render(messages: list[dict], tools, tokenizer) -> dict:
  """Every assistant turn is trained (a tool-using recovery has several: the tool calls, then the decision); the
  rest is context. Tool-call arguments go to the template as dicts, as learn/chat_server.py serves them."""
  from chat_server import template_messages
  messages = template_messages(messages)

  def text(upto: int, generation: bool) -> str:
    return tokenizer.apply_chat_template(messages[:upto], tools=tools or None, add_generation_prompt=generation,
                                         tokenize=False, reasoning_strength="low")

  segments, done = [], ""
  for index, message in enumerate(messages):
    if message["role"] != "assistant":
      continue
    prompt, full = text(index, True), text(index + 1, False)
    assert prompt.startswith(done) and full.startswith(prompt), \
      "the template renders earlier turns differently once later turns are appended"
    segments += [{"label": False, "text": prompt[len(done):]}, {"label": True, "text": full[len(prompt):]}]
    done = full
  assert segments and messages[-1]["role"] == "assistant", "a row ends with an assistant turn"
  return {"segments": segments}


def render_file(source: Path, target: Path, tokenizer, max_tokens: int = MAX_TOKENS) -> dict:
  """Renders every row of `source` into `target`; {"rows", "dropped_too_long"}."""
  kept, dropped = [], 0
  for line in source.read_text().splitlines():
    row = json.loads(line)
    rendered = render(row["messages"], row.get("tools"), tokenizer)
    if len(tokenizer("".join(segment["text"] for segment in rendered["segments"]))["input_ids"]) > max_tokens:
      dropped += 1
      continue
    kept.append(rendered)
  target.write_text("".join(json.dumps(row) + "\n" for row in kept))
  return {"rows": len(kept), "dropped_too_long": dropped}


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--data", required=True)
  parser.add_argument("--max-tokens", type=int, default=MAX_TOKENS)
  args = parser.parse_args()
  data = Path(args.data)
  tokenizer = transformers.AutoTokenizer.from_pretrained(MODEL)
  (data / "rendered").mkdir(exist_ok=True)
  manifest = {split: render_file(data / f"{split}.jsonl", data / "rendered" / f"{split}.jsonl", tokenizer,
                               args.max_tokens)
              for split in ("train", "validation")}
  (data / "rendered" / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
  print(f"{data / 'rendered'}: " + " ".join(f"{split} rows={value['rows']} dropped_too_long="
                                             f"{value['dropped_too_long']}" for split, value in manifest.items()))
