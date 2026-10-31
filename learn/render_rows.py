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
MAX_TOKENS = 4096


def render(messages: list[dict], tools, tokenizer) -> dict:
  prompt = tokenizer.apply_chat_template(messages[:-1], tools=tools or None, add_generation_prompt=True,
                                         tokenize=False, reasoning_strength="low")
  full = tokenizer.apply_chat_template(messages, tools=tools or None, tokenize=False, reasoning_strength="low")
  assert full.startswith(prompt), "the template renders the prompt differently with the reply appended"
  return {"segments": [{"label": False, "text": prompt}, {"label": True, "text": full[len(prompt):]}]}


def render_file(source: Path, target: Path, tokenizer) -> dict:
  """Renders every row of `source` into `target`; {"rows", "dropped_too_long"}."""
  kept, dropped = [], 0
  for line in source.read_text().splitlines():
    row = json.loads(line)
    rendered = render(row["messages"], row.get("tools"), tokenizer)
    if len(tokenizer("".join(segment["text"] for segment in rendered["segments"]))["input_ids"]) > MAX_TOKENS:
      dropped += 1
      continue
    kept.append(rendered)
  target.write_text("".join(json.dumps(row) + "\n" for row in kept))
  return {"rows": len(kept), "dropped_too_long": dropped}


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--data", required=True)
  args = parser.parse_args()
  data = Path(args.data)
  tokenizer = transformers.AutoTokenizer.from_pretrained(MODEL)
  (data / "rendered").mkdir(exist_ok=True)
  manifest = {split: render_file(data / f"{split}.jsonl", data / "rendered" / f"{split}.jsonl", tokenizer)
              for split in ("train", "validation")}
  (data / "rendered" / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
  print(f"{data / 'rendered'}: " + " ".join(f"{split} rows={value['rows']} dropped_too_long="
                                             f"{value['dropped_too_long']}" for split, value in manifest.items()))
