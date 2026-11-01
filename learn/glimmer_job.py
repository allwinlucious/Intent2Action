"""Glimmer training and generation jobs, independent of where they run (Modal: learn/modal_train.py; a RunPod pod:
runpod/). One source of truth for the cookbook-based config and the cookbook-style adapter evaluation.

Layout under `root` (Modal: /train on the volume; pod: /workspace/train):
  data/<name>/{train,validation}.jsonl   rendered rows (learn/render_rows.py)
  runs/<run>/ (adapter), runs/<run>.yaml, runs/<run>.log, runs/<run>.done.json

On a pod:  python glimmer_job.py train|generate ...   (see main)
"""

import argparse
import json
from pathlib import Path
import subprocess

MODEL = "meta-models/Muse-Glimmer-30B"


def config(root: str, data: str, run: str, cookbook_yaml: str, sequence_len: int = 4096,
           overrides: dict | None = None) -> str:
  """The cookbook's muse-glimmer-30b-qlora.yaml with our data; returns the written config path."""
  import yaml
  cfg = yaml.safe_load(Path(cookbook_yaml).read_text())
  cfg.pop("model_revision", None)
  # Rows are rendered with Glimmer's chat template in Python (learn/render_rows.py): the exact product prompt,
  # tool definitions included, as Axolotl's template-free segments (prompt masked, reply trained). Text only, so
  # the cookbook's multimodal processor and collator settings do not apply.
  for key in ("datasets", "test_datasets"):
    cfg[key] = [{"path": f"{root}/data/{data}/{'train' if key == 'datasets' else 'validation'}.jsonl",
                 "type": "input_output", **({"split": "train"} if key == "test_datasets" else {})}]
  for key in ("processor_type", "skip_prepare_dataset", "remove_unused_columns"):
    cfg.pop(key, None)
  # The same model class as the cookbook's multimodal path, so the LoRA target names (model.language_model...) match.
  cfg["model_type"] = "AutoModelForImageTextToText"
  cfg["attn_implementation"] = "sdpa"
  cfg["sequence_len"] = sequence_len  # 8192 for recovery rows (their prompt carries the robot's tool list)
  cfg.update(dataset_prepared_path=f"{root}/cache/{run}", output_dir=f"{root}/runs/{run}",
             wandb_project="", wandb_mode="disabled")
  cfg.update(overrides or {})
  path = Path(f"{root}/runs/{run}.yaml")
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(yaml.safe_dump(cfg, sort_keys=False))
  return str(path)


def train(root: str, data: str, run: str, cookbook_yaml: str, max_steps: int = 0, sequence_len: int = 4096,
          overrides: dict | None = None, launcher: list[str] | None = None) -> dict:
  """Axolotl on the config; raises with the log tail on failure. `launcher`: e.g. torchrun args for several GPUs."""
  path = config(root, data, run, cookbook_yaml, sequence_len, overrides)
  command = ["axolotl", "train", path]
  if max_steps:
    command += ["--max-steps", str(max_steps), "--eval-strategy", "no", "--save-strategy", "steps",
                "--save-steps", str(max_steps)]
  if launcher:
    command += ["--launcher", *launcher]
  log_path = Path(f"{root}/runs/{run}.log")
  with log_path.open("w") as log:
    process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, cwd=root)
  tail = log_path.read_text()[-4000:]
  if process.returncode != 0:
    raise RuntimeError(f"axolotl exited {process.returncode}:\n{tail}")
  return {"run": run, "log_tail": tail}


def _adapter(root: str, run: str) -> Path:
  run_dir = Path(f"{root}/runs/{run}")
  return run_dir if (run_dir / "adapter_config.json").exists() else sorted(run_dir.glob("checkpoint-*"))[-1]


def _model(adapter: Path | None):
  """The cookbook's --load-in-4bit base (precision-matched to QLoRA), plus the adapter if any."""
  import peft
  import torch
  import transformers
  quantization = transformers.BitsAndBytesConfig(
      load_in_4bit=True, bnb_4bit_quant_type="nf4", bnb_4bit_use_double_quant=True,
      bnb_4bit_compute_dtype=torch.bfloat16, bnb_4bit_quant_storage=torch.bfloat16)
  model = transformers.AutoModelForImageTextToText.from_pretrained(MODEL, device_map="cuda", dtype=torch.bfloat16,
                                                                   quantization_config=quantization)
  if adapter is not None:
    model = peft.PeftModel.from_pretrained(model, str(adapter), is_trainable=False)
  return model.eval()


def sample(root: str, data: str, run: str) -> str:
  """Base + adapter answer the first validation prompt: the adapter reloads and generates."""
  import transformers
  row = json.loads(Path(f"{root}/data/{data}/validation.jsonl").read_text().splitlines()[0])
  tokenizer = transformers.AutoProcessor.from_pretrained(MODEL).tokenizer
  model = _model(_adapter(root, run))
  inputs = tokenizer(row["segments"][0]["text"], return_tensors="pt", add_special_tokens=False).to("cuda")
  output = model.generate(**inputs, max_new_tokens=600, do_sample=False)
  return tokenizer.decode(output[0][inputs["input_ids"].shape[1]:], skip_special_tokens=False)


def reply_text(raw: str) -> str:
  """The first user-directed response (after any private reasoning turn), as the cookbook's evaluator records."""
  marker = "to=user<|message|>"
  text = raw.split(marker, 1)[1] if marker in raw else raw.split("<|message|>", 1)[-1]
  return text.split("<|eot|>", 1)[0].split("<|eom|>", 1)[0]


def generate(root: str, requests: str, out: str, version: str, batch: int = 8, max_new_tokens: int = 1200,
             temperature: float = 0.0, samples: int = 1, on_flush=None) -> None:
  """Generation the way the model cookbook evaluates adapters (post-training/sft/axolotl/scripts/evaluate.py):
  transformers + PEFT on a precision-matched NF4 base (G0 = no adapter), <|eot|> as a stop token; greedy unless
  `temperature` > 0 (then `samples` replies per request, keys suffixed #i). `requests`: JSON lines
  {"key", "messages", "tools"?} under root; writes {"key", "text", "raw"} lines to `out` (resumes)."""
  import torch
  import transformers
  processor = transformers.AutoProcessor.from_pretrained(MODEL)
  tokenizer = processor.tokenizer
  tokenizer.padding_side = "left"
  model = _model(None if version == "G0" else _adapter(root, version))
  stop = [tokenizer.eos_token_id, tokenizer.convert_tokens_to_ids("<|eot|>")]
  rows = [json.loads(line) for line in Path(f"{root}/{requests}").read_text().splitlines()]
  if samples > 1:
    rows = [{**row, "key": f"{row['key']}#{i}"} for row in rows for i in range(samples)]
  out_path = Path(f"{root}/{out}")
  done = {json.loads(line)["key"] for line in out_path.read_text().splitlines()} if out_path.exists() else set()
  rows = [row for row in rows if row["key"] not in done]
  sampling = {"do_sample": True, "temperature": temperature} if temperature > 0 else {"do_sample": False}
  with out_path.open("a") as handle:
    for start in range(0, len(rows), batch):
      chunk = rows[start:start + batch]
      prompts = [processor.apply_chat_template(row["messages"], tools=row.get("tools"), add_generation_prompt=True,
                                               tokenize=False, reasoning_strength="low") for row in chunk]
      inputs = tokenizer(prompts, return_tensors="pt", padding=True, add_special_tokens=False).to("cuda")
      with torch.no_grad():
        output = model.generate(**inputs, max_new_tokens=max_new_tokens, eos_token_id=stop,
                                pad_token_id=tokenizer.pad_token_id, **sampling)
      for row, tokens in zip(chunk, output[:, inputs["input_ids"].shape[1]:]):
        raw = tokenizer.decode(tokens, skip_special_tokens=False)
        handle.write(json.dumps({"key": row["key"], "text": reply_text(raw), "raw": raw}) + "\n")
      handle.flush()
      if on_flush:
        on_flush()
      print(f"{version}: {start + len(chunk)}/{len(rows)}", flush=True)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--root", default="/workspace/train")
  parser.add_argument("--cookbook", default="/workspace/cookbook-qlora.yaml")
  sub = parser.add_subparsers(dest="command", required=True)
  p = sub.add_parser("train")
  p.add_argument("--data", required=True)
  p.add_argument("--run", required=True)
  p.add_argument("--max-steps", type=int, default=0)
  p.add_argument("--sequence-len", type=int, default=4096)
  p.add_argument("--overrides", default="{}", help="JSON merged into the config (e.g. seed, adapter, load_in_4bit)")
  p.add_argument("--sample", action="store_true", help="also generate once from the adapter")
  p = sub.add_parser("generate")
  p.add_argument("--requests", required=True)
  p.add_argument("--out", required=True)
  p.add_argument("--version", required=True)
  p.add_argument("--batch", type=int, default=8)
  p.add_argument("--temperature", type=float, default=0.0)
  p.add_argument("--samples", type=int, default=1)
  args = parser.parse_args()
  if args.command == "train":
    Path(f"{args.root}/runs").mkdir(parents=True, exist_ok=True)
    status = {"run": args.run}
    try:
      status["log_tail"] = train(args.root, args.data, args.run, args.cookbook, args.max_steps, args.sequence_len,
                                 json.loads(args.overrides))["log_tail"][-3000:]
      if args.sample:
        status["sample"] = sample(args.root, args.data, args.run)
      status["ok"] = True
    except Exception as error:  # pylint: disable=broad-except
      status.update(ok=False, error=f"{type(error).__name__}: {error}"[-3000:])
    Path(f"{args.root}/runs/{args.run}.done.json").write_text(json.dumps(status, indent=1))
    print(json.dumps({k: v for k, v in status.items() if k != "log_tail"})[:2000])
  else:
    generate(args.root, args.requests, args.out, args.version, args.batch, temperature=args.temperature,
             samples=args.samples)


if __name__ == "__main__":
  main()
