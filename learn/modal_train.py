"""Glimmer QLoRA SFT on Modal with the model cookbook's Axolotl recipe (one H100).

  tools/modal.sh run learn/modal_train.py --data logs/sft/smoke --run smoke --max-steps 4
      uploads <data>/rendered/{train,validation}.jsonl (learn/render_rows.py), trains, saves the adapter on the volume `show-once-train`
      under runs/<run>/, and generates one reply with it from the first validation prompt.

The environment, versions and hyperparameters are the cookbook's `muse-glimmer-30b-qlora.yaml` and setup (Axolotl
0.19.0, torch 2.12.1 cu130, cut-cross-entropy at Axolotl's commit). Differences, all deliberate:
- one H100 instead of eight (the cookbook verified QLoRA on one H100). The project's Modal budget is $30 in total
  (tools/modal_budget.sh); an H100 costs 2x an L40S per hour but is ~3-4x faster (memory bandwidth and bf16
  compute), so a run costs less on it. The smoke run fit an L40S too (21 GiB peak);
- PyTorch SDPA attention instead of flash-attn 2.8.3: no CUDA extension build (an hour of builder time) and no
  GPU-architecture pin;
- our data paths, W&B off (the loss history is in the run log), no `model_revision`: the cached main revision has
  the same weights, config and template as the cookbook's pinned a4e59da (identical file hashes).
"""

import json
import subprocess
from pathlib import Path

import modal

MODEL = "meta-models/Muse-Glimmer-30B"
COOKBOOK = "https://raw.githubusercontent.com/meta-models/meta-oss-cookbook/main/muse-glimmer/post-training/sft/axolotl"
CCE = "git+https://github.com/axolotl-ai-cloud/ml-cross-entropy.git@4dfa52212d0552805aec5a6a2176c0abbea277d2"
GPU = "H100"

app = modal.App("show-once-train")
hf_cache = modal.Volume.from_name("show-once-glimmer", create_if_missing=True)
train_volume = modal.Volume.from_name("show-once-train", create_if_missing=True)
image = (
    modal.Image.from_registry("nvidia/cuda:13.0.2-devel-ubuntu24.04", add_python="3.12")
    .apt_install("git", "build-essential", "curl")
    .pip_install("uv")
    .run_commands(
        "uv pip install --system --torch-backend cu130 torch==2.12.1 torchvision==0.27.1",
        "uv pip install --system axolotl==0.19.0",
        f"uv pip install --system 'cut-cross-entropy[transformers] @ {CCE}'",
        f"curl -fsSL {COOKBOOK}/configs/muse-glimmer-30b-qlora.yaml -o /opt/cookbook-qlora.yaml",
    )
)


def _config(data: str, run: str) -> str:
  import yaml
  config = yaml.safe_load(Path("/opt/cookbook-qlora.yaml").read_text())
  config.pop("model_revision", None)
  # Rows are rendered with Glimmer's chat template in Python (learn/render_rows.py): the exact product prompt,
  # tool definitions included, as Axolotl's template-free segments (prompt masked, reply trained). Text only, so
  # the cookbook's multimodal processor and collator settings do not apply.
  for key in ("datasets", "test_datasets"):
    config[key] = [{"path": f"/train/data/{data}/{'train' if key == 'datasets' else 'validation'}.jsonl",
                    "type": "input_output", **({"split": "train"} if key == "test_datasets" else {})}]
  for key in ("processor_type", "skip_prepare_dataset", "remove_unused_columns"):
    config.pop(key, None)
  # The same model class as the cookbook's multimodal path, so the LoRA target names (model.language_model...) match.
  config["model_type"] = "AutoModelForImageTextToText"
  config["attn_implementation"] = "sdpa"
  config.update(dataset_prepared_path=f"/train/cache/{run}", output_dir=f"/train/runs/{run}",
                wandb_project="", wandb_mode="disabled")
  path = Path(f"/train/runs/{run}.yaml")
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(yaml.safe_dump(config, sort_keys=False))
  return str(path)


@app.function(image=image, gpu=GPU, volumes={"/root/.cache/huggingface": hf_cache, "/train": train_volume},
              timeout=6 * 60 * 60)
def train(data: str, run: str, max_steps: int = 0) -> dict:
  config = _config(data, run)
  command = ["axolotl", "train", config]
  if max_steps:
    command += ["--max-steps", str(max_steps), "--eval-strategy", "no", "--save-strategy", "steps",
                "--save-steps", str(max_steps)]
  log_path = Path(f"/train/runs/{run}.log")
  with log_path.open("w") as log:
    process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, cwd="/train")
  train_volume.commit()
  tail = log_path.read_text()[-4000:]
  if process.returncode != 0:
    raise RuntimeError(f"axolotl exited {process.returncode}:\n{tail}")
  return {"run": run, "log_tail": tail}


@app.function(image=image, gpu=GPU, volumes={"/root/.cache/huggingface": hf_cache, "/train": train_volume},
              timeout=60 * 60)
def sample(data: str, run: str) -> str:
  """Loads base + adapter (4-bit, as trained) and answers the first validation prompt: the adapter reloads."""
  import peft
  import torch
  import transformers
  adapter = sorted(Path(f"/train/runs/{run}").glob("checkpoint-*"))[-1] if not (
      Path(f"/train/runs/{run}") / "adapter_config.json").exists() else Path(f"/train/runs/{run}")
  row = json.loads(Path(f"/train/data/{data}/validation.jsonl").read_text().splitlines()[0])
  prompt_text = row["segments"][0]["text"]
  processor = transformers.AutoProcessor.from_pretrained(MODEL)
  model = transformers.AutoModelForImageTextToText.from_pretrained(
      MODEL, device_map="cuda", quantization_config=transformers.BitsAndBytesConfig(
          load_in_4bit=True, bnb_4bit_compute_dtype=torch.bfloat16, bnb_4bit_quant_type="nf4"))
  model = peft.PeftModel.from_pretrained(model, str(adapter))
  inputs = processor.tokenizer(prompt_text, return_tensors="pt", add_special_tokens=False).to("cuda")
  output = model.generate(**inputs, max_new_tokens=600, do_sample=False)
  return processor.decode(output[0][inputs["input_ids"].shape[1]:], skip_special_tokens=False)


@app.local_entrypoint()
def main(data: str, run: str, max_steps: int = 0, budget_usd: float = 3.0):
  subprocess.run(["tools/modal_budget.sh", str(budget_usd)], check=True)  # refuses when the $30 cap is near
  name = Path(data).name
  with train_volume.batch_upload(force=True) as batch:
    for split in ("train", "validation"):
      batch.put_file(f"{data}/rendered/{split}.jsonl", f"/data/{name}/{split}.jsonl")
  result = train.remote(name, run, max_steps)
  print(result["log_tail"][-1500:])
  print("---- sample from the adapter ----")
  print(sample.remote(name, run))
