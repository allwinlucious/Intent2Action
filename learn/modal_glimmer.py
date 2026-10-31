"""Muse Glimmer on Modal: an OpenAI-compatible endpoint (vLLM) for the program writer and the recovery agent.

  tools/modal.sh deploy learn/modal_glimmer.py     # deploys; prints the URL; scales to zero after 10 idle minutes
  tools/modal.sh app stop show-once-glimmer        # stops it

Follows the model cookbook's vLLM page: the vllm/vllm-openai:muse-glimmer image (Glimmer support is not in a
released vLLM wheel yet), the muse_glimmer tool-call and reasoning parsers, the checkpoint's own chat template.
The weights (~60 GB) are cached on the volume, so only the first start downloads them. Fine-tuned adapters
(learn/modal_train.py) are served as extra model names: `"model": "G1"` etc. The endpoint is public, so
vLLM requires an API key: Modal secret `show-once-llm` (VLLM_API_KEY), the same value as LLM_API_KEY in .env.
"""

import subprocess

import modal

MODEL = "meta-models/Muse-Glimmer-30B"
SERVED_NAME = "muse-glimmer"
PORT = 8000
MAX_MODEL_LEN = 32768  # our longest prompts (recovery with the robot tool schemas) are ~8k tokens

app = modal.App("show-once-glimmer")
cache = modal.Volume.from_name("show-once-glimmer", create_if_missing=True)
adapters = modal.Volume.from_name("show-once-train", create_if_missing=True)  # learn/modal_train.py's runs/
# The image has only python3; Modal looks for `python` to detect the version.
image = (modal.Image.from_registry("vllm/vllm-openai:muse-glimmer")
         .dockerfile_commands(["RUN ln -sf \"$(command -v python3)\" /usr/local/bin/python"]).entrypoint([]))


@app.function(
    image=image,
    gpu="A100-80GB",  # bf16 weights (~60 GB) + KV cache; $2.50/h vs H100 $3.95/h (project budget $30 total)
    volumes={"/root/.cache/huggingface": cache, "/train": adapters},
    secrets=[modal.Secret.from_name("show-once-llm")],
    timeout=60 * 60,
    scaledown_window=10 * 60,
)
@modal.concurrent(max_inputs=8)
@modal.web_server(port=PORT, startup_timeout=30 * 60)
def serve() -> None:
  import os
  from pathlib import Path

  # Every trained adapter (runs/<version>/adapter_config.json) is served under its version name next to the base
  # model (G0 = SERVED_NAME): one endpoint, so every version runs the identical evaluation.
  lora = [f"{d.name}={d}" for d in sorted(Path("/train/runs").glob("*")) if (d / "adapter_config.json").exists()]
  # Eager mode with adapters: vLLM's LoRA kernel asserted during CUDA-graph capture with Glimmer
  # (token_lora_mapping size mismatch in lora_shrink, engine start-up).
  lora_args = ["--enable-lora", "--max-lora-rank", "32", "--max-loras", "4", "--enforce-eager",
               "--lora-modules", *lora] if lora else []

  subprocess.Popen([
      "vllm", "serve", MODEL,
      "--served-model-name", SERVED_NAME,
      "--tensor-parallel-size", "1",
      "--enable-auto-tool-choice",
      "--tool-call-parser", "muse_glimmer",
      "--reasoning-parser", "muse_glimmer",
      "--generation-config", "auto",
      "--max-model-len", str(MAX_MODEL_LEN),
      "--host", "0.0.0.0", "--port", str(PORT),
      "--api-key", os.environ["VLLM_API_KEY"],
      *lora_args,
  ])
