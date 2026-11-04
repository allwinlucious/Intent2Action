"""The trained models on Modal: Muse Glimmer 30B + the three Intent2Action adapters, served as an OpenAI-compatible API.

  pip install modal && modal setup                     # once: a Modal account and token
  export I2A_API_KEY=$(openssl rand -hex 16)           # the endpoint is public: callers need this key
  modal deploy learn/modal_serve.py                    # prints the URL, https://<you>--intent2action-serve.modal.run
  LLM_BASE_URL=<url>/v1 LLM_API_KEY=$I2A_API_KEY make ui-models     # the demo panel, asking these models
  modal app stop intent2action                         # stop it (it also scales to zero after 10 idle minutes)

The first start downloads the base model (about 56 GB) into the Modal volume `intent2action-hf` and loads it
(10-20 minutes); later starts reuse the volume. Models: G0 (base), G1d (program writer and recovery), G1e (+ push
experience), G1b (program writer). One A100 80GB; transformers + PEFT on an NF4 base, as the evaluation ran.
"""

import os
from pathlib import Path
import subprocess

import modal

ROOT = Path(__file__).resolve().parent
PORT = 8000
ADAPTERS = {"G1d": "mewbot97/Intent2Action-glimmer-recovery", "G1e": "mewbot97/Intent2Action-glimmer-push",
            "G1b": "mewbot97/Intent2Action-glimmer-programs"}

app = modal.App("intent2action")
hf_cache = modal.Volume.from_name(os.environ.get("I2A_HF_VOLUME", "intent2action-hf"), create_if_missing=True)
image = (
    modal.Image.from_registry("nvidia/cuda:13.0.2-devel-ubuntu24.04", add_python="3.12")
    .pip_install("uv")
    .run_commands(  # the training environment (runpod/setup.sh, learn/modal_train.py): the adapters' exact libraries
        "uv pip install --system --torch-backend cu130 torch==2.12.1 torchvision==0.27.1",
        "uv pip install --system axolotl==0.19.0",
    )
    .env({"HF_HOME": "/hf"})
    .add_local_file(ROOT / "chat_server.py", "/app/chat_server.py")
)
# The endpoint is public: it is deployed only with a key from your shell (the container gets it as a secret).
if not os.environ.get("I2A_API_KEY"):
  raise SystemExit("set I2A_API_KEY first, e.g. export I2A_API_KEY=$(openssl rand -hex 16)")
key = modal.Secret.from_dict({"I2A_API_KEY": os.environ["I2A_API_KEY"]})


@app.function(image=image, gpu="A100-80GB", volumes={"/hf": hf_cache}, secrets=[key], timeout=24 * 3600,
              scaledown_window=600)
@modal.concurrent(max_inputs=8)
@modal.web_server(port=PORT, startup_timeout=40 * 60)
def serve() -> None:
  from huggingface_hub import snapshot_download
  for name, repo in ADAPTERS.items():
    snapshot_download(repo, local_dir=f"/hf/runs/{name}", allow_patterns=["adapter_config.json", "*.safetensors"])
  # chat_server loads the base model and the adapters (both cached in the volume after the first start) and listens;
  # it reads the key from I2A_API_KEY, so the key is not on a command line.
  subprocess.Popen(["python", "/app/chat_server.py", "--root", "/hf", "--adapters", *ADAPTERS, "--port", str(PORT),
                    "--host", "0.0.0.0"], cwd="/app")
