#!/usr/bin/env bash
# Serves base Muse Glimmer 30B with the three published adapters, straight from Hugging Face, on a GPU pod with about
# 40 GB of GPU memory, CUDA >= 13 and a writable /workspace (runpod/setup.sh installs there; e.g. a RunPod PyTorch pod):
#   bash runpod/serve_from_hf.sh            # then: OpenAI-compatible API on the pod's http://127.0.0.1:8000/v1
# Model names: G0 (base), G1d (program writer and recovery), G1e (+ push experience), G1b (program writer).
# The server listens on the pod's localhost only. From your machine: ssh -N -L 8000:127.0.0.1:8000 -p <port> root@<pod>,
# then: make ui-models (the demo panel with a simulated robot, asking these models).
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=${ROOT:-/workspace/train}
bash runpod/setup.sh                                   # environment + base model download (about 56 GB)
source /workspace/venv/bin/activate
export HF_HOME=/workspace/hf
for pair in G1b:mewbot97/Intent2Action-glimmer-programs G1d:mewbot97/Intent2Action-glimmer-recovery \
            G1e:mewbot97/Intent2Action-glimmer-push; do
  hf download "${pair#*:}" adapter_config.json adapter_model.safetensors --local-dir "$ROOT/runs/${pair%%:*}"
done
exec python learn/chat_server.py --root "$ROOT" --adapters G1b G1d G1e --port 8000
