#!/usr/bin/env bash
# Serves base Muse Glimmer 30B with the three published adapters, straight from Hugging Face, on any GPU host with
# about 40 GB of GPU memory and CUDA >= 13 (RunPod, Modal, a workstation):
#   bash runpod/serve_from_hf.sh            # then: OpenAI-compatible API on http://127.0.0.1:8000/v1
# Model names: G0 (base), G1d (program writer and recovery), G1e (+ push experience), G1b (program writer).
# Point the panel at it from the robot host:  LLM_BASE_URL=http://<host>:8000/v1 LLM_API_KEY=unused make ui-fake
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
