#!/usr/bin/env bash
# Runs ON a RunPod pod (CUDA >= 13 host): the model cookbook's Glimmer SFT environment, as in learn/modal_train.py's
# image (torch 2.12.1 cu130, Axolotl 0.19.0, cut-cross-entropy at Axolotl's commit; SDPA, no flash-attn build),
# plus TRL for DPO/GRPO. Then downloads Glimmer into /workspace/hf.
set -euo pipefail
export HF_HOME=/workspace/hf UV_LINK_MODE=copy
CCE="git+https://github.com/axolotl-ai-cloud/ml-cross-entropy.git@4dfa52212d0552805aec5a6a2176c0abbea277d2"
COOKBOOK="https://raw.githubusercontent.com/meta-models/meta-oss-cookbook/main/muse-glimmer/post-training/sft/axolotl"
cd /workspace
curl -LsSf https://astral.sh/uv/install.sh | sh >/dev/null  # the image's uv predates the cu130 torch backend
export PATH="$HOME/.local/bin:$PATH"
[ -d venv ] || uv venv --python 3.12 venv
source venv/bin/activate
uv pip install -q --torch-backend cu130 torch==2.12.1 torchvision==0.27.1
uv pip install -q axolotl==0.19.0
uv pip install -q "cut-cross-entropy[transformers] @ $CCE"
uv pip install -q "trl" pyyaml
curl -fsSL "$COOKBOOK/configs/muse-glimmer-30b-qlora.yaml" -o /workspace/cookbook-qlora.yaml
python - <<'PY'
import torch; print("torch", torch.__version__, "cuda", torch.version.cuda, "gpu", torch.cuda.get_device_name(0),
                    "capability", torch.cuda.get_device_capability(0))
PY
hf download meta-models/Muse-Glimmer-30B --quiet >/dev/null 2>&1 || huggingface-cli download meta-models/Muse-Glimmer-30B >/dev/null
echo "setup done"
