# Reproducing the experiments

Every stage writes to `logs/` (git-ignored). `results/` holds the published copy of the data behind every number
(`python tools/results.py restore` unpacks it into `logs/`), so later stages can start from it without rerunning
the earlier ones. All commands run from the repository root.

## 0. Environments

```bash
make setup                                     # .venv: runtime, tests, plan tier, evaluation (requirements-dev.txt)
python3 -m venv .venv-ml && .venv-ml/bin/pip install -r requirements-ml.txt   # local model work (optional)
```

*   **GPU work** (training, generation, serving Glimmer) runs on a pod with CUDA ≥ 13:
    *   the setup is `runpod/setup.sh`, run on the pod;
    *   we used an RTX PRO 6000 (96 GB) on RunPod;
    *   copy `learn/` and the data folders to the pod's `/workspace/train`.
*   **The Meta Model API** (hosted Muse models, speech-to-text) needs `META_API_KEY` in `.env`. `.env` is
    git-ignored; never commit it.

## 1. The robot stack

*   **Intrinsic Core:** this project builds against the public commit
    [`895281d`](https://github.com/intrinsic-ai/intrinsic-core/commit/895281d62b950ca46d21885906796848692b9638)
    (`make vendor` and CI use it).
*   The author's local build also carried one upstream-style fix, `eigenmath`'s `SO3::FromMatrix` rejecting
    reflections. It is not required by this project; it is in `container/patches/` (`git am` it in the Intrinsic
    Core checkout to build the identical stack).

```bash
container/build_intrinsic.sh && container/stage_intrinsic.sh             # Intrinsic Core from source
sg docker -c "docker build -f container/Dockerfile -t intrinsic-core:dev ."
ROBOT_BACKEND=plan container/run.sh                                     # one plan-tier container (no physics)
```

*   **Physics:** Isaac Sim 5.1 on the Windows host:
    *   `tools/sync_isaac.sh` copies `sim/`, `config/` and `assets/` across;
    *   the e2e checks (`tests/e2e/pickplace_check.py`, `tests/e2e/program_check.py [--gui]`) start Isaac and the
        container and verify the stack.
*   **The evaluation fleet:** `learn/fleet.py` runs N plan-tier containers side by side (about 1 core and 0.6 GB
    each).

## 2. Task families and expert labels

```bash
.venv/bin/python learn/fleet.py --workers 8 -- learn/search_expert.py solve --split train --n 150 \
    --out logs/expert/train_all_sorting.jsonl
.venv/bin/python learn/fleet.py --workers 8 -- learn/search_expert.py solve --split heldout --n 50 \
    --out logs/expert/heldout_all.jsonl
```

*   **The families** (`learn/families.py`): cluster, packing, stacking, reach and sorting are trained; `mixed` is
    held out. The seeds are deterministic: train 0–9999, held-out 100000 and above.
*   **The expert** runs every strategy on every task. The label is the most reliable strategy that succeeded.

## 3. Training sets and fine-tuning

```bash
.venv/bin/python learn/sft_data.py build --version G1 --solve logs/expert/train_all_sorting.jsonl --out logs/sft/G1b
.venv/bin/python learn/sft_data.py build --version G3b --solve logs/expert/train_all_sorting.jsonl \
    --recovery logs/expert/recovery_best.jsonl --out logs/sft/G1d
# on the pod:
python learn/glimmer_job.py train --data G1b --run G1b      # the cookbook's QLoRA recipe (r 32, alpha 16)
python learn/glimmer_job.py train --data G1d --run G1d
```

*   The published training sets come with their manifests: `results/sft/G1b`, `results/sft/G1d`.
*   **DPO:** `learn/dpo_data.py` then `learn/dpo_train.py`.
*   **GRPO:** `learn/grpo_data.py` then `learn/grpo_train.py`.
*   **The trained adapters:**
    [G1b](https://huggingface.co/mewbot97/show-once-glimmer-programs) and
    [G1d](https://huggingface.co/mewbot97/show-once-glimmer-recovery).

## 4. Held-out evaluation (planning simulator, 3 runs)

```bash
.venv/bin/python learn/offline_eval.py requests --solve logs/expert/heldout_all.jsonl --out logs/eval/requests.jsonl
# generation on the pod: greedy, one attempt, the product's writer prompt
python learn/glimmer_job.py generate --requests eval/requests.jsonl --out eval/G1d_replies.jsonl --version G1d
.venv/bin/python learn/offline_eval.py programs --version G1d --replies logs/eval/G1d_replies.jsonl
.venv/bin/python learn/offline_eval.py repeat --version G1d --times 3
.venv/bin/python learn/fleet.py --workers 8 -- learn/glimmer_sample.py run \
    --in logs/eval/G1d_programs_r3.jsonl --out logs/eval/G1d_episodes_r3.jsonl
.venv/bin/python learn/eval_table.py --repeat 3 G0 G1b G1d
```

*   **Base Glimmer** is `G0`: the same generation with the adapter disabled.
*   **Hosted models** (Meta Model API) go through the same requests:

    ```bash
    .venv/bin/python learn/api_replies.py --model muse-spark-1.3 --version SPARK13
    ```

    then `programs`, `repeat` and the fleet as above. Spend is logged and capped (`logs/eval/api_spend.json`).

## 5. Physics, recovery, demo

```bash
.venv/bin/python learn/isaac_eval.py run --version G1d --per-family 10      # the 50 Isaac tasks, physics only
runpod/recovery_eval.sh <pod-host> <ssh-port> G0 G1d                         # live recovery, 20 tasks
.venv/bin/python tests/e2e/demo_stack.py                                     # Isaac viewer + container
.venv/bin/python -m demo_ui.server                                           # the panel; ?panel docks beside Isaac
```

*   **The panel's models** come from an OpenAI-compatible endpoint: `LLM_BASE_URL`, `LLM_MODEL` and `LLM_API_KEY`.
    `learn/chat_server.py` serves base Glimmer plus the adapters on a GPU pod.
*   **For a free local rehearsal,** `learn/qwen_server.py` serves Qwen3-8B (text only).

## Rebuilding the published numbers

```bash
make tables        # restores results/ into logs/ and prints the main tables
```
