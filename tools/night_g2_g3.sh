#!/usr/bin/env bash
# One-off chain for the night of 2026-11-01: G2 and G3 from sampling to held-out evaluation.
set -uo pipefail
cd "$(dirname "$0")/.."
export MODAL_BUDGET_USD=45  # owner-approved for this night
log() { echo "[$(date +%H:%M)] $*"; }

log "waiting for sampling"
while pgrep -f "glimmer_sample.py generate" >/dev/null; do sleep 30; done
set -a; source .env; set +a
.venv/bin/python learn/glimmer_sample.py generate --split train --n 150 --k 4 --solve logs/expert/train_all.jsonl \
  --out logs/samples/train_programs.jsonl > logs/samples/generate_retry.out 2>&1  # retries failed requests
tools/modal.sh app stop -y show-once-glimmer >/dev/null 2>&1  # the endpoint is not needed any more tonight
log "grading samples on the fleet"
.venv/bin/python learn/fleet.py --workers 8 -- learn/glimmer_sample.py run --in logs/samples/train_programs.jsonl \
  --out logs/samples/train_episodes.jsonl > logs/fleet/samples.out 2>&1
for v in G2 G3; do
  .venv/bin/python learn/sft_data.py build --version $v --solve logs/expert/train_all.jsonl --out logs/sft/$v
  .venv-ml/bin/python learn/render_rows.py --data logs/sft/$v
  tools/modal.sh run --detach learn/modal_train.py --data logs/sft/$v --run $v --budget-usd 4 | grep spawned
done
for v in G2 G3; do tools/modal_wait.sh $v 7200 | head -3; done
for v in G2 G3; do .venv/bin/python learn/offline_eval.py spawn --version $v | tail -1; done
for v in G2 G3; do .venv/bin/python learn/offline_eval.py collect --version $v | tail -1; done
for v in G2 G3; do
  .venv/bin/python learn/fleet.py --workers 8 -- learn/glimmer_sample.py run --in logs/eval/${v}_programs.jsonl \
    --out logs/eval/${v}_episodes.jsonl > logs/fleet/eval_$v.out 2>&1
done
log "done"
