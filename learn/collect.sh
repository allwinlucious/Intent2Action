#!/bin/bash
# Sim data for the experiments, one stack at a time: stage-1 benchmark, stage-1 train, stage-2 expert episodes.
set -euo pipefail
cd "$(dirname "$0")/.."
PY=.venv-ml/bin/python
$PY learn/stage1.py --intents 300 --seed 2 --out stage1_bench
$PY learn/stage1.py --intents 2500 --seed 1 --out stage1_train
$PY learn/run.py --policy expert --split train --count 200 --out expert_train
echo COLLECT_DONE
