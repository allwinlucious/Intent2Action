#!/usr/bin/env bash
# Waits for a version's held-out replies on a pod, copies them back, grades them on the plan-tier fleet, prints the table.
#   runpod/pull_eval.sh <host> <port> <version> [compare versions...]
set -euo pipefail
host="$1"; port="$2"; version="$3"; shift 3
cd "$(dirname "$0")/.."
ssh_="ssh -i $HOME/.ssh/runpod_ed25519 -o ConnectTimeout=20 -p $port root@$host"
until [ "$($ssh_ "wc -l < /workspace/train/eval/${version}_replies.jsonl 2>/dev/null || echo 0")" -ge 250 ]; do sleep 60; done
scp -i "$HOME/.ssh/runpod_ed25519" -P "$port" -q "root@$host:/workspace/train/eval/${version}_replies.jsonl" "logs/eval/${version}_replies.jsonl"
.venv/bin/python learn/offline_eval.py programs --version "$version" --replies "logs/eval/${version}_replies.jsonl"
.venv/bin/python learn/fleet.py --workers 8 -- learn/glimmer_sample.py run --in "logs/eval/${version}_programs.jsonl" \
  --out "logs/eval/${version}_episodes.jsonl" > "logs/fleet/eval_${version}.out" 2>&1
.venv/bin/python learn/eval_table.py G0 "$@" "$version"
