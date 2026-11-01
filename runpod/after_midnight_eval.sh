#!/usr/bin/env bash
# The owner's PC is free from 00:00: then, for each version whose held-out replies are on the pod, copy them back,
# grade them 3x on the plan-tier fleet, and run its programs on the 50 Isaac tasks G1b was measured on.
#   runpod/after_midnight_eval.sh <host> <port> <version>...
set -uo pipefail
host="$1"; port="$2"; shift 2
cd "$(dirname "$0")/.."
[ -n "${NOW:-}" ] || until [ "$(date +%H)" -lt 8 ]; do sleep 300; done  # 00:00-07:59 the PC is free (NOW=1: start at once)
ssh_="ssh -i $HOME/.ssh/runpod_ed25519 -o ConnectTimeout=20 -p $port root@$host"
for version in "$@"; do
  until [ "$($ssh_ "wc -l < /workspace/train/eval/${version}_replies.jsonl 2>/dev/null || echo 0")" -ge 250 ]; do sleep 120; done
  scp -i "$HOME/.ssh/runpod_ed25519" -P "$port" -q "root@$host:/workspace/train/eval/${version}_replies.jsonl" logs/eval/
  .venv/bin/python learn/offline_eval.py programs --version "$version" --replies "logs/eval/${version}_replies.jsonl"
  .venv/bin/python learn/offline_eval.py repeat --version "$version" --times 3
  .venv/bin/python learn/fleet.py --workers 8 -- learn/glimmer_sample.py run --in "logs/eval/${version}_programs_r3.jsonl" \
    --out "logs/eval/${version}_episodes_r3.jsonl" > "logs/fleet/eval_${version}_r3.out" 2>&1
  timeout 10800 .venv/bin/python learn/isaac_eval.py run --version "$version" --per-family 10 > "logs/eval/${version}_isaac.out" 2>&1
done
.venv/bin/python learn/eval_table.py --repeat 3 DOCS G0 G1b "$@"
for version in G1b "$@"; do .venv/bin/python learn/isaac_eval.py agreement --version "$version" 2>/dev/null | head -3; done
