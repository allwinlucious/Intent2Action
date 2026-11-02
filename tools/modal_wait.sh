#!/usr/bin/env bash
# Waits for a spawned training run (learn/modal_train.py) to write runs/<run>.done.json, then prints it.
#   tools/modal_wait.sh <run> [timeout_s]
set -euo pipefail
run="$1"; deadline=$(( $(date +%s) + ${2:-14400} ))
D="$(cd "$(dirname "$0")" && pwd)"
out="$(mktemp -d)"
until "$D/modal.sh" volume get show-once-train "runs/$run.done.json" "$out/" --force >/dev/null 2>&1; do
  [ "$(date +%s)" -lt "$deadline" ] || { echo "timeout waiting for runs/$run.done.json" >&2; exit 1; }
  sleep 60
done
python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print('ok' if d.get('ok') else 'FAILED', d.get('error','')); print(d.get('log_tail','')[-1500:]); print('---- sample ----'); print(d.get('sample',''))" "$out/$run.done.json"
python3 -c "import json,sys; sys.exit(0 if json.load(open(sys.argv[1])).get('ok') else 1)" "$out/$run.done.json"
