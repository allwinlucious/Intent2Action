#!/usr/bin/env bash
# Live-recovery evaluation: the docs-level programs on held-out tasks where they fail, each model version as the
# recovery agent, served by learn/chat_server.py on a pod through an SSH tunnel. One worker: the server answers one
# request at a time, so parallel workers only queue (and time out).
#   runpod/recovery_eval.sh <host> <port> <version>...
set -euo pipefail
host="$1"; port="$2"; shift 2
cd "$(dirname "$0")/.."
ssh -i "$HOME/.ssh/runpod_ed25519" -o ExitOnForwardFailure=yes -N -L 18800:127.0.0.1:8000 -p "$port" "root@$host" &
tunnel=$!
trap 'kill $tunnel' EXIT
until curl -sf http://127.0.0.1:18800/v1/models >/dev/null; do sleep 15; done
set -a; source .env; set +a
export LLM_BASE_URL=http://127.0.0.1:18800/v1 LLM_API_KEY=unused LLM_TIMEOUT_S=900
for version in "$@"; do
  .venv/bin/python learn/fleet.py --workers 1 --prefix rec --base-port 18400 -- learn/glimmer_sample.py run \
    --in logs/eval/RECOVER_programs.jsonl --out "logs/eval/REC_${version}_episodes.jsonl" --recover "$version" \
    > "logs/fleet/rec_${version}.out" 2>&1
done
