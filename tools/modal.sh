#!/usr/bin/env bash
# Runs the Modal CLI with this repo's credentials from .env (never committed; see .gitignore).
#
#   .env (owner-only, chmod 600):
#     MODAL_TOKEN_ID=ak-...       # from modal.com -> Settings -> API tokens
#     MODAL_TOKEN_SECRET=as-...   # shown once, next to the id, when the token is created
#
# Usage:
#   tools/modal.sh token info                    # check the credentials
#   tools/modal.sh run learn/modal_glimmer.py    # run a Modal app's local entrypoint (GPU work runs in the cloud)
#   tools/modal.sh volume ls show-once-glimmer   # list the persistent volume (model cache, adapters, logs)
#   tools/modal.sh app list                      # running and recent apps; `app stop <id>` stops one
#
# Costs accrue only while a function runs (GPU billed per second); the volume stores data between runs.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
[[ -f "$ROOT/.env" ]] || { echo "missing $ROOT/.env (see the header of this script)" >&2; exit 1; }
set -a
# shellcheck disable=SC1091
source "$ROOT/.env"
set +a
for var in MODAL_TOKEN_ID MODAL_TOKEN_SECRET; do
  [[ -n "${!var:-}" ]] || { echo "$var is empty in $ROOT/.env" >&2; exit 1; }
done
exec "$ROOT/.venv-ml/bin/modal" "$@"
