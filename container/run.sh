#!/usr/bin/env bash
# Runs the Intrinsic container on the host network (see container/README.md for ports).
set -euo pipefail
sg docker -c "docker rm -f intrinsic" >/dev/null 2>&1 || true
exec sg docker -c "docker run --name intrinsic --network host \
  --cap-add IPC_LOCK --cap-add SYS_NICE --ulimit memlock=-1:-1 \
  -v intrinsic-data:/var/lib/intrinsic intrinsic-core:dev"
