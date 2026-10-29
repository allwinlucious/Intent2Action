#!/usr/bin/env bash
# Runs the Intrinsic container on the host network (see container/README.md for ports).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
sg docker -c "docker rm -f intrinsic" >/dev/null 2>&1 || true
exec sg docker -c "docker run --name intrinsic --network host \
  --cap-add IPC_LOCK --cap-add SYS_NICE --ulimit memlock=-1:-1 -e ROBOT_BACKEND=${ROBOT_BACKEND:-isaac} \
  -e GRASP_PLANNER=${GRASP_PLANNER:-on} \
  -v $ROOT/container/services:/opt/app/container/services:ro -v $ROOT/gen:/opt/app/gen:ro -v $ROOT/assets/objects:/opt/app/assets/objects:ro \
  -v $ROOT/sim/scenes/default.yaml:/opt/app/config/default.yaml:ro \
  -v intrinsic-data:/var/lib/intrinsic intrinsic-core:dev"
