#!/usr/bin/env bash
# Runs the Intrinsic container on the host network (see container/README.md for ports).
#   CONTAINER_NAME (default intrinsic), ROBOT_BACKEND, GRASP_PLANNER as before.
#   PUBLISH_PORT=<p>: instead of the host network, the container gets its own network and only robot_service is
#   published on 127.0.0.1:<p> (several plan-tier containers side by side, learn/fleet.py; Isaac needs the host
#   network). Each named container has its own data volume.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NAME="${CONTAINER_NAME:-intrinsic}"
VOLUME="intrinsic-data"
[ "$NAME" = intrinsic ] || VOLUME="intrinsic-data-$NAME"
if [ -n "${PUBLISH_PORT:-}" ]; then
  NETWORK="-p 127.0.0.1:$PUBLISH_PORT:18100 -e ROBOT_BIND=0.0.0.0"
else
  NETWORK="--network host"
fi
sg docker -c "docker rm -f $NAME" >/dev/null 2>&1 || true
exec sg docker -c "docker run --name $NAME $NETWORK \
  --cap-add IPC_LOCK --cap-add SYS_NICE --ulimit memlock=-1:-1 -e ROBOT_BACKEND=${ROBOT_BACKEND:-isaac} \
  -e GRASP_PLANNER=${GRASP_PLANNER:-on} \
  -v $ROOT/container/entrypoint.sh:/opt/app/container/entrypoint.sh:ro \
  -v $ROOT/container/services:/opt/app/container/services:ro -v $ROOT/gen:/opt/app/gen:ro -v $ROOT/assets/objects:/opt/app/assets/objects:ro \
  -v $ROOT/sim/scenes/default.yaml:/opt/app/config/default.yaml:ro \
  -v $VOLUME:/var/lib/intrinsic intrinsic-core:dev"
