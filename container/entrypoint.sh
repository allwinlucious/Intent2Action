#!/usr/bin/env bash
# Starts the Intrinsic services as plain processes (what k3s does in a normal deployment).
# If any of them exits, the container exits, so failures are never silent.
set -euo pipefail

BIN=/opt/intrinsic/bin
CFG=/opt/app/container/config
ZENOH="tcp/127.0.0.1:7447"

run() {  # run <name> <command...>: start in the background, prefix every log line with the name
  local name="$1"; shift
  "$@" > >(sed -u "s/^/[$name] /") 2>&1 &
}

run zenohd       /opt/zenoh/zenohd --listen "$ZENOH" --no-multicast-scouting
run localcas     "$BIN/localcas" --port=18020 --objects_dir=/var/lib/intrinsic/cas --partial_dir=/var/lib/intrinsic/cas-partial
run stubs        python3 /opt/app/container/services/platform_stubs.py --port=18010
sleep 2
run geometry     "$BIN/geometry_service" --port=18001 --cas_service_address=localhost:18020
run world        "$BIN/world_binary" --port=18000 --resource_registry_address=localhost:18010 \
                   --cas_service_address=localhost:18020 --zenoh_router="$ZENOH"
sleep 5
run planner      "$BIN/motion_planner_service_assetized_main" --runtime_context_file="$CFG/motion_planner.runtime.pb" \
                   --world_service_address=localhost:18000 --geometry_service_address=localhost:18001 \
                   --zenoh_router="$ZENOH"

wait -n
echo "a service exited; stopping the container" >&2
exit 1
