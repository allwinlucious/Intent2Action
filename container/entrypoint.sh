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

export ALLOWED_PUBSUB_IPv4=127.0.0.1  # Zenoh peer listeners bind loopback instead of 0.0.0.0
mkdir -p /var/lib/intrinsic/worlds

run zenohd       /opt/zenoh/zenohd --listen "$ZENOH" --no-multicast-scouting
run localcas     "$BIN/localcas" --port=18020 --objects_dir=/var/lib/intrinsic/cas --partial_dir=/var/lib/intrinsic/cas-partial
run stubs        python3 /opt/app/container/services/platform_stubs.py --port=18010
sleep 2
run geometry     "$BIN/geometry_service" --port=18001 --cas_service_address=localhost:18020
run world        "$BIN/world_binary" --port=18000 --conductor_port=18009 --resource_registry_address=localhost:18010 \
                   --cas_service_address=localhost:18020 --zenoh_router="$ZENOH" --worlds_path=/var/lib/intrinsic/worlds
sleep 5
run planner      "$BIN/motion_planner_service_assetized_main" --runtime_context_file="$CFG/motion_planner.runtime.pb" \
                   --world_service_address=localhost:18000 --geometry_service_address=localhost:18001 \
                   --zenoh_router="$ZENOH"
python3 -m robot_model.load_franka || { echo "[loader] failed" >&2; exit 1; }
run hwm          "$BIN/loopback_hardware_module_main" --module_config_file="$CFG/loopback_hwm.txtpb" --logger_connection_timeout=0s
run icon         "$BIN/generic_icon_machine_server" --config_pbtxt_file="$CFG/icon.txtpb" --non_realtime \
                   --icon_server_port=18003 --use_runtime_asset_fallback=false --server_name=robot \
                   --zenoh_router="$ZENOH" --resource_registry_service=localhost:18010

wait -n
echo "a service exited; stopping the container" >&2
exit 1
