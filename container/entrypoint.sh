#!/usr/bin/env bash
# Starts the Intrinsic services as plain processes (what k3s does in a normal deployment).
# If any of them exits, the container exits, so failures are never silent.
set -euo pipefail

BIN=/opt/intrinsic/bin
CFG=/opt/app/container/config
ZENOH="tcp/127.0.0.1:7447"

BACKEND="${ROBOT_BACKEND:-isaac}"
case "$BACKEND" in
  isaac|loopback|plan) ;;
  *) echo "ROBOT_BACKEND must be isaac, loopback or plan, got '$BACKEND'" >&2; exit 1 ;;
esac
GRASP_PLANNER="${GRASP_PLANNER:-on}"
case "$GRASP_PLANNER" in
  on|off) ;;
  *) echo "GRASP_PLANNER must be on or off, got '$GRASP_PLANNER'" >&2; exit 1 ;;
esac

run() {  # run <name> <command...>: start in the background, prefix every log line with the name
  local name="$1"; shift  # and append it to /var/lib/intrinsic/logs/<name>.log
  "$@" > >(python3 -u /opt/app/container/logpipe.py "$name") 2>&1 &
}

export ALLOWED_PUBSUB_IPv4=127.0.0.1  # Zenoh peer listeners bind loopback instead of 0.0.0.0
mkdir -p /var/lib/intrinsic/worlds
echo "robot backend: $BACKEND, grasp planner: $GRASP_PLANNER"

run zenohd       /opt/zenoh/zenohd -c "$CFG/zenohd.json5"  # listens on $ZENOH, kv_store/** storage
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
python3 -m robot_model.load_camera || { echo "[camera_loader] failed" >&2; exit 1; }
if [ "$BACKEND" = loopback ]; then
  run hwm        "$BIN/loopback_hardware_module_main" --module_config_file="$CFG/loopback_hwm.txtpb" --logger_connection_timeout=0s
  ICON_CFG="$CFG/icon_loopback.txtpb"
elif [ "$BACKEND" = isaac ]; then
  # Bazel's own ROS 2 (no setup sourced); the RMW is loaded relative to the working directory.
  run ros2_hwm   bash -c "cd $BIN/ros2_hwm_main.runfiles/_main && CYCLONEDDS_URI=file:///opt/app/container/dds/cyclonedds.xml \
                   exec $BIN/ros2_hwm_main --module_config_file=$CFG/ros2_hwm.txtpb --logger_connection_timeout=0s"
  ICON_CFG="$CFG/icon.txtpb"  # ICON must follow at once: the HWM's clock needs ICON within 1 minute
fi
if [ "$BACKEND" != plan ]; then
  run icon       "$BIN/generic_icon_machine_server" --config_pbtxt_file="$ICON_CFG" --non_realtime \
                   --icon_server_port=18003 --use_runtime_asset_fallback=false --server_name=robot \
                   --zenoh_router="$ZENOH" --resource_registry_service=localhost:18010
fi
if [ "$BACKEND" = isaac ] || [ "$BACKEND" = plan ]; then
  # The image's ROS 2 with Fast DDS, the vendor of Isaac's services (sourcing ROS under set -u aborts).
  FASTDDS="set +u; source /opt/ros/humble/setup.bash; source /opt/ros_ext/setup.bash; set -u;
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=/opt/app/container/dds/fastdds.xml exec python3"
  run scene_sync bash -c "$FASTDDS /opt/app/container/services/scene_sync.py --backend $BACKEND"
  if [ "$GRASP_PLANNER" = on ]; then
    # move_group + grasp planner on CycloneDDS, robot_service's vendor (actions and services do not cross vendors).
    run grasp_planner bash -c "set +u; source /opt/ros/humble/setup.bash; source /opt/ros_ext/setup.bash; set -u;
      RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///opt/app/container/dds/cyclonedds.xml \
      exec ros2 launch franka_grasp_config grasp_planner.launch.py"
  fi
  if [ "$BACKEND" = isaac ]; then
    run camera_sim bash -c "$FASTDDS /opt/app/container/services/camera_sim.py"
    run calibration "$BIN/calibration_service_main" --runtime_context_file="$CFG/calibration.runtime.pb" \
                     --zenoh_router="$ZENOH"
  fi
  # CycloneDDS for /clock (and the grasp planner later); the vendored ICON client is in /opt/app/vendor.
  run robot_service bash -c "set +u; source /opt/ros/humble/setup.bash; source /opt/ros_ext/setup.bash; set -u;
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///opt/app/container/dds/cyclonedds.xml \
    PYTHONPATH=/opt/app:/opt/app/gen:/opt/app/vendor:\$PYTHONPATH exec python3 /opt/app/container/services/robot_service.py --bind ${ROBOT_BIND:-127.0.0.1} --backend $BACKEND"
fi

wait -n
echo "a service exited; stopping the container" >&2
exit 1
