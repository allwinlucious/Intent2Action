#!/usr/bin/env bash
# Builds the Intrinsic Core services that go into the container.
set -euo pipefail

INTRINSIC_DIR="${INTRINSIC_DIR:-$HOME/intrinsic-core}"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

cd "$INTRINSIC_DIR"
# --override_module: GitHub's tinygltf v2.9.6 archive no longer matches the BCR checksum (see third_party/tinygltf).
bazel build --keep_going --jobs=12 --local_resources=memory=HOST_RAM*.7 \
  --override_module=tinygltf="$PROJECT_DIR/third_party/tinygltf" \
  //intrinsic_motion_planning/intrinsic/motion_planning/service:motion_planner_service_assetized_main \
  //intrinsic/world/binary:world_binary \
  //intrinsic/geometry/service:geometry_service \
  //intrinsic_control/intrinsic/icon/server:generic_icon_machine_server \
  //intrinsic_control/intrinsic/icon/hardware_modules/loopback:loopback_hardware_module_main \
  //intrinsic/storage/onprem_cas/cmd:localcas
