#!/usr/bin/env bash
# Builds the Intrinsic Core services that go into the container, plus this repository's //ext code.
# Usage: build_intrinsic.sh [--test]   (--test also runs the //ext unit tests)
set -euo pipefail

INTRINSIC_DIR="${INTRINSIC_DIR:-$HOME/intrinsic-core}"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RUN_TESTS=false
[ "${1:-}" = "--test" ] && RUN_TESTS=true

# //ext is this repository's ext/, linked into the Intrinsic workspace and kept out of its git status.
[ -e "$INTRINSIC_DIR/ext" ] || ln -s "$PROJECT_DIR/ext" "$INTRINSIC_DIR/ext"
grep -qx "ext" "$INTRINSIC_DIR/.git/info/exclude" || echo "ext" >> "$INTRINSIC_DIR/.git/info/exclude"

cd "$INTRINSIC_DIR"
# --override_module: GitHub's tinygltf v2.9.6 archive no longer matches the BCR checksum (see third_party/tinygltf).
FLAGS=(--keep_going --jobs=12 --local_resources=memory=HOST_RAM*.7
  --override_module=tinygltf="$PROJECT_DIR/third_party/tinygltf")
bazel build "${FLAGS[@]}" \
  //intrinsic_motion_planning/intrinsic/motion_planning/service:motion_planner_service_assetized_main \
  //intrinsic/world/binary:world_binary \
  //intrinsic/geometry/service:geometry_service \
  //intrinsic_control/intrinsic/icon/server:generic_icon_machine_server \
  //intrinsic_control/intrinsic/icon/hardware_modules/loopback:loopback_hardware_module_main \
  //intrinsic/storage/onprem_cas/cmd:localcas \
  //intrinsic_perception/intrinsic/perception/calibration/services/v1:calibration_service_main
# hardware_module_binary depends on ICON's malloc_guard, visible only to Intrinsic's own packages,
# so //ext builds with visibility checks off (phase_3_notes.md, Part 1).
bazel build "${FLAGS[@]}" --nocheck_visibility //ext/ros2_hwm:ros2_hwm_main //ext/calib_poses:sample_poses
if $RUN_TESTS; then
  bazel test "${FLAGS[@]}" --test_output=errors --nocheck_visibility \
    //ext/ros2_hwm:ros2_hwm_logic_test //ext/calib_poses:sample_poses_test
fi
