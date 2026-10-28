#!/usr/bin/env bash
# Copies the built Intrinsic binaries, their runfiles and the Bazel-built shared libraries they link
# into container/stage/, which the Dockerfile copies into the image.
set -euo pipefail

INTRINSIC_DIR="${INTRINSIC_DIR:-$HOME/intrinsic-core}"
STAGE="$(cd "$(dirname "$0")" && pwd)/stage"
BIN="$INTRINSIC_DIR/bazel-bin"
BINARIES=(
  intrinsic/geometry/service/geometry_service
  intrinsic/world/binary/world_binary
  intrinsic_motion_planning/intrinsic/motion_planning/service/motion_planner_service_assetized_main
  intrinsic_control/intrinsic/icon/server/generic_icon_machine_server
  intrinsic_control/intrinsic/icon/hardware_modules/loopback/loopback_hardware_module_main
  intrinsic/storage/onprem_cas/cmd/localcas
  intrinsic_perception/intrinsic/perception/calibration/services/v1/calibration_service_main
  ext/ros2_hwm/ros2_hwm_main
  ext/calib_poses/sample_poses
)

rm -rf "$STAGE" && mkdir -p "$STAGE/bin" "$STAGE/lib"
for b in "${BINARIES[@]}"; do
  name="$(basename "$b")"
  cp -L "$BIN/$b" "$STAGE/bin/$name"
  [ -d "$BIN/$b.runfiles" ] && cp -rL "$BIN/$b.runfiles" "$STAGE/bin/$name.runfiles"
  # Shared libraries that live in Bazel's output tree (not in the base image) go to lib/.
  ldd "$BIN/$b" 2>/dev/null | awk '/=> \// {print $3}' | grep "/.cache/bazel/" | xargs -r cp -L -t "$STAGE/lib/" || true
done

missing="$(for f in "$STAGE"/bin/*; do [ -f "$f" ] && LD_LIBRARY_PATH="$STAGE/lib" ldd "$f" 2>/dev/null | grep "not found" || true; done | sort -u)"
[ -z "$missing" ] || { echo "unresolved libraries:"; echo "$missing"; exit 1; }
du -sh "$STAGE"
