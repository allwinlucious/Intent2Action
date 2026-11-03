#!/usr/bin/env bash
# Copies the ICON Python client modules robot_service uses into container/stage/vendor/ (phase_3.md
# decision 1), then imports each one with only the vendor tree and gen/ on the path.
# Run after stage_intrinsic.sh (which recreates container/stage/).
set -euo pipefail

INTRINSIC_DIR="${INTRINSIC_DIR:-$HOME/intrinsic-core}"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VENDOR="$PROJECT_DIR/container/stage/vendor"
SRC="$INTRINSIC_DIR/intrinsic_control/intrinsic/icon"
MODULES=(
  python/_session.py
  python/actions.py
  python/reactions.py
  python/errors.py
  python/state_variable_path.py
  actions/point_to_point_move_utils.py
  actions/trajectory_tracking_action_utils.py
  actions/stop_utils.py
  actions/tare_force_torque_sensor_utils.py
)

rm -rf "$VENDOR"
for m in "${MODULES[@]}"; do
  mkdir -p "$VENDOR/intrinsic/icon/$(dirname "$m")"
  cp "$SRC/$m" "$VENDOR/intrinsic/icon/$m"
done
for m in "${MODULES[@]}"; do
  module="intrinsic.icon.$(echo "${m%.py}" | tr / .)"
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$VENDOR:$PROJECT_DIR/gen" "${PYTHON:-$PROJECT_DIR/.venv/bin/python}" -c "import $module" \
    || { echo "import $module failed"; exit 1; }
done
echo "vendored ${#MODULES[@]} modules into $VENDOR"
