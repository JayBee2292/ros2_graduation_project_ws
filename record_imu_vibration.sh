#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

LABEL="${1:-baseline}"
SAFE_LABEL="$(printf '%s' "$LABEL" | tr -cs '[:alnum:]_-' '_')"
BAG_ROOT="$HOME/.ros/bag/h753_imu_vibration"
BAG_PATH="$BAG_ROOT/$(date +%Y%m%d_%H%M%S)_$SAFE_LABEL"
mkdir -p "$BAG_ROOT"

# Keep trials mutually exclusive. Running several recorders at once records
# overlapping data under different labels and can starve the drive safety path.
LOCK_FILE="$BAG_ROOT/record.lock"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  printf '%s\n' \
    'Another IMU vibration recorder is already running.' \
    'Stop that trial with Ctrl+C before starting the next label.' >&2
  exit 1
fi
printf '%s\n' "$$" >&9

printf '%s\n' \
  '[H753 IMU vibration recording]' \
  "Label: $LABEL" \
  "Output: $BAG_PATH" \
  '' \
  'Recommended separate recordings:' \
  '  stationary, straight_slow, straight_fast' \
  '  rotate_cw_020, rotate_ccw_020' \
  '  rotate_cw_035, rotate_ccw_035' \
  '  rotate_cw_050, rotate_ccw_050' \
  '' \
  'Record only one trial at a time.' \
  'Stop recording with Ctrl+C.'

exec ros2 bag record \
  --output "$BAG_PATH" \
  /camera/camera/imu \
  /wheel/odom \
  /odom \
  /cmd_vel_selected \
  /imu_filter/yaw_rate_raw \
  /imu_filter/yaw_rate_filtered \
  /imu_filter/vibration_variance \
  /imu_filter/imu_weight
