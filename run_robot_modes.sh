#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

# CLion installed as a snap exports SNAP_COMMON into its terminal. The system
# slam_toolbox package mistakes that for a snap runtime and rewrites map paths.
unset SNAP_COMMON

LOG_DIR="$HOME/.ros/log/h753_robot_modes"
mkdir -p "$LOG_DIR"
JOY_LOG="$LOG_DIR/joy_$(date +%Y%m%d_%H%M%S).log"

ros2 run joy joy_node --ros-args \
  -p deadzone:=0.15 \
  -p autorepeat_rate:=20.0 \
  >"$JOY_LOG" 2>&1 &
JOY_PID=$!

cleanup() {
  if kill -0 "$JOY_PID" 2>/dev/null; then
    kill "$JOY_PID" 2>/dev/null || true
    wait "$JOY_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "joy_node log: $JOY_LOG"
echo "Xbox가 연결되지 않아도 키보드 숫자와 w/s/a/d/x로 기능을 확인할 수 있습니다."

ros2 run h753_can_odom robot_mode_manager_node --ros-args \
  --params-file "$PWD/install/h753_can_odom/share/h753_can_odom/config/h753_robot_mode_manager.yaml" \
  "$@"
