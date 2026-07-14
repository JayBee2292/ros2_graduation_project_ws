#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

LOG_DIR="$HOME/.ros/log/h753_odom_imu_compare"
mkdir -p "$LOG_DIR"
CSV_PATH="$LOG_DIR/compare_$(date +%Y%m%d_%H%M%S).csv"

cat <<EOF
[H753 encoder kinematics vs D435i IMU comparison]
CSV log: $CSV_PATH

Run this while robot mode 1 is active, then drive straight and rotate in place.
Reset accumulated values at any time from another terminal:
  ros2 service call /h753_odom_imu_compare/reset std_srvs/srv/Trigger '{}'

Current Jetson kernel does not expose /camera/camera/imu yet. Until the
RealSense HID driver issue is fixed, the terminal will show imu[missing].
EOF

exec ros2 launch h753_can_odom odom_imu_compare.launch.py \
  csv_path:="$CSV_PATH" \
  "$@"
