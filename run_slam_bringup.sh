#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

# Avoid slam_toolbox treating a CLion snap terminal as its own snap runtime.
unset SNAP_COMMON

for arg in "$@"; do
  if [[ "$arg" == "launch_lidar:=false" ]]; then
    exec ros2 launch h753_can_odom slam_bringup.launch.py "$@"
  fi
done

if fuser /dev/ttyUSB0 >/dev/null 2>&1; then
  echo "ERROR: /dev/ttyUSB0 is already in use. Stop the existing YDLIDAR/ROS launch first." >&2
  fuser -v /dev/ttyUSB0 >&2 || true
  exit 1
fi

exec ros2 launch h753_can_odom slam_bringup.launch.py "$@"
