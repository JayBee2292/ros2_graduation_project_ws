#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

# slam_toolbox/Nav2 can misinterpret this variable when launched from a snap IDE.
unset SNAP_COMMON

MAP_YAML="$PWD/maps/go2/go2_map.yaml"
ODOM_PARAMS="$PWD/src/h753_can_odom/config/h753_can_odom.yaml"
UART_PORT="/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_0036002C3235511837333439-if02"
LAUNCH_NAVIGATION=false
for arg in "$@"; do
  case "$arg" in
    map:=*) MAP_YAML="${arg#map:=}" ;;
    odom_params:=*) ODOM_PARAMS="${arg#odom_params:=}" ;;
    uart_port:=*) UART_PORT="${arg#uart_port:=}" ;;
    launch_navigation:=true) LAUNCH_NAVIGATION=true ;;
    launch_navigation:=false) LAUNCH_NAVIGATION=false ;;
  esac
done

if [[ ! -f "$MAP_YAML" ]]; then
  echo "ERROR: Go2 map YAML not found: $MAP_YAML" >&2
  exit 1
fi

MAP_IMAGE="$(awk '$1 == "image:" {print $2; exit}' "$MAP_YAML")"
if [[ -z "$MAP_IMAGE" ]]; then
  echo "ERROR: image entry is missing from: $MAP_YAML" >&2
  exit 1
fi
if [[ "$MAP_IMAGE" != /* ]]; then
  MAP_IMAGE="$(dirname "$MAP_YAML")/$MAP_IMAGE"
fi
if [[ ! -f "$MAP_IMAGE" ]]; then
  echo "ERROR: occupancy image not found: $MAP_IMAGE" >&2
  exit 1
fi

if [[ ! -f "$ODOM_PARAMS" ]]; then
  echo "ERROR: odometry parameter file not found: $ODOM_PARAMS" >&2
  exit 1
fi

CAN_PORT="$(awk '$1 == "can_port:" {print $2; exit}' "$ODOM_PARAMS")"
if [[ -z "$CAN_PORT" || ! -e "$CAN_PORT" ]]; then
  echo "ERROR: CAN odometry device is not connected: ${CAN_PORT:-unknown}" >&2
  exit 1
fi
if [[ ! -e /dev/ttyUSB0 ]]; then
  echo "ERROR: YDLIDAR device /dev/ttyUSB0 is not connected." >&2
  exit 1
fi
if [[ -z "$UART_PORT" || ! -e "$UART_PORT" ]]; then
  echo "ERROR: STM UART device is not connected: ${UART_PORT:-unknown}" >&2
  exit 1
fi

for device in /dev/ttyUSB0 "$CAN_PORT" "$UART_PORT"; do
  if fuser "$device" >/dev/null 2>&1; then
    echo "ERROR: device is already in use: $device" >&2
    echo "Stop run_robot_modes.sh, run_xbox_uart_drive.sh, and old ROS launches first." >&2
    fuser -v "$device" >&2 || true
    exit 1
  fi
done

for package in joy nav2_amcl nav2_collision_monitor nav2_map_server; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "ERROR: required ROS package is not installed: $package" >&2
    exit 1
  fi
done

if [[ "$LAUNCH_NAVIGATION" == true ]]; then
  for package in nav2_bt_navigator nav2_controller nav2_planner; do
    if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
      echo "ERROR: required Nav2 package is not installed: $package" >&2
      exit 1
    fi
  done
fi

if ! ros2 pkg executables h753_can_odom | grep -q 'go2_manual_drive_node'; then
  echo "ERROR: go2_manual_drive_node is not installed in this workspace." >&2
  echo "Run: colcon build --symlink-install --packages-select h753_can_odom" >&2
  exit 1
fi

if [[ "$LAUNCH_NAVIGATION" == true ]]; then
  echo "Go2 AMCL + Nav2 goal drive (Xbox override + collision monitor)"
else
  echo "Go2 AMCL manual drive (Xbox deadman + collision monitor)"
fi
echo "  map: $MAP_YAML"
echo "  image: $MAP_IMAGE"
echo "  CAN odom: $CAN_PORT"
echo "  motor UART: $UART_PORT"
echo
echo "Controls:"
echo "  1. In RViz, set the current pose with '2D Pose Estimate'."
if [[ "$LAUNCH_NAVIGATION" == true ]]; then
  echo "  2. Wait for AMCL scan alignment, then set one 1-2 m Nav2 Goal."
  echo "  3. Hold Xbox LB for manual override; release LB to resume the goal."
  echo "  4. Xbox B cancels the goal and latches stop. Release LB, then press A."
  echo "  5. Ctrl+C stops the launch and sends a final UART stop command."
else
  echo "  2. Hold Xbox LB and move the sticks to drive. Releasing LB stops."
  echo "  3. Xbox B latches stop. Release LB, then press A to clear it."
  echo "  4. Ctrl+C stops the launch and sends a final UART stop command."
fi
echo
echo "CAUTION: first test with the robot lifted or in a clear open area."

exec ros2 launch h753_can_odom go2_amcl_manual_drive_bringup.launch.py "$@"
