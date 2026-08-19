#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

MAP_YAML="$PWD/maps/go2/go2_map.yaml"
ODOM_PARAMS="$PWD/src/h753_can_odom/config/h753_can_odom.yaml"
LAUNCH_LIDAR=true
LAUNCH_ODOM=true
for arg in "$@"; do
  case "$arg" in
    map:=*) MAP_YAML="${arg#map:=}" ;;
    odom_params:=*) ODOM_PARAMS="${arg#odom_params:=}" ;;
    launch_lidar:=false) LAUNCH_LIDAR=false ;;
    launch_odom:=false) LAUNCH_ODOM=false ;;
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

if ! ros2 pkg prefix nav2_map_server >/dev/null 2>&1 || \
   ! ros2 pkg prefix nav2_amcl >/dev/null 2>&1; then
  echo "ERROR: nav2_map_server or nav2_amcl is not installed." >&2
  exit 1
fi

if [[ "$LAUNCH_LIDAR" == true ]]; then
  if [[ ! -e /dev/ttyUSB0 ]]; then
    echo "ERROR: YDLIDAR device /dev/ttyUSB0 is not connected." >&2
    echo "For a map-only test, add: launch_lidar:=false launch_odom:=false" >&2
    exit 1
  fi
  if fuser /dev/ttyUSB0 >/dev/null 2>&1; then
    echo "ERROR: /dev/ttyUSB0 is already in use. Stop the existing robot mode first." >&2
    fuser -v /dev/ttyUSB0 >&2 || true
    exit 1
  fi
fi

if [[ "$LAUNCH_ODOM" == true ]]; then
  if [[ ! -f "$ODOM_PARAMS" ]]; then
    echo "ERROR: odometry parameter file not found: $ODOM_PARAMS" >&2
    exit 1
  fi
  CAN_PORT="$(awk '$1 == "can_port:" {print $2; exit}' "$ODOM_PARAMS")"
  if [[ -z "$CAN_PORT" || ! -e "$CAN_PORT" ]]; then
    echo "ERROR: CAN odometry device is not connected: ${CAN_PORT:-unknown}" >&2
    echo "For a map-only test, add: launch_lidar:=false launch_odom:=false" >&2
    exit 1
  fi
  if fuser "$CAN_PORT" >/dev/null 2>&1; then
    echo "ERROR: CAN odometry device is already in use. Stop the existing robot mode first." >&2
    fuser -v "$CAN_PORT" >&2 || true
    exit 1
  fi
fi

echo "Go2 AMCL localization test (NO motor-command node)"
echo "  map: $MAP_YAML"
echo "  image: $MAP_IMAGE"
echo "After RViz opens, set the robot pose with '2D Pose Estimate'."

exec ros2 launch h753_can_odom go2_amcl_test_bringup.launch.py "$@"
