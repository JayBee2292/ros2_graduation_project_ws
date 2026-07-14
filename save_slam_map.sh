#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

MAP_BASE="$(realpath -m "${1:-$PWD/maps/h753_map}")"
mkdir -p "$(dirname "$MAP_BASE")"

if ! ros2 service type /slam_toolbox/serialize_map >/dev/null 2>&1; then
  echo "ERROR: /slam_toolbox/serialize_map is not available." >&2
  echo "Start mapping first: $PWD/run_slam_bringup.sh" >&2
  exit 1
fi

if ! ros2 service type /slam_toolbox/save_map >/dev/null 2>&1; then
  echo "ERROR: /slam_toolbox/save_map is not available." >&2
  exit 1
fi

echo "Saving slam_toolbox posegraph: $MAP_BASE.posegraph / $MAP_BASE.data"
SERIALIZE_OUTPUT="$(
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$MAP_BASE'}"
)"
echo "$SERIALIZE_OUTPUT"

if [[ "$SERIALIZE_OUTPUT" != *"result=0"* && -n "${SNAP_COMMON:-}" ]]; then
  # Recover sessions started from a CLion snap terminal before SNAP_COMMON was
  # filtered by the launch wrappers. slam_toolbox prepends SNAP_COMMON itself.
  SNAP_RELATIVE_BASE="$(realpath --relative-to="$SNAP_COMMON" "$MAP_BASE")"
  echo "Retrying posegraph save with snap-path workaround: $SNAP_RELATIVE_BASE"
  SERIALIZE_OUTPUT="$(
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
      "{filename: '$SNAP_RELATIVE_BASE'}"
  )"
  echo "$SERIALIZE_OUTPUT"
fi

if [[ "$SERIALIZE_OUTPUT" != *"result=0"* ]]; then
  echo "ERROR: slam_toolbox posegraph serialization failed." >&2
  exit 1
fi

echo "Saving occupancy map: $MAP_BASE.yaml / $MAP_BASE.pgm"
SAVE_MAP_OUTPUT="$(
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '$MAP_BASE'}}"
)"
echo "$SAVE_MAP_OUTPUT"

if [[ "$SAVE_MAP_OUTPUT" != *"result=0"* ]]; then
  echo "ERROR: slam_toolbox occupancy map save failed." >&2
  exit 1
fi

echo "Saved files:"
ls -l "$MAP_BASE".posegraph "$MAP_BASE".data "$MAP_BASE".yaml "$MAP_BASE".pgm
