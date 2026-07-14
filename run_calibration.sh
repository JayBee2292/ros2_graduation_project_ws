#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

SRC_DIR="$PWD/src/h753_can_odom"

check_robot_mode_manager() {
  if pgrep -f 'h753_can_odom[/ ]robot_mode_manager_node' >/dev/null 2>&1; then
    echo "ERROR: run_robot_modes.sh is already running. Stop it with Ctrl+C before calibration." >&2
    pgrep -af 'h753_can_odom[/ ]robot_mode_manager_node' >&2 || true
    exit 1
  fi
}

check_lidar_port() {
  for arg in "$@"; do
    if [[ "$arg" == "launch_lidar:=false" ]]; then
      return
    fi
  done

  if fuser /dev/ttyUSB0 >/dev/null 2>&1; then
    echo "ERROR: /dev/ttyUSB0 is already in use. Stop the existing YDLIDAR/ROS launch first." >&2
    fuser -v /dev/ttyUSB0 >&2 || true
    exit 1
  fi
}

if [[ "${1:-}" == "--launch-only" ]]; then
  shift
  check_robot_mode_manager
  check_lidar_port "$@"
  exec ros2 launch h753_can_odom calibration_bringup.launch.py "$@"
fi

if [[ "${1:-}" == "--tool-only" ]]; then
  shift
  exec ros2 run h753_can_odom interactive_calibration --ros-args \
    -p odom_config_path:="$SRC_DIR/config/h753_can_odom.yaml" \
    -p imu_config_path:="$SRC_DIR/config/h753_imu_odom_fusion.yaml" \
    -p sensor_tf_config_path:="$SRC_DIR/config/h753_sensor_tf.yaml" \
    "$@"
fi

cat <<'EOF'
[h753 interactive calibration mode]
- SLAM: off
- IMU fusion: off
- Camera: on for D435i IMU measurement
- Active: raw wheel /odom, /odom_vel, odom->base_link TF, /scan, D435i IMU, RViz
- Do not run run_robot_modes.sh at the same time: both runtimes own YDLIDAR and CANable.
- Xbox UART driving starts automatically in the background with the same
  limits as ~/h753_ros_humble/tools/run_xbox_uart_drive.sh.
- This terminal stays on the numbered calibration menu.

Menu:
1. 직진 측정 -> odom_linear_sign 저장
2. 제자리 반시계 회전 측정 -> odom_angular_sign + 유효 track_width_m 저장
3. IMU 반시계 회전 측정 -> imu_yaw_axis/sign 저장
4. 라이다 앞 물체 측정 -> laser_yaw 저장
5. 현재 저장값 보기

EOF

check_robot_mode_manager
check_lidar_port "$@"

LOG_DIR="$HOME/.ros/log/h753_calibration"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/launch_$(date +%Y%m%d_%H%M%S).log"
UART_LOG_FILE="$LOG_DIR/uart_drive_$(date +%Y%m%d_%H%M%S).log"
UART_DRIVE_PID=""

setsid ros2 launch h753_can_odom calibration_bringup.launch.py "$@" >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if [[ -n "$UART_DRIVE_PID" ]] && kill -0 "$UART_DRIVE_PID" 2>/dev/null; then
    kill -TERM -- "-$UART_DRIVE_PID" 2>/dev/null || true
    wait "$UART_DRIVE_PID" 2>/dev/null || true
  fi
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    for _ in {1..80}; do
      if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        wait "$LAUNCH_PID" 2>/dev/null || true
        return
      fi
      sleep 0.1
    done
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "calibration launch log: $LOG_FILE"
echo "RViz와 센서가 뜰 때까지 잠시 기다립니다."
sleep 3

setsid "$PWD/run_calibration_uart_drive.sh" >"$UART_LOG_FILE" 2>&1 &
UART_DRIVE_PID=$!
sleep 1
if ! kill -0 "$UART_DRIVE_PID" 2>/dev/null; then
  wait "$UART_DRIVE_PID" 2>/dev/null || true
  echo "ERROR: calibration Xbox UART drive failed to start. Check $UART_LOG_FILE" >&2
  exit 1
fi
echo "calibration Xbox UART drive log: $UART_LOG_FILE"

ros2 run h753_can_odom interactive_calibration --ros-args \
  -p odom_config_path:="$SRC_DIR/config/h753_can_odom.yaml" \
  -p imu_config_path:="$SRC_DIR/config/h753_imu_odom_fusion.yaml" \
  -p sensor_tf_config_path:="$SRC_DIR/config/h753_sensor_tf.yaml"
