#!/usr/bin/env bash
set -euo pipefail

RUNNER="${H753_UART_DRIVE_RUNNER:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/h753_ros_humble/tools/run_xbox_uart_drive.sh}"

if [[ ! -x "$RUNNER" ]]; then
  echo "ERROR: UART drive runner is not executable: $RUNNER" >&2
  exit 1
fi

if [[ -z "${STM_UART_PORT:-}" ]]; then
  shopt -s nullglob
  stm_ports=(/dev/serial/by-id/*STMicroelectronics*STLINK*)
  shopt -u nullglob
  if [[ "${#stm_ports[@]}" -ne 1 ]]; then
    echo "ERROR: expected exactly one STMicroelectronics ST-LINK UART port." >&2
    echo "Set STM_UART_PORT explicitly after checking /dev/serial/by-id/." >&2
    exit 1
  fi
  STM_UART_PORT="${stm_ports[0]}"
fi

if fuser "$STM_UART_PORT" >/dev/null 2>&1; then
  echo "ERROR: STM UART is already in use: $STM_UART_PORT" >&2
  fuser -v "$STM_UART_PORT" >&2 || true
  exit 1
fi

cat <<EOF
[h753 calibration UART drive]
- STM UART only: $STM_UART_PORT
- CAN telemetry: off (run_calibration.sh owns CANable)
- Drive limits: use run_xbox_uart_drive.sh defaults or caller overrides unchanged.
- Use gentle Xbox stick input during tape measurement and keep an emergency stop path available.
- Do not start run_robot_modes.sh while calibration is active.
EOF

if [[ "${1:-}" == "--check-only" ]]; then
  exit 0
fi

exec "$RUNNER" "$STM_UART_PORT" --no-can "$@"
