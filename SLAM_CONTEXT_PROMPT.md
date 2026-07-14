# SLAM integration context prompt

Use this file as the starting prompt when continuing the project later.

> Update on 2026-05-27: read `JETSON_CODEX_HANDOFF.md` first for the current
> open-loop runtime state, corrected encoder PPR (`548776`), and Jetson
> migration steps.

## Current robot communication architecture

- Board project: STM32H753 firmware in `h753_ros_humble`.
- PC control script: `tools/xbox_uart_can_telemetry.py`.
- Driving control path: Xbox controller on PC sends open-loop drive commands to STM32 over UART.
- Telemetry path: STM32 sends robot state and encoder data to PC over CAN FD+BRS.
- PC CAN hardware: Makerbase CANable V2.0 Pro, usually visible as `/dev/ttyACM1`.
- STM32 CAN transceiver: WeAct ISOCANFD module V1, CA-IS2062A, 5 Mbps CAN FD transceiver.
- Typical run command:

```bash
cd ~/h753_ros_humble/tools
python3 xbox_uart_can_telemetry.py --port /dev/ttyACM0 --can-port /dev/ttyACM1
```

Do not put a shell line-continuation backslash before a separated argument unless the command is split across lines correctly.

## STM32 runtime state

- Runtime loop is integrated in `Core/Src/freertos.c`.
- UART drive control is implemented in `App/Src/app_uart_smoke.c`.
- CAN telemetry is implemented in `App/Src/can_telemetry.c`.
- Encoder and robot kinematics are implemented in `Algorithm/Src/calc_input_data.c`.
- Wheel order is always FL, FR, RL, RR.
- Encoder timer map is in `Core/Src/main.c`:
  - FL: TIM3
  - FR: TIM4
  - RL: TIM2
  - RR: TIM8

## Encoder handling

- Encoder counting is done by hardware TIM encoder mode.
- Firmware does not use update interrupts for 64-bit counting.
- Instead, every 20 ms it reads raw CNT, computes wrap-corrected delta, and accumulates that into `int64_t total_tick`.
- This keeps all four timers as 64-bit software accumulators while avoiding TIM2 32-bit overflow counter glitches.
- Previous issue: RL on TIM2 printed huge negative values like `-2147...` because TIM2 is 32-bit and overflow-count math incorrectly added/subtracted multiples of `2^32`.

## STM encoder fault defense

- STM rejects physically implausible encoder samples before integrating them.
- Current threshold is based on `ENCODER_MAX_WHEEL_SPEED_MPS = 3.5f` plus `ENCODER_MAX_DELTA_MARGIN_TICKS = 2048`.
- If a wheel delta exceeds the threshold:
  - that sample is not added to `total_tick`
  - that wheel velocity is set to zero for the sample
  - `wheel->is_fault` is set
  - the raw counter baseline is resynchronized to the current CNT
  - CAN still transmits telemetry with fault flags

## CAN FD telemetry protocol

- `0x100`, STM32 to host, 64 bytes:
  - `u32 seq`
  - `u32 timestamp_ms`
  - `f32 robot_linear_v_mps`
  - `f32 robot_angular_w_radps`
  - `f32 wheel_velocity_mps[4]`
  - `i16 left_duty_percent`
  - `i16 right_duty_percent`
  - `i16 duty_percent`
  - `i16 curve_ratio_percent`
  - `u8 motion`
  - `u8 flags`
    - bit0: robot ready
    - bit1: motion active
    - bit2: encoder fault present

- `0x101`, STM32 to host, 64 bytes:
  - `u32 seq`
  - `u32 timestamp_ms`
  - `i64 encoder_tick[4]`
  - `u32 encoder_delta_abs[4]`
  - `u8 encoder_valid_mask`
  - `u8 encoder_fault_mask`
  - reserved bytes

- `0x1FF`, host to STM32:
  - telemetry enable/disable command.

## SLAM plan

The SLAM stack should not consume raw encoder ticks directly. Build a ROS 2 odometry node first:

1. Read CAN telemetry from the PC side.
2. Validate `seq`, `timestamp_ms`, `dt`, `encoder_fault_mask`, and wheel delta limits.
3. Convert wheel tick deltas to skid-steer odometry.
4. Publish `nav_msgs/Odometry`.
5. Publish TF `odom -> base_link`.
6. Feed 2D LiDAR `sensor_msgs/LaserScan` plus odom/TF into SLAM Toolbox or another 2D LiDAR SLAM package.

Keep a second layer of PC-side defense even though STM already rejects bad encoder samples. PC-side odometry must still reject packet loss, timestamp jumps, impossible speed/acceleration, CAN decoding errors, and any nonzero encoder fault mask.
