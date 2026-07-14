# H753 궤도형 AMR 개발 기록 및 인수인계

최종 갱신: 2026-07-14

이 문서는 프로젝트 개발 기록의 단일 기준 문서다. 현재 동작 상태와 주요 개발
흐름만 유지하며, 과거의 폐기된 추정은 기록하지 않는다.

## 1. 시스템 구성

```text
Xbox/키보드 또는 Nav2
  -> ROS 2 /cmd_vel 계열
  -> ST-LINK VCP UART
  -> STM32H753 모터 제어

STM32H753 encoder/state
  -> CA-IS2062A CAN FD
  -> CANable V2.0 Pro S
  -> h753_can_odom
  -> /odom, /odom_vel, TF odom -> base_link

YDLIDAR Tmini Pro + D435i
  -> /scan + IMU
  -> SLAM Toolbox / Nav2
```

- ROS 워크스페이스: `/home/jyl1015/ros2_graduation_project_ws`
- STM32 펌웨어: `h753_ros_humble`
- ROS 패키지: `src/h753_can_odom`
- 주행 명령은 UART, encoder/state telemetry는 CAN FD를 사용한다.
- STM 모터 제어는 현재 PID가 아닌 open-loop다.

## 2. 현재 검증된 기준 상태

### CAN 물리 계층과 장치

- 전원 차단 상태 CANH-CANL: `61 ohm`
- CANH/CANL과 공통 GND 연속성 확인 완료
- 외부 CAN FD 모듈에는 별도 STB/EN/SILENT 핀이 없다.
- STM32: nominal `1 Mbps`, data `5 Mbps`, CAN FD+BRS, normal mode
- bit timing: prescaler `5/1`, TSEG1 `20/20`, TSEG2 `3/3`, SJW `3/3`
- TDC offset: `21 mtq`
- CANable: Makerbase CANable V2.0 Pro S, STM32G431C8T6
- CANable 펌웨어: ElmueSoft Slcan 2.5 Multiboard `0x260618`
- CANable firmware SHA-256:
  `9287a1310d4b7e6052139bfc80aca10dc022b6f496bdf9b9f36e036156081dc9`
- CANable BOOT0 option byte는 `Off`로 설정했다. 전원 재인가 후 DFU가 아닌
  Slcan으로 정상 부팅한다.

현재 CANable stable path:

```text
/dev/serial/by-id/usb-ElmueSoft__netcult.ch_elmue__Slcan_2.5_-_Multiboard_209F338336305011-if00
```

최종 실기 검증:

- 사용자 지정 87.5% sample point 1M/5M, 15초: 381 frames, 오류 없음
- 기본 `S8/Y5` 1M/5M, 10초: 253 frames, 오류 없음
- H753 bus-off/TEC/REC/error event 모두 0
- ROS `can_odom_node` encoder baseline 수신 성공
- `/odom` 약 `20.30 Hz`, STM FDCAN 경고 없음
- CANable STATE/WORK LED 정상 동작

### 로봇 및 odometry 기준값

- encoder 순서: `FL, FR, RL, RR`
- timer: `FL=TIM3`, `FR=TIM4`, `RL=TIM2`, `RR=TIM8`
- PPR: `548776`
- wheel diameter: `0.21 m`
- track width/gauge: `0.45 m`
- ROS odom sign: linear `+1.0`, angular `+1.0`
- 현재 ROS 코드는 네 encoder를 좌우 각각 평균한다.
- 직진 실측 `3 m / 6.05 s = 0.496 m/s`, STM 평균 `0.501 m/s`

### 센서와 ROS 상태

- YDLIDAR Tmini Pro: `230400 baud`, 약 `10 Hz`, intensity 비활성화
- D435i IMU: librealsense2 RSUSB backend 사용
- IMU 설정: yaw axis `y`, sign `-1.0`
- mode manager에서 `enable_imu=true`, `launch_imu_odom=true`
- Nav2 inflation radius: `0.35 m`
- 저장된 지도/posegraph는 `maps/`에 유지한다.

## 3. 주요 개발 흐름

### 2026-05: STM32 주행 및 encoder 기반 확립

1. Xbox 입력을 STM UART twist 패킷으로 전달하는 open-loop 주행 경로를 구성했다.
2. UART frame을 `A5 5A 10 v_i16_le w_i16_le checksum`으로 고정했다.
3. encoder 누적을 raw counter delta의 wrap 보정 후 `int64` 누적으로 변경했다.
4. FL timer pin을 `PB4/PB5 = TIM3_CH1/CH2`로 수정했다.
5. 실측을 통해 PPR `548776`, wheel diameter `0.21 m`를 확정했다.
6. STM에서 비현실적 encoder sample을 거부하고 fault를 telemetry에 포함했다.

### 2026-05~06: ROS 2, SLAM, Nav2 통합

1. CAN telemetry를 `/odom`, `/odom_vel`, `odom -> base_link`로 변환했다.
2. YDLIDAR, SLAM Toolbox, Nav2, RealSense, RViz launch를 통합했다.
3. mode manager를 추가해 Stop/수동 mapping/자동 mapping/수동 localization/
   목표 navigation/inspection의 6개 모드를 한 프로세스에서 관리했다.
4. UART 단일 소유 lock, CAN serial exclusive open, deadman, lidar stale stop을
   추가했다.
5. 중복 YDLIDAR/CAN 프로세스가 serial stream을 나눠 읽던 문제를 제거하고,
   mode 전환 시 장치 소유자를 검사하도록 했다.
6. 키보드/조이패드/UART/odom 부호를 실제 주행 기준으로 정리했다.
7. Nav2 저속 명령이 motor stiction을 이기지 못해 bridge에 최소 명령
   `0.08 m/s`, `0.30 rad/s` floor를 추가했다.
8. D435i IMU는 Jetson kernel HID 문제를 우회하도록 RSUSB backend를 적용했다.
9. 한동안 고장 encoder 두 개만 제외하는 임시 코드가 있었으나, 하드웨어 확인 후
   네 encoder 평균 방식으로 복원했다.

### 2026-07: CAN FD 통신 장애 진단과 해결

1. 중복 CAN node, 잘못된 serial fallback, 오래된 by-id, 비독점 open을 먼저
   수정했다.
2. H753 telemetry에 bus-off, LEC/DLEC, TEC/REC/CEL, TDCV 진단값을 추가했다.
3. H753 external loopback과 역방향 400-frame 시험으로 STM RX 및 기본 bit timing을
   검증했다.
4. 기존 Makerbase firmware에서는 STM 송신 방향에서 ACK/data phase 오류와
   bus-off가 반복됐다. USB 부하, D435i, xone은 원인에서 배제됐다.
5. CANable을 ElmueSoft Slcan 2.5 Makerbase Multiboard image로 업데이트했다.
6. Pro S의 BOOT 지점은 PB8/BOOT0과 연결되어 있어 점퍼가 꽂힌 동안 CAN RX를
   방해했다. 점퍼를 제거하고 `*Boot0:Off`를 적용했다.
7. 전원 재인가 후 1M/5M raw CAN 및 ROS odom 시험이 모두 정상화됐다.
8. ROS 설정과 자동 장치 판별을 새 ElmueSoft by-id에 맞게 수정했다.

### 2026-07-14: 빌드 정리

- ROS `build/install/log`, STM 중복 build, Python cache, 도구용 `.venv`, 로컬
  `dfu-util` 복사본을 정리했다.
- STM firmware directory에 `COLCON_IGNORE`를 추가해 colcon의 중복 CMake
  build를 막았다.
- STM Debug build와 ROS 두 패키지 clean build를 완료했다.
- 전체 workspace 용량은 약 `471 MB -> 300 MB`로 감소했다.
- 지도와 실기 로그는 보존했다.

## 4. 주요 ROS 실행 모드

| Mode | 용도 | 명령 소스 |
| --- | --- | --- |
| 0 | Stop | 강제 zero |
| 1 | 수동 mapping | Xbox/키보드 |
| 2 | 자동 frontier mapping | Nav2 |
| 3 | 저장 지도 수동 localization | Xbox/키보드 |
| 4 | 저장 지도 goal navigation | Nav2 |
| 5 | 센서/구동 inspection | Xbox/키보드 |

통합 실행:

```bash
cd /home/jyl1015/ros2_graduation_project_ws
./run_robot_modes.sh
```

관련 보조 실행:

```bash
./run_calibration.sh
./run_slam_bringup.sh
./save_slam_map.sh
./run_localization.sh
./run_navigation.sh
./run_slam_navigation.sh
./run_auto_explore.sh
./run_odom_imu_compare.sh
```

## 5. 빌드와 기본 확인

ROS clean build:

```bash
cd /home/jyl1015/ros2_graduation_project_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

STM32 Debug build:

```bash
cd /home/jyl1015/ros2_graduation_project_ws/h753_ros_humble
cmake --preset Debug
cmake --build --preset Debug
```

생성 ELF:

```text
h753_ros_humble/build/Debug/h753_ros_humble.elf
```

STM build 디렉터리는 ROS workspace 안에 있지만 `COLCON_IGNORE` 대상이다.
STM firmware는 별도 CMake preset으로만 빌드한다.

## 6. 운용 시 지켜야 할 사항

- CAN odom node와 YDLIDAR reader는 각각 하나만 실행한다.
- ROS 운용 중 standalone Xbox 도구가 ST-LINK UART를 동시에 열면 안 된다.
- 장치는 `/dev/ttyACM*` 번호보다 `/dev/serial/by-id`를 우선 사용한다.
- CANable BOOT 점퍼는 firmware update 때만 사용하고 normal 운용에서는 제거한다.
- H753 Nucleo `LD4 COM`의 빨간색/점멸은 ST-LINK 통신 활동이며 CAN 오류 표시가
  아니다. overcurrent LED는 `LD6`이다.
- PID는 아직 비활성화 상태다. encoder/SLAM 검증 전에 활성화하지 않는다.
- Xbox 도구용 `.venv`는 정리했으므로 standalone 도구 사용 시 다시 생성한다.

## 7. 다음 우선순위

1. 안정화된 1M/5M CAN 상태에서 mode 5 직진/좌우 회전과 RViz odom 방향을 확인한다.
2. D435i IMU yaw 축/부호를 실제 제자리 회전으로 최종 보정한다.
3. 수동 SLAM 직사각형 loop를 주행해 track width와 lidar offset을 미세 조정한다.
4. mode 2/4에서 `/cmd_vel -> /cmd_vel_selected -> /cmd_vel_safe -> UART`를 확인하고
   collision monitor의 감속/정지를 실공간에서 검증한다.
5. 저장 map localization과 Nav2 goal 주행을 검증한 뒤 frontier exploration을
   시험한다.
