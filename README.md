# ros2_graduation_project_ws

재난 정찰 로봇의 **상위 제어기 ROS 2 워크스페이스**입니다.
SLAM·측위·내비게이션, YOLO 기반 사람 탐지, VLM 안전 게이트웨이, 동작 모드 관리를 담당합니다.

하위 제어기(STM32H753ZI) 펌웨어는 별도 저장소에 있습니다.
→ [H753zi_AMR](https://github.com/JayBee2292/H753zi_AMR)

**환경** — ROS 2 Humble · Ubuntu · NVIDIA Jetson

---

## 시스템 구성

```
                    ┌──────────────── Jetson (본 저장소) ────────────────┐
  YDLidar ────────> │  slam_toolbox / AMCL / RTAB-Map                    │
  RealSense ──────> │  Nav2  +  Collision Monitor                        │
  (RGB-D · IMU)     │  YOLOv8 인식  ·  VLM Safety Gate                   │
                    │  Robot Mode Manager (7개 동작 모드)                 │
                    └───────────────────────┬───────────────────────────┘
                                            │  FD-CAN (SLCAN)
                                  0x200 cmd_vel ↓   ↑ 0x100 odometry
                    ┌───────────────────────┴───────────────────────────┐
                    │  STM32H753ZI — 엔코더 · 역기구학 · PID · PWM       │
                    └───────────────────────────────────────────────────┘
```

---

## 패키지

| 패키지 | 역할 |
|---|---|
| `h753_can_odom` | CAN 브리지, 오도메트리, IMU 융합, 자율 탐사, VLM 게이트웨이, 모드 관리 |
| `h753_perception` | YOLOv8 기반 사람 탐지 및 거리 추정 |
| `ydlidar_ros2_driver` | YDLidar 드라이버 |

---

## 동작 모드

`robot_mode_manager_node`가 7개 모드를 통합 관리합니다.
모드 전환 시 활성 노드와 안전 조건을 함께 제어합니다.

| 모드 | 이름 | 설명 |
|---:|---|---|
| 0 | `STOP` | 런타임은 유지하고 모터 명령만 차단 |
| 1 | `MANUAL_MAPPING` | 수동 주행하며 실시간 지도 생성 |
| 2 | `AUTO_MAPPING` | frontier 목표를 따라 자율 이동하며 지도 생성 |
| 3 | `MANUAL_LOCALIZATION` | 저장된 맵에서 위치를 추정하며 수동 주행 |
| 4 | `GOAL_NAVIGATION` | Nav2 목표점 자율 주행 |
| 5 | `INSPECTION_DRIVE` | 정찰 주행 |
| 6 | `DISASTER_MAPPING` | 재난 상황 매핑 |

---

## 주요 노드

### h753_can_odom

| 노드 | 설명 |
|---|---|
| `can_odom_node` | SLCAN으로 STM32와 통신, 엔코더 틱 → `/odom`·TF 발행 |
| `imu_odom_fusion_node` | 엔코더 오도메트리와 IMU 융합 |
| `odom_imu_compare_node` | 오도메트리와 IMU 정합도 정량 비교 |
| `imu_vibration_analysis` | 주행 중 IMU 진동 특성 분석 |
| `frontier_explorer_node` | frontier 기반 자율 탐사 목표 생성 |
| `vlm_gateway_node` | 외부 VLM 연동 및 안전 게이트 |
| `robot_mode_manager_node` | 7개 동작 모드 통합 관리 |
| `cmd_vel_uart_bridge_node` | UART 경로 `cmd_vel` 브리지 |
| `go2_manual_drive_node` | Unitree Go2 수동 주행 |
| `interactive_calibration` | 휠 파라미터 대화형 캘리브레이션 |

### h753_perception

| 노드 | 설명 |
|---|---|
| `yolo_perception_node` | YOLOv8 사람 탐지, RGB-D 기반 거리 추정 |

---

## VLM Safety Gate

`vlm_gateway_node`는 외부 VLM의 판단을 **명령이 아니라 검증 대상**으로 취급합니다.

- 압축 영상을 VLM에 전달하고 판단 결과를 수신
- **현재 로봇 모드에 따라** VLM의 정지 신호를 수용할지 결정하는 상태 기계
- 기본적으로 모드 3·4·5(측위·목표주행·정찰)에서만 게이트를 활성화

VLM이 오판하더라도 그 출력이 곧바로 구동 명령이 되지 않도록 하는 계층입니다.

---

## YOLO 인식

| 모델 | 용도 | 신뢰도 임계값 |
|---|---|---:|
| YOLOv8s | 사람 탐지 | 0.40 |
| YOLOv8n | 특정 색상 착의자 구분 | 0.50 |

발행 토픽 — `/yolo/target`, `/yolo/status`, 탐지 여부, 추정 거리, 압축·원본 영상

---

## 실행

```bash
colcon build --symlink-install
source install/setup.bash
```

| launch 파일 | 용도 |
|---|---|
| `sensors_bringup.launch.py` | 라이다·RealSense 등 센서 기동 |
| `can_odom.launch.py` | CAN 오도메트리 브리지 |
| `mapping_bringup.launch.py` | slam_toolbox 매핑 |
| `slam_navigation_bringup.launch.py` | SLAM + Nav2 동시 기동 |
| `localization_bringup.launch.py` | 저장 맵 기반 AMCL 측위 |
| `navigation_bringup.launch.py` | Nav2 내비게이션 |
| `rtabmap_mapping_bringup.launch.py` | RTAB-Map 3D 매핑 |
| `inspection_drive_bringup.launch.py` | 정찰 주행 |
| `calibration_bringup.launch.py` | 휠 파라미터 캘리브레이션 |
| `odom_imu_compare.launch.py` | 오도메트리–IMU 비교 검증 |

---

## 주요 파라미터

`can_odom_node` 기준입니다. (`config/h753_can_odom.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `can_baud` | 921600 | SLCAN 시리얼 속도 |
| `can_nominal_command` / `can_data_command` | `S8` / `Y5` | CAN FD 노미널·데이터 비트레이트 |
| `ppr` | 548776 | 바퀴 1회전당 엔코더 카운트 |
| `wheel_diameter_m` | 0.21 | 바퀴 지름 |
| `track_width_m` | 0.50 | 윤거 |
| `max_wheel_speed_mps` | 3.5 | 이상치 판정용 최대 바퀴 속도 |
| `delta_margin_ticks` | 2048 | 엔코더 delta 허용 마진 |

---

## 토픽

| 토픽 | 타입 | 방향 |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | 발행 |
| `/odom_vel` | `geometry_msgs/Twist` | 발행 |
| `/cmd_vel` | `geometry_msgs/Twist` | 구독 |
| `/yolo/target`, `/yolo/status` | `std_msgs/String` | 발행 |
| TF `odom → base_link` | — | 발행 |

---

## 맵

```
maps/
 ├ h753_map.{pgm,yaml,data,posegraph}   slam_toolbox 2D 맵
 ├ rtabmap.db                            RTAB-Map 3D 데이터베이스
 └ go2/go2_map.{pgm,yaml}                Unitree Go2 주행 맵
```

---

## 의존성

`nav2_*` (bringup · amcl · collision_monitor · lifecycle_manager · map_server · rotation_shim_controller) ·
`cv_bridge` · `opencv` · `numpy` · `scipy` · `pyserial` · `joy`

---

## 관련 저장소

| 저장소 | 역할 |
|---|---|
| **ros2_graduation_project_ws** (현재) | 상위 제어기 — SLAM · Nav2 · YOLO · VLM |
| [H753zi_AMR](https://github.com/JayBee2292/H753zi_AMR) | 하위 제어기 펌웨어 — 엔코더 · IK · PID · FD-CAN |
