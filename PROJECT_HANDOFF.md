# H753 궤도형 AMR 개발 기록 및 인수인계

최종 갱신: 2026-07-27

이 문서는 프로젝트 개발 기록의 단일 기준 문서다. 현재 동작 상태와 주요 개발
흐름만 유지하며, 과거의 폐기된 추정은 기록하지 않는다.

## 1. 시스템 구성

```text
Xbox/키보드 또는 Nav2
  -> ROS 2 /cmd_vel 계열 -> /cmd_vel_safe
  -> h753_cmd_vel_uart_bridge (injury_stop/scan/deadman 가드 병합)
  -> ST-LINK VCP UART
  -> STM32H753 모터 제어

STM32H753 encoder/state
  -> CA-IS2062A CAN FD
  -> CANable V2.0 Pro S
  -> h753_can_odom
  -> /odom, /odom_vel, TF odom -> base_link

YDLIDAR Tmini Pro + D435i
  -> /scan + IMU + 640x480@15 RGB (mode 4/5는 aligned RGB-D도)
  -> SLAM Toolbox / Nav2 + h753_perception(YOLO) + h753_vlm_gateway

h753_perception (보드, mode 4/5에서만 실행)
  -> /yolo/person_found, /yolo/blue_person (사람/파란 옷 감지)
  -> /yolo/target, /yolo/nearest_distance_m, /yolo/status,
     /yolo/detected_image/compressed

h753_vlm_gateway (보드, mode 3/4/5)
  <- /yolo/person_found, /yolo/blue_person  -> 즉시(로컬) /safety/vlm_stop=1
  -> optional /vlm/request/image/compressed -> 원격 VLM 서버(노트북 vlm.py)
  <- /vlm/injury_stop, /vlm/result, /vlm/result_detail  (서버 판정/해제)
  -> validated /safety/vlm_stop -> h753_cmd_vel_uart_bridge
```

YOLO 즉시-정지와 VLM 서버 판정은 같은 `/safety/vlm_stop`에 병합된다. 정지는
둘 중 하나만 걸려도 걸리고(OR), 해제는 오직 서버의 명시적
`/vlm/injury_stop=0`으로만 된다 — 4.5절 "정지 토픽 정리" 참고.

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
- physical track center gauge: `0.50 m`
- odom `track_width_m`: `0.50 m`에서 시작해 실제 매핑 회전 속도의
  skid-steer 유효값으로 캘리브레이션한다.
- ROS odom sign: linear `+1.0`, angular `+1.0`
- 현재 ROS 코드는 네 encoder를 좌우 각각 평균한다.
- 직진 실측 `3 m / 6.05 s = 0.496 m/s`, STM 평균 `0.501 m/s`

### 센서와 ROS 상태

- YDLIDAR Tmini Pro: `230400 baud`, 약 `10 Hz`, intensity 비활성화
- D435i IMU: librealsense2 RSUSB backend 사용
- IMU 설정: yaw axis `y`, sign `-1.0`
- mode manager에서 `enable_imu=true`, `launch_imu_odom=true`
- 원격 VLM 안전 인터록은 `h753_vlm_gateway`가 소유한다.
  - 현재 서버 입력 영상: `/camera/camera/color/image_raw/compressed`
  - 추후 전용 입력으로 사용할 수 있는 alias: `/vlm/request/image/compressed`
  - 서버 원시 정지: `/vlm/injury_stop` (`std_msgs/msg/Int32`)
  - 로봇 내부 검증 정지: `/safety/vlm_stop` (`std_msgs/msg/Int32`)
  - 서버 재판정 상태 입력: `/vlm/status` (`std_msgs/msg/String`, JSON)
  - gateway 연결/안전 상태: `/vlm/gateway/status` (`std_msgs/msg/String`, JSON)
  - gateway는 mode `3/4/5`에서만 활성화되고 mapping mode `1/2/6`에서는 제외된다.
  - `0`: 주행 허용
  - `1` 또는 잘못된 non-zero 값: UART 모터 명령 강제 정지
  - publisher가 없거나 아직 메시지를 받지 않은 경우: 인터록 제외, 기존 주행 허용
  - 한 번 정지되면 서버가 명시적으로 `0`을 보낼 때까지 정지 상태 유지
  - 정지 중에도 영상 전달과 VLM 탐지는 `0`을 받을 때까지 계속한다.
  - **온보드 즉시 정지 (2026-07-16)**: gateway가 `/yolo/person_found`,
    `/yolo/blue_person`(둘 다 `h753_perception`이 로봇 위에서 직접 발행)을
    구독한다. 둘 중 하나라도 `0→1`이 되는 순간 서버 왕복 없이 그 자리에서
    `/safety/vlm_stop=1`을 발행한다. YOLO 탐지와 정지 실행이 모두 로봇 안에서
    끝나므로 Wi‑Fi/서버 지연이나 단절이 즉시 정지를 막지 못한다. 정지 해제는
    여전히 서버의 명시적 `/vlm/injury_stop=0`만 수행한다 — 부상 여부 판정 권한은
    계속 VLM 서버가 가진다.
- D435i normal profile: color `640x480@15`, depth/align/pointcloud 비활성화
- RTAB-Map 전용 profile에서만 depth와 align을 활성화한다.
- Nav2 inflation radius: `0.30 m`
- 저장된 지도/posegraph는 `maps/`에 유지한다.
- localization/navigation은 AMCL이 아니라 slam_toolbox localization을 사용한다.
  RViz 위치 벡터는 publisher가 없는 `/particle_cloud` 대신 slam_toolbox의
  `geometry_msgs/PoseWithCovarianceStamped` 출력 `/pose`를 표시한다.

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

### 2026-07-14: VLM 통신 소유권 통합

1. `h753_vlm_gateway`를 추가해 D435i 영상 전달, 서버 결과 수신, 값 검증,
   discovery/응답 지연 상태 발행을 로봇 workspace로 이동했다.
2. UART bridge의 안전 입력을 서버 토픽 `/vlm/injury_stop`에서 로봇 내부 토픽
   `/safety/vlm_stop`으로 분리했다.
3. mode manager가 모든 주행 mode `1~6`에서 D435i RGB `640x480@15` 화면을
   제공한다. mapping mode `1/2/6`은 RGB+IMU만 켜고 Depth/IR/pointcloud와
   VLM gateway는 꺼 부하를 제한한다. mode `3`은 일반 RGB camera profile,
   mode `4/5`는 YOLO가 켜질 때 aligned RGB-D profile을 사용한다.
4. VLM/일반 preview용 RGB profile과 YOLO/RTAB-Map용 aligned RGB-D profile을
   분리해, depth가 필요 없는 mode에서는 USB와 Jetson 부하를 제한한다.

### 2026-07-15: YOLO 기능을 mode 4/5에 통합

1. `h753_perception` package를 추가해 `~/yolo_project/person.py`와
   `person_blue.py`의 실제 기능을 하나의 robot-side process로 통합했다.
2. 기존 `yolov8s.pt` 사람 confidence `0.40`, 중심 `20%` depth median ROI와
   `yolov8n.pt` 파란 옷 confidence `0.50`, HSV `H=100~130/S>=80/V>=50`,
   torso `15~60%`, blue ratio `15%` 설정은 그대로 유지한다.
3. mode `4 GOAL_NAVIGATION`과 mode `5 INSPECTION_DRIVE`에서만 YOLO를 자동
   실행한다. mode 4는 GUI를 끄고, mode 5는 legacy preview와 HSV tuning
   window를 켠다. mode 3과 4는 같은 localization topology지만 YOLO process
   구성이 달라 전환 시 runtime을 안전하게 재시작한다.
4. RealSense는 계속 robot workspace가 한 번만 실행한다. mode 4/5에서
   `640x480@15` color/depth와 aligned depth를 켜며, yolo_project의
   `test_run.launch.py`는 실행하지 않는다.
5. 출력은 `/yolo/person_found`, `/yolo/blue_person`, `/yolo/target`,
   `/yolo/nearest_distance_m`, `/yolo/status`,
   `/yolo/detected_image/compressed`이다. `/yolo/person_found`,
   `/yolo/blue_person`은 `h753_vlm_gateway`가 직접 구독해 `0→1` 순간
   `/safety/vlm_stop`을 즉시(서버 왕복 없이) 발행한다 — 2절 "온보드 즉시 정지"
   참고. 정지 해제는 여전히 `/vlm/injury_stop -> h753_vlm_gateway ->
   /safety/vlm_stop -> UART bridge` 경로로 서버만 수행한다.
   mode 4/5에서는 manager가 `/yolo/person_found`의 상태 변화를 받아 실행
   터미널에 사람 인식/해제 로그를 한 번씩 출력한다.
6. torch/ultralytics는 기존 GPU virtualenv, 모델 weight는 기존 경로를 참조한다.
   따라서 `~/yolo_project` launch나 run script를 별도로 실행할 필요는 없지만,
   `/home/jyl1015/yolo_project/venv_gpu`와 두 `.pt` 파일은 아직 runtime 자산으로
   보존해야 한다.

### 2026-07-15: SLAM 정합 안정화

1. Tmini Pro 실측 출력은 약 `10 Hz`, `430 points/scan`이었다. 기존 수동 최대
   회전 속도 `2.67 rad/s`에서는 한 scan 동안 약 `15.3 deg` 회전하므로,
   deskew가 없는 scan 자체가 휘어져 정합을 방해할 수 있었다.
2. mapping mode `1/2/6`에 별도 저속 제한을 적용했지만 open-loop PWM에서 궤도
   정지마찰을 넘지 못했다. 이후 모든 주행 mode `1~6`을 기존 UART teleop과
   동일한 `0.60 m/s`, `2.67 rad/s` 한계로 통일했다. mode별 차이는 명령 소스와
   동작 목적뿐이며 최종 UART track 제한과 충돌/VLM/scan 안전정지는 유지된다.
3. IMU 실측 recorder 세 개가 동시에 실행된 상태에서 collision monitor 출력
   간격이 약 `0.354 s`까지 늘어났다. 중복 recorder를 종료하자 최대 간격이
   약 `0.100 s`로 회복되어 UART deadman은 안전값 `0.30 s`를 유지했다.
   측정 스크립트에는 단일 recorder lock을 추가했다.
4. D435i USB가 끊겼을 때 IMU fusion pose를 encoder의 절대 pose로 교체하던
   동작을 제거했다. 이제 현재 fused pose를 원점으로 유지한 채 wheel odom의
   상대 이동량과 상대 회전량을 적분하고, IMU 복구 후 같은 pose에서 이어간다.
5. D435i startup gyro bias는 정지 상태의 `200 samples`(약 1초)로 자동 계산한다.
   매핑 시작 직후 `Startup gyro bias ready` 로그가 나오기 전까지 로봇을 움직이지
   않는다.
6. 핵심 pose/속도 선택 알고리즘에는 pseudocode 주석을, 관련 YAML 설정에는
   파라미터 용도와 단위 주석을 추가했다.
7. 새 SLAM 지도의 occupancy resolution을 `0.05 m/cell`에서 `0.025 m/cell`로
   낮추고 RViz LaserScan 표시를 `3 px` 사각형에서 `1 px` 점으로 바꿨다. 기존에
   저장된 5 cm 지도는 바뀌지 않으므로 새 해상도 비교에는 재매핑이 필요하다.
6. 새 맵을 만드는 frontier mode 2는 그대로 유지하고, 저장 posegraph를 불러와
   재난 후 scan을 추가하는 mode 6 `DISASTER_MAPPING`을 별도로 추가했다.

### 2026-07-15: IMU 진동 억제 및 yaw 융합 (1차 구현, 실측 활성화 전)

알고리즘의 중심은
Yi et al.의 skid-steer IMU/encoder 융합 구조와 Zhang et al.의 동적 분산 기반
adaptive Kalman filtering이며, 실시간 전처리와 현재 2D SLAM 구조에 맞게
단순화한다.

참고 논문:

- [Yi et al., Kinematic Modeling and Analysis of Skid-Steered Mobile Robots, 2009](https://jingangyi.rutgers.edu/pdfs/IEEETRO_2009.pdf)
- [Yi et al., IMU-Based Localization and Slip Estimation for Skid-Steered Mobile Robots, 2007](https://www.researchwithrutgers.org/en/publications/imu-based-localization-and-slip-estimation-for-skid-steered-mobil/)
- [Zhang et al., Dynamic Variance Adaptive Filtering for MEMS Gyroscope, 2018](https://www.mdpi.com/1424-8220/18/11/3943)
- [Bai et al., Adaptive Filtering for MEMS Gyroscope with Dynamic Noise Model, 2020](https://www.sciencedirect.com/science/article/pii/S0019057820300410)
- [Suwandi et al., Vehicle Vibration Error Compensation with Adaptive LMS and LPF, 2019](https://www.jstage.jst.go.jp/article/ipsjjip/27/0/27_33/_article)

현재 구성은 encoder가 선속도를 제공하고 D435i gyro가 yaw rate를 제공하며,
IMU timeout에는 relative wheel yaw로 연속 전환한다. 계획된 필터는 이 구조를
교체하지 않고 gyro axis/sign 보정과 yaw 적분 사이에 삽입한다. 장기 오차는
계속 slam_toolbox의 scan matching이 `map -> odom`에서 보정한다.

계획된 처리 순서:

```text
D435i raw gyro Y, 200 Hz
  -> calibrated axis/sign 적용 및 bias 제거
  -> Hampel 이상치 제거
  -> 2차 Butterworth low-pass
  -> high-frequency residual의 rolling variance 계산
  -> 진동 분산에 따라 IMU 신뢰도/가중치 조절
  -> encoder yaw rate와 adaptive fusion
  -> yaw 적분
  -> LiDAR SLAM 장기 보정
```

핵심 pseudocode:

```text
ON_IMU_SAMPLE(raw_imu, timestamp):
    selected_rate = SELECT_AXIS(raw_imu.angular_velocity, axis="y")
    corrected_rate = imu_yaw_sign * (selected_rate - gyro_bias)

    # 짧고 큰 USB/센서 spike가 적분 yaw를 오염시키지 않게 한다.
    median = MEDIAN(recent_corrected_rates)
    mad = MEDIAN_ABSOLUTE_DEVIATION(recent_corrected_rates)
    IF ABS(corrected_rate - median) > hampel_sigma * 1.4826 * mad:
        robust_rate = median
    ELSE:
        robust_rate = corrected_rate

    filtered_rate = BUTTERWORTH_LOWPASS(robust_rate, cutoff_hz, sample_rate_hz)
    vibration_residual = robust_rate - filtered_rate
    vibration_variance = ROLLING_VARIANCE(vibration_residual, vibration_window)

    CACHE(filtered_rate, vibration_variance, timestamp)


ON_WHEEL_ODOM(wheel_odom, timestamp):
    encoder_linear_rate = wheel_odom.linear_x
    encoder_yaw_rate = wheel_odom.angular_z

    IF IMU_IS_STALE(timestamp):
        fused_yaw_rate = encoder_yaw_rate
    ELSE:
        # 낮은 진동에서는 IMU를 강하게 사용하고, 진동이 커지면 비중을 줄인다.
        imu_weight = MAP_VARIANCE_TO_WEIGHT(
            vibration_variance,
            variance_low,
            variance_high,
            imu_weight_max,
            imu_weight_min,
        )

        innovation = filtered_imu_rate - encoder_yaw_rate
        IF vibration_variance IS_HIGH AND ABS(innovation) IS_LARGE:
            imu_weight = REDUCE_IMU_WEIGHT(imu_weight)
        ELSE IF robot_is_turning AND vibration_variance IS_LOW:
            # 무한궤도 제자리 회전은 encoder slip 가능성이 크므로 IMU를 우선한다.
            imu_weight = KEEP_IMU_PRIORITY(imu_weight)

        fused_yaw_rate = (
            imu_weight * filtered_imu_rate
            + (1 - imu_weight) * encoder_yaw_rate
        )

    IF IMU_IS_FRESH AND robot_is_stationary \
       AND vibration_variance IS_LOW FOR bias_hold_time:
        gyro_bias = SLOW_ONLINE_BIAS_UPDATE(selected_rate)

    delta_yaw = fused_yaw_rate * dt
    distance = encoder_linear_rate * dt
    x, y, yaw = INTEGRATE_PLANAR_POSE_AT_MID_HEADING(
        x, y, yaw, distance, delta_yaw
    )
    PUBLISH_ODOM_AND_TF(x, y, yaw, encoder_linear_rate, fused_yaw_rate)
```

계획 파라미터와 의미는 다음과 같다. 수치는 확정값이 아니라 raw log 분석용
초기 후보이며, PSD와 회전 실측 후 YAML에 반영한다.

| Parameter 후보 | 초기 후보 | 의미 |
| --- | ---: | --- |
| `enable_vibration_filter` | `false` | A/B 시험 전 기존 동작을 보존하는 기능 스위치 |
| `hampel_window_samples` | `5` | 200 Hz 기준 25 ms 이상치 판정 구간 |
| `hampel_threshold_sigma` | `3.0` | median에서 spike로 판정할 robust sigma 배수 |
| `gyro_lpf_cutoff_hz` | `5/10/15/20` 비교 | 실제 회전은 보존하고 궤도 진동은 줄이는 LPF cutoff |
| `vibration_window_s` | `0.25~0.50` | high-frequency residual 분산 계산 구간 |
| `vibration_variance_low` | 측정 후 결정 | IMU를 정상으로 간주하는 분산 하한 |
| `vibration_variance_high` | 측정 후 결정 | IMU 신뢰도를 낮추기 시작할 분산 상한 |
| `imu_weight_min` | 측정 후 결정 | 고진동에서도 유지할 최소 IMU 비중 |
| `imu_weight_max` | 측정 후 결정 | 정상 회전에서 사용할 최대 IMU 비중 |
| `innovation_gate_rad_s` | 측정 후 결정 | IMU와 encoder yaw rate 차이의 이상 판단 기준 |
| `online_bias_alpha` | 측정 후 결정 | 정지 상태 gyro bias의 느린 갱신 비율 |
| `bias_hold_time_s` | `1.0` 후보 | bias 갱신 전 연속 정지 확인 시간 |

구현 및 검증 계획:

1. 필터를 구현하기 전에 `/camera/camera/imu`, `/wheel/odom`, `/odom`,
   `/cmd_vel_selected`를 rosbag으로 기록한다. 정지, 저속 직진, 제자리 회전
   `0.20/0.35/0.50 rad/s`를 CW/CCW 각각 3회 이상 측정한다.
2. Welch PSD와 Allan deviation으로 정지 noise floor, 좁은 진동 peak,
   회전 속도별 분산 변화를 계산한다. cutoff와 variance threshold는 이 결과로
   결정하고 임의로 확정하지 않는다.
3. Hampel과 2차 Butterworth를 ROS와 분리된 pure function으로 구현하고
   spike 제거, step 응답, timestamp gap, cutoff별 지연을 unit test한다.
4. 기존 `h753_imu_odom_fusion`에 `enable_vibration_filter:=false`를 기본값으로
   추가한다. 먼저 raw IMU와 filtered IMU만 비교하고 pose 적분에는 연결하지
   않는다.
5. 필터값이 회전 step을 훼손하지 않는 것이 확인되면 yaw 적분에 연결한다.
   그 다음 rolling variance 기반 adaptive weight를 별도 단계로 활성화한다.
6. encoder-only, 기존 IMU, filtered IMU, adaptive fusion 네 조건에서 동일한
   360도 회전과 직사각형 SLAM loop를 수행해 최종 yaw 오차와 맵 중첩을 비교한다.
7. 단순 adaptive weighted fusion으로 부족할 때만 상태 `[yaw, gyro_bias,
   slip/effective_track_width]`를 갖는 EKF로 확장한다. 처음부터 EKF와 진동
   필터를 동시에 바꾸지 않아 원인 분석이 가능하도록 한다.

잠정 통과 기준:

- 필터 지연이 `50 ms` 이하이고 급격한 회전 방향 전환을 삭제하지 않는다.
- 모터/궤도 진동 구간 gyro RMS가 raw 대비 최소 `40%` 감소한다.
- 모든 시험 속도에서 360도 yaw 오차가 기존 IMU보다 악화되지 않고, 전체
  median absolute yaw error가 최소 `20%` 감소한다.
- IMU timeout/recovery 시 fused pose와 yaw가 불연속적으로 점프하지 않는다.
- 동일한 직사각형 loop에서 시작/종료 벽의 이중선과 맵 찢어짐이 감소한다.

Hampel window, cutoff, adaptive weight는 위 측정을 통과한 값만 최종 파라미터로
승격한다. 기계적 방진은 소프트웨어 필터와 별개로 시험하되, 지나치게 부드러운
마운트가 저주파 공진과 camera extrinsic 변화를 만들지 않도록 동일 로그로
장착 전후를 비교한다.

1차 구현 현황:

- `imu_filtering.py`에 Hampel, 2차 Butterworth biquad, rolling population
  variance, adaptive IMU weight, yaw-rate blending을 ROS 비의존 pure algorithm으로
  구현했다.
- `h753_imu_odom_fusion`은 후보 필터를 항상 계산하지만
  `enable_vibration_filter`, `enable_adaptive_yaw_fusion`,
  `enable_online_bias_update`의 기본값을 모두 `false`로 유지한다. 따라서 실측값을
  확정하기 전에는 기존 검증된 raw IMU yaw 적분 동작이 바뀌지 않는다.
- `/imu_filter/yaw_rate_raw`, `/imu_filter/yaw_rate_filtered`,
  `/imu_filter/vibration_variance`, `/imu_filter/imu_weight`를 200 Hz sensor-data
  QoS로 발행해 A/B rosbag을 만들 수 있다.
- IMU timestamp가 역행하거나 `max_imu_dt_s`를 초과하면 causal filter history를
  초기화해 USB 복구 후 오래된 filter state가 yaw를 튀게 하지 않는다.
- `record_imu_vibration.sh`를 추가했고, `imu_vibration_analysis`가 rosbag의
  summary, Welch PSD, Allan deviation CSV를 생성한다.
- pure algorithm, PSD/Allan 도구와 기존 기능을 포함한 package test를 유지한다.

데이터 수집과 분석:

```bash
./record_imu_vibration.sh stationary
./record_imu_vibration.sh rotate_cw_020
./record_imu_vibration.sh rotate_ccw_020

ros2 run h753_can_odom imu_vibration_analysis \
  ~/.ros/bag/h753_imu_vibration/<bag_directory>
```

분석 결과는 선택한 bag directory의 `imu_analysis/summary.csv`,
`raw_yaw_rate_psd.csv`, `raw_yaw_rate_allan.csv`에 생성된다. 현재 YAML의
`10 Hz`, variance threshold, weight 범위는 실행 가능한 후보일 뿐이며 위 실측
결과가 나오기 전에는 세 기능 스위치를 활성화하지 않는다.

### 2026-07-22: 구동 하한·조향비 보정과 넓은 유리 복도 매핑 진단

구동 변경:

1. 적재 상태에서는 PWM 40% 이하로 궤도가 움직이지 않는 조건을 반영했다.
   STM의 `0.60 m/s = 100%` 변환을 역산해, 움직이는 각 트랙의 최소 출력을
   당시 초기값을 PWM 41%인 `0.246 m/s`로 설정했다. 이후 2026-07-26 실측으로
   50%까지 상향했다. zero 명령과 `0.01 m/s` 이하 deadband,
   VLM/scan/deadman 정지는 계속 완전 정지다.
2. 좌·우 트랙을 같은 비율로 확대해 곡률을 보존한 뒤 다시 차체 `v`, `w`로
   환산한다. 명령 변환용 track gauge는 STM 펌웨어와 같은 `0.45 m`이며,
   encoder odometry의 `track_width_m=0.50 m`와는 용도가 다르다.
3. mapping mode 1·2·6에 시험했던 `0.25 m/s`, `0.35 rad/s` 상한은 모터 힘 부족으로
   주행하지 못해 롤백했다. 현재 모든 주행 mode의 상한은 다시 `0.60 m/s`,
   `2.67 rad/s`이고, 당시 트랙별 PWM 하한은 41%였다(현재 50%, 제자리 회전 55%).
4. standalone `run_xbox_uart_drive.sh`와 ROS mode manager의 이동 조향비를
   `40:100 -> 50:100 -> 75:100` 순서로 보정했다. 과거 `40:100` 로그의 평균
   encoder delta는 약 `1:4.61`, 2026-07-22 `50:100` 실측은 정상상태에서 약
   `1:3.5~3.6`이었다. STM이 아직 open-loop라 명령비가 실제 속도비를 보장하지
   않아 현재 `75:100`의 실측은 encoder 하드웨어 정상화 후 다시 계산한다.
5. ROS package test `67 passed`, standalone Xbox 계산 test `6 passed`, Python/YAML/
   shell 검사와 `colcon build --packages-select h753_can_odom --symlink-install`을
   통과했다.

화면 및 로그 진단:

1. 16:07 RViz 창을 직접 캡처했다. 당시 실행은 mapping이 아니라
   `Mode 5 INSPECTION_DRIVE`, `calibration.rviz`, `Fixed Frame=odom`이었다.
   이 설정에는 `/map` display가 없으므로 지도 절단 자체를 판정할 화면은 아니다.
2. `LaserScan`과 RViz Global Status는 `OK`였지만, 카메라 영상에서 양쪽 벽이
   대부분 유리인 구간의 `/scan`은 연속 벽선 대신 희소한 점과 짧은 조각으로
   보였다. 반복되는 긴 복도 형상에 유리의 투과·정반사가 겹쳐 원시 scan 피처가
   부족할 가능성이 높다. SLAM 파라미터만으로 존재하지 않는 벽점을 복원할 수는
   없으므로 먼저 유효 range 비율을 측정한다.
3. 같은 화면의 `/odom` `Keep=200` 궤적은 긴 직선 끝에서 부채꼴로 벌어졌다.
   Mode 5는 IMU fusion 없이 `can_odom_node`의 encoder odom을 직접 표시하므로,
   이 궤적은 Slam Toolbox의 `map -> odom` 보정 결과가 아니다.
4. 캡처 직후 정지 구간의 반복 `/odom` 표본은 위치와 quaternion이 모두 동일했다.
   이번 표본에서는 정지 중 지속 yaw drift가 없었다. 부채꼴이 물리적 직진 중
   생긴 것이라면 좌·우 encoder delta와 거리 환산계수를 먼저 검사한다.
5. 15:28 장시간 mapping 로그에서 Slam Toolbox 자체의 queue-full scan drop은
   시작 구간 2회였다. 별도로 global costmap은 TF cache보다 약 0.4초 오래된 scan을
   79회 버렸고 RViz queue-full도 16회 기록됐다. costmap/RViz 시간 지연은 고쳐야
   하지만, 현재 근거만으로 SLAM 입력 유실을 매핑 발산의 1차 원인으로 단정하지
   않는다.
6. 매핑 시작 gyro bias는 장시간 시험 `0.00520108 rad/s`, 16:02 단기 시험
   `0.00544543 rad/s`였다. 이전에 관찰한 비정상 후보 `-0.05728171 rad/s`와 달리
   이번 두 시작값은 서로 일관적이었다.

다음 단계는 한 번에 하나씩 진행한다.

| 순서 | 시험 | 판정 기준 |
| ---: | --- | --- |
| 1 | Mode 1/2에서 `/scan`, `/wheel/odom`, `/odom`, `/tf`, `/map`, `/cmd_vel_selected` 동시 기록과 RViz `Fixed Frame=map` 캡처 | 실제 지도 절단 시각과 원시 센서/odom 변화 시각을 일치시킨다. |
| 2 | 유리 복도 정지 scan의 전체 range, 유효 range, 좌·우 최장 연속점 수 계산 | 유리벽에서 원시 피처가 얼마나 사라지는지 수치화한다. |
| 3 | 물리적 직진 구간의 좌·우 encoder delta 비교 | `/wheel/odom`만 회전하면 encoder 환산, `/odom`만 회전하면 IMU fusion, 둘은 정상인데 map만 회전하면 scan matching 문제로 분리한다. |
| 4 | 같은 rosbag으로 지도 해상도 `0.025 m`와 `0.05 m` A/B | 원거리 430 points/scan의 공간 밀도와 map cell 불일치를 확인한다. |
| 5 | loop closure OFF 1회 후 sequential/loop matcher 임계값 A/B | 갑작스러운 전체 지도 변형과 누적 odom 오차를 분리한다. |
| 6 | Nav2 TF 지연을 별도 수정 | global costmap의 `earlier than all data in transform cache`를 0회로 만든다. |
| 7 | 앞 단계로 부족할 때 IMU filter, LaserScan deskew, 유리 대응 센서/환경 표식 검토 | 기존의 직진 pose 고정 문제를 재발시키지 않고 벽 이중선과 지도 절단을 줄인다. |

### 2026-07-25: Go2 정적 지도 AMCL 및 안전 주행 경로 추가

다른 로봇이 작성한 `/home/jyl1015/Downloads/maps/go2_map.yaml`과
`go2_map.pgm`을 확인했다. 표준 ROS occupancy map이며 크기는 `1093 x 1596`,
해상도는 `0.05 m/cell`, origin은 `[-33.6273, -50.1284, 0.0]`이다.

- 기존 Mode 3/4는 계속 `h753_map.posegraph/.data`를 사용하는 Slam Toolbox
  localization이다. Go2 PGM을 기존 `posegraph_file`로 바꾸지 않았다.
- Go2 지도만 시험하는 `go2_amcl_test_bringup.launch.py`와
  `run_go2_amcl_test.sh`를 추가했다. 이 경로는 `map_server + AMCL + YDLIDAR +
  CAN odom + RViz`만 실행한다.
- 모터 UART bridge, `/cmd_vel` 선택기, Nav2 controller는 시험 launch에 포함하지
  않았다. 따라서 이 실행만으로 로봇을 움직이지 않는다.
- AMCL은 현재 로봇 프레임 `map/odom/base_link/laser_frame`, Tmini range
  `0.03~12.0 m`, differential motion model을 사용한다. 다른 지도에서 저장된
  pose가 재사용되지 않도록 실행할 때마다 RViz `2D Pose Estimate`를 요구한다.
- 기본 AMCL은 `likelihood_field`, beam skipping OFF로 두었다. Go2 센서 높이와
  Tmini의 유리벽 반환 차이가 실측으로 확인된 뒤에만 beam skipping을 A/B한다.
- 격리 시험에서 map_server가 Go2 지도를 `1093 x 1596 @ 0.05 m/cell`로 읽었고,
  map_server와 AMCL 모두 lifecycle `active`까지 전환됐다.
- package test `67 passed`, 새 launch `ament_flake8`, Python/YAML/shell 문법과
  `git diff --check`를 통과했다.
- 제한된 진단 환경에서는 처음에 `/dev/ttyUSB0`과 CANable이 보이지 않았지만,
  호스트에서 재확인하니 YDLIDAR `/dev/ttyUSB0`, CANable `/dev/ttyACM1`, STM UART
  `/dev/ttyACM0`이 모두 정상 연결되어 있었다. 실행 스크립트는 실제 호스트에서
  장치 미연결 또는 기존 프로세스의 장치 사용을 검사하고 안전하게 중단한다.
- 실장치 비주행 시험에서 Tmini는 `10 Hz / 430 points/scan`, CANable은 encoder
  baseline 수신에 성공했다. 측정 발행률은 `/scan` 약 `9.98 Hz`, `/odom` 약
  `20.34 Hz`였고 map_server와 AMCL도 active 상태를 유지했다.
- 초기 pose를 보내지 않은 상태에서 AMCL이 `Please set the initial pose`를 출력한
  것은 의도한 동작이다. 다음 현장 단계는 RViz `2D Pose Estimate`로 Go2 지도상의
  실제 로봇 위치와 방향을 입력하고 scan/particle 수렴을 확인하는 것이다.
- 최초 현장 실행에서는 Go2 launch의 공개 `launch_rviz` 값이 내부에서 재사용한
  `mapping_bringup`의 `launch_rviz:=false`에 덮여 RViz 프로세스가 생성되지 않았다.
  RViz 조건을 내부 include보다 먼저 평가하도록 순서를 수정했고, 실제 장치가
  연결된 상태에서 RViz OpenGL 초기화와 Go2 `/map` 표시까지 확인했다.
- RViz에서 Go2 지도상의 초기 pose를 여러 번 조정했고 AMCL 로그의
  `initialPoseReceived`와 최종 `Setting pose`를 확인했다. 사용자가 RViz Navigation
  시작을 눌렀을 때 `navigate_to_pose action server is not available`이 나온 것은
  현재 시험이 localization 전용이며 planner/controller를 의도적으로 실행하지
  않았기 때문이다.
- Go2 지도 전용 수동 주행 경로 `go2_amcl_manual_drive_bringup.launch.py`와
  `run_go2_amcl_manual_drive.sh`를 추가했다. 명령은 `Xbox /joy ->
  /cmd_vel_selected -> collision_monitor -> /cmd_vel_safe -> UART` 순서로만
  전달된다. 일반 Mode 3/4의 posegraph 설정은 변경하지 않았다.
- 최초 저속 시험의 `0.30 m/s`, `1.10 rad/s` 제한은 실장 로봇에서 PWM이 약해
  움직이지 않아 해제했다. 현재 Go2 수동 주행도 실제 구동 최대치 `0.60 m/s`,
  `2.67 rad/s`를 사용하므로 풀 스틱에서 바깥 궤도 100% PWM에 도달한다. 주행 중
  최대 조향은 기존 보정값과 같은 안쪽:바깥쪽 궤도 `75:100`이며, 작은 non-zero
  입력에는 최종 UART의 궤도별 breakaway floor가 적용된다. 당시 값은 41%였으며
  2026-07-26 실측 이후 일반 50%, 제자리 회전 55%가 현재값이다.
- 수동 Go2 주행은 Xbox `LB`를 누르는 동안에만 활성화된다. LB 해제, Joy 0.5초
  timeout, scan timeout은 정지이며, LB를 누른 채 Joy가 끊기면 LB를 한 번 놓기
  전에는 재출발하지 않는다. `B`는 정지를 latch하고, LB를 놓은 뒤 `A`를 눌러야
  해제된다.
- 이 경로에서는 Nav2용으로 기본 비활성인 collision polygon을 별도 설정에서
  활성화했다. 전방 정지 영역과 감속 영역이 모두 실제 `/scan`을 사용한다.
- 실장치 정지 기동에서 Xbox, map_server/AMCL, Tmini, CAN odom, collision monitor,
  STM UART가 함께 활성화됐고 LB 미입력 동안 모터 명령은 정지 상태였다. UART를
  끈 격리 시험에서는 합성 LB+직진 입력 `0.15 m/s`가 `/cmd_vel_safe`로 전달된 뒤
  Joy timeout 시 0으로 바뀌는 것까지 확인했다. Ctrl+C 종료 시 모든 노드와 장치가
  clean exit했다.
- package test는 새 테스트 3개를 포함해 `70 passed`이며 build, flake8,
  Python/YAML/shell 문법 검사를 통과했다.
- 사용자 현장 시험에서 Go2-AMCL 원격 수동 주행 중 RViz의 로봇 pose와 scan이
  지도에 계속 정확히 일치하는 것을 확인했다. 주행 중 눈에 띄는 `map -> odom`
  점프나 지도 대비 위치 이탈이 없었으므로 Go2 지도 수동 localization 검증은
  통과로 판정한다. 이 항목은 현장 육안 판정이며 rosbag 정량 분석은 아직 아니다.
- 수동 localization 통과 후의 다음 단계로 `run_go2_amcl_navigation.sh`를 추가했다.
  이 실행은 같은 Go2 `map_server + AMCL` 위에 Nav2 controller/planner/behavior/
  BT navigator/velocity smoother를 올린다. localization lifecycle을 먼저 활성화하고
  collision monitor는 1.5초 뒤, Nav2는 5초 뒤 시작해 lifecycle manager 간 기동
  경합을 피한다.
- 주행 명령 경로는 `Nav2 /cmd_vel -> Go2 selector -> /cmd_vel_selected ->
  collision_monitor -> /cmd_vel_safe -> UART`이다. Nav2의 표준 전진 부호는 기존
  H753 UART/Mode 4 구동 부호에 맞춰 selector에서 변환한다.
- 자율주행 중 Xbox `LB`를 누르면 조이스틱이 Nav2보다 우선하고, LB를 놓으면 현재
  자율 목표를 계속 수행한다. LB를 누른 채 Joy가 끊기면 Nav2로 갑자기 복귀하지
  않고 정지한다. `B`는 모터 출력을 latch 정지하면서 `/navigate_to_pose`와
  `/navigate_through_poses` 목표를 취소한다. LB를 놓고 `A`로 latch를 해제한 뒤에는
  안전을 위해 새 목표를 보내야 한다.
- 하드웨어를 열지 않은 `ROS_DOMAIN_ID=99` 격리 기동 시험에서 Go2 지도
  `1093 x 1596 @ 0.05 m/cell`을 다시 읽었고 map_server, AMCL, collision monitor,
  controller, planner, behavior, BT navigator, waypoint follower, velocity smoother가
  모두 active로 전환됐다. `/navigate_to_pose`와 `/navigate_through_poses` 액션 및
  `/cmd_vel -> /cmd_vel_selected` 연결도 확인했다. 이 시험에서는 모터 명령이나
  실제 목표 주행을 수행하지 않았다.
- package build와 전체 test는 Nav2 선택/부호 변환, LB 우선권, stale Joy 차단,
  정지 latch 테스트를 포함해 pytest `76 passed`, colcon 기준 `79 tests,
  0 failures`이다. flake8, shell 문법과 `git diff --check`도 통과했다.
- 다음 현장 단계는 현재 실행 중인 수동 Go2 launch를 먼저 `Ctrl+C`로 종료한 뒤,
  아래 자율 실행을 단 하나만 시작하고 직선상 자유공간에 1~2 m의 짧은 Nav2 Goal을
  한 번 주는 것이다. Xbox `B` 정지를 준비한 상태에서 AMCL pose, local/global
  plan, `/cmd_vel -> /cmd_vel_selected -> /cmd_vel_safe`와 충돌 감시를 확인한다.
- 사용자가 Go2 지도에서 실제 Nav2 목표 주행이 정상 동작하는 것을 확인했다. 이후
  현장 피드백에 따라 곡선으로 크게 도는 경향과 벽 근처의 과도한 감속을 1차
  조정했다.
- `FollowPath` 앞에 `nav2_rotation_shim_controller::RotationShimController`를
  추가했다. 새 경로의 전방 0.35 m 방향과 로봇 heading 차이가 `0.35 rad`(약 20도)
  이상이면 직진을 섞지 않고 먼저 제자리 회전한다. `0.12 rad`까지 정렬한 뒤 기존
  DWB로 넘기며, 제자리 각속도는 `1.20 rad/s`, 각가속도는 `1.50 rad/s^2`이다.
  목표점 최종 yaw도 제자리 회전으로 맞춘다.
- Go2 collision monitor의 정지 polygon
  `[x -0.33~0.43 m, y -0.38~0.38 m]`은 변경하지 않았다. 감속 polygon만 전방
  `0.73 -> 0.65 m`, 측면 `+/-0.43 -> +/-0.40 m`, 후방 `-0.43 -> -0.38 m`로
  줄이고 `slowdown_ratio`를 `0.60 -> 0.80`으로 높였다. 따라서 평행한 벽 때문에
  감속되는 빈도와 감속량은 줄지만 기존 긴급 정지 여유는 유지된다.
- `ROS_DOMAIN_ID=99` 비하드웨어 기동에서 Rotation Shim 내부에 DWB가 생성되는
  로그와 controller_server를 포함한 전체 Nav2 lifecycle active를 확인했다. 새
  설정의 실주행 판정은 재기동 후 같은 경로에서 회전 전 `/cmd_vel`의 `linear.x`가
  0에 가깝고 `angular.z`만 출력되는지, 벽 근처 `/cmd_vel_safe`가 입력의 약 80%를
  유지하는지 확인하면 된다.
- Go2 단독 시험 경로를 일반 `run_robot_modes.sh`의 Mode 3/4에 통합했다. Mode 3/4의
  기본 `localization_backend`는 `amcl`이며 `map_server + AMCL + Nav2`가 workspace의
  `maps/go2/go2_map.yaml/.pgm`을 사용한다. Mode 1/2의 새 지도 작성과 Mode 6의 기존
  posegraph 이어그리기는 계속 Slam Toolbox를 사용하므로 매핑 알고리즘은 이번
  통합에서 변경하지 않았다.
- 원본 `/home/jyl1015/Downloads/maps/go2_map.pgm`을 `maps/go2/`로 이관했고 SHA-256
  `7ac42b120bc5029425f7fcf9cfb823d868197e34af063afc3b7114acd5a24b04` 일치를
  확인했다. YAML은 같은 디렉터리의 `go2_map.pgm`을 상대경로로 참조한다. 기존
  Go2 단독 실행 스크립트도 Downloads가 아닌 이 workspace 맵을 기본값으로 쓴다.
- Mode 3/4 진입 시 manager가 YAML뿐 아니라 YAML의 image 파일까지 검사한다.
  rollback은 `h753_robot_mode_manager.yaml`의 `localization_backend`를
  `slam_toolbox`로 바꾸면 되고, 이때 기존 `h753_map.posegraph/.data` 검사가 다시
  적용된다. Mode 6은 backend 설정과 관계없이 계속 posegraph를 요구한다.
- AMCL 최초 `/amcl_pose`가 오기 전 Mode 4의 Nav2 명령은 0으로 차단한다. 이때도
  Xbox LB 수동 인계는 사용할 수 있다. Mode 3과 4를 직접 전환해 카메라/YOLO 구성
  때문에 localization runtime이 재시작되면 마지막 AMCL pose를 새 `/initialpose`로
  재발행해 위치를 유지한다. Mode 0이나 mapping topology를 거쳐 들어오는 경우에는
  물리적 이동 가능성이 있으므로 자동 복원하지 않고 다시 `2D Pose Estimate`를
  요구한다.
- 일반 Mode 4는 검증된 `h753_go2_manual_collision_monitor.yaml`을 사용한다.
  Mode 4에서는 stop polygon과 80% slow polygon을 활성화하지만 Mode 2/6은 기존
  `h753_collision_monitor_modes.yaml`과 기존 stop 정책을 유지한다. RViz navigation
  설정에는 `/particle_cloud` 표시를 추가했다.
- 비하드웨어 Mode manager 통합 시험에서 Go2 맵 `1093 x 1596 @ 0.05 m/cell`을
  읽고 collision monitor, map_server, AMCL, controller, planner, BT navigator가
  active가 됐다. `/navigate_to_pose`, `/navigate_through_poses` 액션과 Rotation Shim
  내부 DWB 생성도 확인했다. manager 로그에서 초기 pose 전 Nav2 차단 및 합성
  `/amcl_pose` 수신 후 자율주행 허용 전환을 확인했다. 전체 시험은 pytest
  `86 passed`, colcon 기준 `89 tests, 0 failures`이다.
- 다음 현장 확인은 기존 Go2 단독 script가 아니라 `./run_robot_modes.sh`에서 Mode 3
  또는 Mode 4를 선택해 수행한다. 첫 진입에는 RViz `2D Pose Estimate`가 필수이며,
  동일한 짧은 목표에서 AMCL 정합, 제자리 회전, 벽 80% 감속, LB 인계, B 정지,
  Mode 4 YOLO/VLM 정지를 차례로 재확인한다.

### 2026-07-26: 경로 회전 지연 및 센서 주기 저하 1차 개선

- 최근 자율주행 로그에서 기본 NavigateToPose BT가 약 1초마다 새 전역 경로를
  controller에 전달했고, `Failed to make progress`와 controller/planner 주기 누락이
  함께 발생했다. 제자리 회전 중 이동 거리가 늘지 않아 15초 뒤 복구 동작으로
  넘어가는 현상도 확인했다.
- `SimpleProgressChecker`를 `PoseProgressChecker`로 변경했다. 기존 `0.15 m / 15초`
  조건은 유지하고 `required_movement_angle=0.20 rad`(약 11.5도)를 추가해 정상적인
  제자리 회전도 진행으로 인정한다.
- Rotation Shim의 `0.35 rad` 진입, `0.12 rad` 해제, `1.20 rad/s` 회전속도와 DWB
  설정은 변경하지 않았다.
- Nav2 Humble 기본 BT를 바탕으로 0.5 Hz 재계획 BT를 패키지에 추가했다. Mode 2/6의
  SLAM navigation, Mode 3/4의 저장 지도 navigation, Go2 단독 navigation launch가
  모두 설치된 사용자 BT를 기본 NavigateToPose 트리로 전달한다.
- planner의 `expected_planner_frequency`도 0.5 Hz로 맞췄다. 이 값은 실제 주기를
  만드는 파라미터가 아니라 2초를 넘는 계산을 경고하기 위한 진단 기준이며, 실제
  재계획 주기는 BT의 `RateController`가 정한다.
- 센서 데이터 토픽 주기가 전반적으로 낮아진다는 현장 피드백이 추가됐다. 재계획
  부하 감소가 센서 callback 지연에 미치는 영향을 함께 비교하되, `/scan`,
  `/wheel/odom`, `/odom`, 카메라 토픽을 같은 시점에 측정하기 전에는 센서 FPS나
  baudrate를 변경하지 않는다.
- 수정 전 실행 중인 Mode 4 계열 ROS graph를 12초간 동시에 측정한 기준값은
  `/scan 10.00 Hz`, `/wheel/odom 20.08 Hz`, `/odom 20.38 Hz`, D435i 결합 IMU
  `196.49 Hz`였다. 이 네 토픽은 각각의 현재 목표값을 거의 유지했다. RGB raw는
  `7.35 Hz`, aligned depth raw는 `8.98 Hz`로 15 Hz 설정보다 낮았지만, 하나의
  Python subscriber가 두 raw 영상과 IMU를 동시에 역직렬화한 측정이라 카메라
  publisher 저하로 단정하지 않고 각 영상 토픽을 독립 측정한다.
- 사용자 요청으로 Tmini 드라이버의 `frequency`를 `10.0 -> 12.0 Hz`로 변경했다.
  모든 H753 launch가 같은 패키지 `Tmini.yaml`을 기본값으로 사용하므로 재빌드 후
  Mode 1~6과 단독 LiDAR/Go2 시험에 공통 적용된다. 실행 중인 드라이버에는 동적
  반영하지 않으며 안전하게 mode 프로그램을 재시작한 뒤 `/scan` 실측으로 확인한다.
  `sample_rate=4 kHz`는 유지하므로 12 Hz에서는 한 회전당 점 수가 10 Hz보다 줄 수
  있다는 점을 벽 피처 시험에서 함께 확인한다.
- 영상 토픽을 각각 따로 10초 측정했을 때 RGB raw는 최종 평균 `12.59 Hz`, aligned
  depth raw는 `12.78 Hz`였다. 동시 구독 때의 7~9 Hz보다는 높지만 설정값 15 Hz에는
  못 미친다. 같은 로그에서 D435i는 USB 3.2, RGB/depth `640x480@15`로 정상 열렸고
  `Frames didn't arrive`나 frame-drop 오류는 기록되지 않았다.
- 로봇 프로세스 종료 직후 호스트는 load average `10.35`였고 NX 원격화면 인코더,
  Xorg, GNOME Shell 등 화면 계층의 CPU 사용이 컸다. 온도는 약 `67 C`로 thermal
  throttling 근거는 없었다. 이 스냅샷만으로 카메라 저하 원인을 확정할 수는 없지만,
  USB 설정 실패보다는 RViz/원격화면/영상 역직렬화와 Jetson scheduling 부하를 우선
  의심한다. 다음 재기동에서는 변경 전후 카메라 Hz와 controller rate-miss를 같은
  조건에서 다시 비교한다.
- `ydlidar_ros2_driver`와 `h753_can_odom`을 symlink-install로 재빌드했고 Nav2/센서
  설정 회귀 테스트를 포함한 `h753_can_odom` 전체 `89 passed`, XML/YAML/Python
  문법 및 `git diff --check`를 통과했다.
- 후방 목표 현장 재시험에서는 전진 목표 뒤 반대 방향 목표를 보냈을 때 전역 경로는
  0.5 Hz로 계속 전달됐지만, pose가 변하지 않아 15초마다 `Failed to make progress`가
  반복됐다. 이어진 recovery Spin도 10초 timeout으로 실패했다. 경로 생성 실패가
  아니라 제자리 회전 출력이 실장 궤도의 정지마찰을 넘지 못한 증상으로 분리했다.
- Rotation Shim의 제자리 회전을 `1.20 -> 1.60 rad/s`, 최대 각가속도를
  `1.50 -> 2.50 rad/s^2`로 높였다. 10 Hz 한 제어 step보다 작은 기존 `0.12 rad`
  해제 임계값은 고출력 회전의 overshoot를 줄이기 위해 `0.18 rad`로 조정했다.
- 추가 현장 확인에서 41% 출력으로는 감속 주행도 전혀 움직이지 않고 최소 50%부터
  조금씩 움직이는 것이 확인됐다. 따라서 모든 non-zero 궤도 breakaway floor를
  `41 -> 50%`로 올리고, `|linear.x| <= 0.02 m/s`인 제자리 회전은 정지마찰 여유를
  두어 55%를 적용했다. collision monitor의 감속 결과도 이 바닥값보다 낮으면
  50/55%로 보정되지만 stop polygon, timeout, VLM stop의 정확한 0은 그대로 유지된다.
- 강화된 회전/PWM 설정을 재빌드했고 관련 단위 테스트 `18 passed`, 패키지 전체
  `92 passed`, Python/YAML 문법과 `git diff --check`를 통과했다.
- 사용자 현장 피드백에 따라 제자리 회전을 한 단계 더 높였다. Rotation Shim의
  목표 각속도를 `1.60 -> 2.00 rad/s`, 제자리 회전 전용 최소 PWM을 `55 -> 60%`로
  조정했다. 10 Hz에서 한 주기의 최대 회전량 약 `0.20 rad`보다 작던 해제 임계값은
  overshoot와 방향 반전을 줄이기 위해 `0.18 -> 0.22 rad`로 높였다. 일반 직진/곡선
  최소 PWM 50%, 최대 각속도 2.67 rad/s, stop/timeout/VLM의 정확한 0은 유지한다.
- RViz costmap에서 파란색으로 보이는 inflation 영역을 local/global 모두
  `0.35 -> 0.30 m`로 5 cm 줄였다. 경로가 벽에서 불필요하게 멀어지는 정도만
  줄이며 실제 충돌 정지를 담당하는 collision monitor의 stop polygon은 변경하지
  않았다.
- 통합 `run_robot_modes.sh`에서 Xbox B는 latch만 거는 버튼이 아니라 Mode 0 STOP을
  선택하고 현재 Nav2 goal을 취소한다. 복귀 순서는 `LB 해제 -> Menu 짧게 누르기 ->
  D-pad로 원래 mode 선택 -> A`이며, Mode 4는 복귀 후 새 goal을 보내야 한다. Go2
  단독 manual/navigation script의 별도 latch는 `LB 해제 -> A`로 해제한다.
- inflation 변경을 재빌드했고 설정 회귀 테스트를 포함한 패키지 전체 `93 passed`,
  YAML 문법과 `git diff --check`를 통과했다.
- 낮은 턱을 자동 판별하는 센서 로직이 아직 없는 상태에서 충격을 줄이기 위해 Nav2
  자율주행 상한을 구동 최대치의 90%로 낮췄다. DWB와 velocity smoother의 선속도는
  `0.60 -> 0.54 m/s`, 각속도 및 recovery 최대 회전은 `2.67 -> 2.40 rad/s`다.
  Rotation Shim의 목표 회전 `2.00 rad/s`는 이 상한 안에서 유지한다. UART와 Xbox
  LB 수동 개입의 `0.60 m/s / 2.67 rad/s` 하드웨어 상한은 변경하지 않았다.
- 직진과 회전이 섞인 명령의 바깥쪽 궤도까지 90%로 제한하기 위해 통합 mode
  manager와 Go2 단독 navigation selector에 좌·우 궤도 합성 상한을 추가했으나,
  각속도가 바뀔 때 선속도까지 함께 재조정되어 현장 움직임이 불안정해졌다. 같은 날
  이 합성 상한은 롤백했다. 현재 자율주행은 최초 90% 정책인 Nav2 선속도
  `0.54 m/s`, 각속도 `2.40 rad/s`의 성분별 상한만 사용하며, UART와 Xbox LB 수동
  개입은 기존 `0.60 m/s / 2.67 rad/s` 상한을 유지한다.
- 2D LiDAR는 스캔 평면 아래의 낮은 턱 높이를 직접 측정할 수 없다. 후속 구현은
  D435 depth의 전방 하단 ROI에서 바닥 평면 대비 높이 차를 계산하고, 통과 가능한
  턱이면 Nav2 `/speed_limit`을 발행해 감속하며 한계보다 높으면 stop을 요청하는
  방식으로 진행한다. 높이 임계값은 로봇이 넘을 수 있는 턱을 단계별 실측한 뒤
  정한다.
- 자율주행 90% 상한을 재빌드했고 설정 회귀 테스트를 포함한 패키지 전체
  `97 passed`, YAML 문법과 `git diff --check`를 통과했다.
- Mode 5 점검주행은 정상인데 Mode 4에서 Xbox LB 수동 인계가 움직이지 않는 현장을
  토픽별로 분리했다. `/joy`의 LB(`buttons[4]=1`)와 `/cmd_vel_selected`
  `linear.x=-0.5867 m/s`, `/safety/vlm_stop=0`까지는 정상이었지만
  `/collision_monitor=inactive`이고 `/cmd_vel_safe`가 발행되지 않았다. UART bridge가
  stale 안전명령을 정지로 처리한 것이 직접 원인이며 90% 속도 상한과는 무관하다.
- 당시 launch 로그에는 collision monitor와 lifecycle manager 동시 기동 중
  `/collision_monitor/change_state` 응답 timeout이 기록됐다. 통합 AMCL navigation,
  SLAM navigation, Mode 5 inspection 모두 collision node를 먼저 띄운 뒤 lifecycle
  manager를 1.5초 지연 실행하도록 변경했다. `ROS_DOMAIN_ID=99`,
  LiDAR/odom/UART/RViz 비활성 격리 시험에서
  `Configuring -> Activating -> Managed nodes are active`를 확인했다.
- lifecycle 기동 순서 회귀 테스트를 추가했고 패키지 전체 `98 passed`, build와
  `git diff --check`를 통과했다. 이미 실행 중인 Mode 4는 예전 launch 순서를 메모리에
  가지고 있으므로 안전하게 종료하고 재시작해야 수정이 적용된다.
- VLM 완료 결과가 보이지 않았는데 자동 재출발한 현상은 로봇 측 자동 해제가
  아니었다. 서버 `vlm_node`가 `/vlm/injury_stop=0`을 실제로 두 번 보냈고 gateway와
  UART 로그 모두 같은 시각에 `Validated VLM stop cleared`를 기록했다. 현재 계약은
  `/vlm/result`나 DB 저장 ACK와 무관하게 이 0 하나만으로 해제한다. 서버 문서의
  `CONFIRMED -> COOLDOWN` 자동 0 정책이 원인이므로, 후속 안전 개선은 detection ID와
  저장 완료 ACK 또는 GUI 운영자 승인 뒤에만 0을 허용하는 것이다.
- 텔레옵 끊김 진단에서 `/joy` 약 17 Hz, `/cmd_vel_selected` 약 20.1 Hz, `/scan`
  약 12.2 Hz로 입력은 유지됐다. 반면 collision monitor가 짧은 간격으로
  `PolygonStop / PolygonSlow / normal`을 반복 전환했고 UART에는 `cmd_vel timeout`과
  `write timeout`이 기록됐다. 조이스틱 문제가 아니라 polygon 경계 흔들림과 안전
  출력/UART 공백이 겹친 현상이며, 시간 임계값만 늘리기 전에 연속 scan 또는
  hysteresis와 `/cmd_vel_safe` 공백을 별도로 개선한다.
- 카메라 데이터 목표 주기를 15 Hz로 통일했다. 모든 RealSense 프로파일은
  `640x480x15`를 유지하고 RGB/depth QoS를 `SENSOR_DATA`, frame queue를 2로 바꿔
  느린 구독자가 센서 publisher를 막지 않고 최신 frame을 우선하도록 했다. YOLO는
  제어 부하를 키우지 않도록 추론 5 Hz를 유지하되 최신 annotated JPEG를 별도
  callback group에서 15 Hz로 발행한다. 따라서 영상 토픽은 15 Hz이지만 bounding box
  판정 갱신은 최대 5 Hz이며 추론 사이에는 최신 결과 frame이 반복된다.
- 변경 전 실측은 RGB compressed 약 14.3 Hz, RGB raw 약 11.4 Hz, aligned depth
  약 14.0 Hz, YOLO annotated 약 4.3 Hz였다. 변경 후 실제 수신률은 실행 중인 이전
  프로세스를 재시작한 다음 서버 PC에서도 다시 측정해야 한다. 두 패키지 build,
  flake8, YAML/config 회귀 테스트와 전체 `h753_can_odom 99 passed`,
  `h753_perception 4 passed`를 통과했다.
- 오늘 추가했던 자율주행 좌·우 궤도 합성 90% 상한은 각속도 변화 때 선속도도 같이
  축소해 움직임이 불안정하다는 현장 피드백에 따라 제거했다. 최초 90% 설정인 Nav2
  선속도 `0.54 m/s`, 각속도 `2.40 rad/s` 성분별 상한은 유지한다. Mode 4의
  `normal / PolygonSlow(80%) / PolygonStop` 3단계는 전날부터 있던 별도 안전 기능이며
  히스테리시스가 없어 경계 scan 점 수에 따라 반복 전환될 수 있다. 복구 후
  `h753_can_odom` build와 핵심 회귀 `53 passed`, 패키지 전체 `97 passed`
  (`colcon test-result`: 101 tests, 오류/실패 0)를 확인했다.
- 복구 후에도 Mode 4가 툭툭 끊기는 현장을 재측정했다. `/scan` 94개에서 라이다
  중심 약 `0.03 m`, 후방 `133~170 deg`의 근거리 점이 매 scan `5~21개` 발생했고,
  Go2 collision polygon 임계값인 slow `>5`, stop `>10`을 반복 통과했다. 실제 로그도
  `normal / PolygonSlow / PolygonStop`이 약 50~150 ms 간격으로 전환됐으므로 PWM
  부족이나 Nav2 90% 상한이 아니라 Mode 4에만 켜진 3단계 polygon 개입이 직접
  원인이었다.
- 사용자 요청에 따라 통합 Mode 4의 collision 정책을 정상 동작한 Mode 5 기준으로
  통일했다. collision monitor 프로세스와 lifecycle, `/cmd_vel_safe` deadman 경로는
  유지하되 통합 모드는 `h753_collision_monitor_modes.yaml`의 비활성 stop/slow
  polygon을 사용한다. 따라서 Nav2 costmap이 장애물 회피를 담당하고 추가 3단계
  속도 전환은 하지 않는다. 자율주행 `0.54 m/s / 2.40 rad/s` 90% 상한, 일반 최소
  PWM 50%, 제자리 회전 60%, scan/VLM/timeout의 정확한 정지는 그대로 유지한다.
- 첫 Mode 5 정책 적용 뒤 Mode 4가 아주 조금씩만 움직이는 회귀가 발생했다. launch는
  비활성 polygon YAML을 올바르게 읽었지만 mode manager의 기존 동적 safety sync가
  Mode 4 진입 후 `PolygonStop.enabled=true`, `PolygonSlow.enabled=true`를 다시 써서
  실제 파라미터를 덮어썼다. 현장 런타임 값이 둘 다 true이고 로그에서 stop과 60%
  slow가 반복된 것을 확인했다. 동적 sync는 lifecycle 활성 확인과 자율주행 gate
  역할은 유지하되, 모든 통합 mode에서 stop/slow polygon 값을 항상 false로 설정하도록
  수정했다. 실행 중 goal이 갑자기 재출발하지 않도록 live parameter는 변경하지 않고
  안전 정지 후 재시작 적용한다. 수정 후 핵심 회귀 `47 passed`, 패키지 전체
  `h753_can_odom 99 passed` (`colcon test-result`: 104 tests, 오류/실패/skip 0),
  build와 `git diff --check`를 통과했다.
- polygon false 적용 후 Mode 4 원격주행이 전혀 움직이지 않는다는 후속 현장은 코드
  차단이 아니었다. `/joy`에서 LB=1과 전진축 전 범위 입력은 정상인데 selected가 계속
  0이어서 manager 로그를 확인했고, 안전 시험 중 Xbox B를 누른 시각에 실제 상태가
  `Mode 0 STOP`으로 바뀐 뒤 Mode 4로 재선택되지 않은 상태였다. `/robot_mode`도 0,
  두 polygon은 false를 확인했다. B는 goal 취소와 Mode 0 선택을 동시에 수행하므로
  `LB 해제 + 스틱 중립 -> Mode 4 재선택 -> 새 자율 goal 또는 LB 수동주행` 순서가
  필요하다. 실제 Mode 4로 복귀한 다음에만 남은 주행 끊김을 재측정한다.
- Mode 4 제자리 회전이 느리다는 현장 피드백에 따라 Rotation Shim 목표 각속도를
  `2.00 -> 2.40 rad/s`로 올려 기존 자율주행 90% 각속도 상한을 전부 사용한다.
  10 Hz에서 한 제어 주기의 회전량이 약 `0.24 rad`가 되므로 고속 pivot overshoot를
  줄이기 위한 disengage 임계값도 `0.22 -> 0.26 rad`로 맞췄다. 직진 상한
  `0.54 m/s`, 일반 최소 PWM 50%, 제자리 회전 최소 PWM 60%, 수동 LB 최대
  `2.67 rad/s`, VLM/scan/timeout 정지는 변경하지 않았다. 설정 회귀 `13 passed`,
  패키지 전체 `h753_can_odom 99 passed` (`colcon test-result`: 104 tests,
  오류/실패/skip 0)와 symlink-install build를 통과했다.
- Mode 5에서 YOLO 카메라 창이 보이지 않는 현상을 분리했다. 실행 파라미터는
  `show_window=true`, `tuning_mode=true`였고 RGB/depth publisher와 YOLO subscriber는
  모두 `BEST_EFFORT`로 일치했으며 `/yolo/detected_image/compressed`도 실측 15 Hz였다.
  다만 15 Hz 영상 출력을 위해 추가한 `MultiThreadedExecutor`의 worker callback에서
  OpenCV `imshow/waitKey`를 호출해 GUI event loop가 첫 frame 뒤 멈췄다. 추론과 영상
  토픽은 worker에서 계속 처리하되 annotated/mask 최신 frame만 공유하고, main loop가
  `spin_once(0.02 s)` 사이에 trackbar, `imshow`, `waitKey`를 처리하도록 변경했다.
  Mode 4/5 수정 후 두 패키지를 symlink-install로 재빌드했고 전체
  `h753_can_odom 98 passed`, `h753_perception 5 passed`
  (`colcon test-result`: 103 tests, 오류/실패/skip 0)와 `git diff --check`를 통과했다.

### 2026-07-27: LaserScan TF 지연 원인 분리 및 라이다 안정 프로파일 복구

- 자율주행 제자리 회전은 먼저 Rotation Shim과 DWB 각속도 상한을
  `2.40 rad/s`로 맞춰 하드웨어 최대 `2.67 rad/s`의 약 90%를 사용했다.
  후속 현장 요청으로 제자리 회전은 아래와 같이 100%로 추가 상향했다.
- 텔레옵은 자율주행 속도 상한을 적용하지 않는다. Mode 1·3·5 수동 주행과
  Mode 2·4·6의 Xbox LB 수동 인계, Go2 단독 수동 주행은 모두
  `0.60 m/s / 2.67 rad/s`를 사용한다. UART bridge의 궤도 상한도
  `0.60 m/s = PWM 100%`로 같으며, 풀스틱 직진·제자리 회전은 100%까지
  전달된다. 주행 중 회전은 외측:내측 궤도의 의도한 `100:75` 비율을
  유지한다.
- 실행 중 `odom -> laser_frame` TF는 정상 조회되어 static laser TF 누락을
  배제했다. 반면 동일 시간대 로그에서 YDLIDAR `Failed to get scan`, checksum
  오류와 약 1초의 scan 공백, wheel odom `dt out of range`, controller loop miss가
  함께 발생했다. RViz `Message Filter dropping message ... queue is full`과
  map/odom future·old transform 오류는 이 전체 시스템 지연의 결과로 분리했다.
- Tmini `12 Hz` 프로파일에서 위 공백과 fixed point count 불일치가 반복되어
  실측 안정성이 확인됐던 `10 Hz`로 복구했다. TF tolerance나 RViz queue만
  늘려 scan/odom 공백을 가리지 않는다. 재시작 후 `/scan`이 약 10 Hz로
  유지되고 `Failed to get scan`, `Transform data too old`, RViz queue drop이
  재발하지 않는지 같은 주행 조건에서 확인한다.
- `ydlidar_ros2_driver`와 `h753_can_odom`을 symlink-install로 재빌드했고,
  텔레옵/자율주행/라이다 설정 회귀 `55 passed`, `h753_can_odom` 전체
  `99 passed`를 통과했다. YDLIDAR 벤더 패키지의 기존 소스 헤더·서식
  lint 6개는 실패했으나, 이번에 수정한 `Tmini.yaml`과 무관한 기존 파일이다.
- 자율주행 직진 최고속도만 구동 최대의 `80%` (`0.48 m/s`)로 낮췄다.
  제자리 회전은 `100%` (`2.67 rad/s`), 이동 중 DWB 회전은 기존 90%에서
  약 `95%` (`2.54 rad/s`)로 높였다. 회전이 천천히 올라가는 현상을 줄이기 위해
  Rotation Shim 가속도를 `2.50 -> 3.00 rad/s²`, DWB를 `2.50 -> 2.80 rad/s²`,
  recovery Spin을 최소 `0.15 -> 0.20 rad/s`, 가속도 `0.80 -> 1.50 rad/s²`로
  함께 높였다. 텔레옵/LB 수동 인계는 직진·회전 모두 `100%`를
  유지하고, 일반 최소 PWM 50%, 제자리 회전 PWM 60%, VLM/scan/timeout
  정지 로직은 변경하지 않았다. 속도 경로 회귀 `55 passed`, 패키지 전체
  `99 tests, 0 errors, 0 failures, 0 skipped`와 symlink-install 빌드를 통과했다.
- VLM 동일 인물 재판정 억제 후 로봇이 다시 정지된 채 해제되지
  않는 현상을 신호 순서로 분리했다. 두 번째 정지는 VLM이 아니라
  Jetson gateway의 `On-board YOLO stop asserted` 로컬 0->1 edge가 걸었고,
  VLM은 COOLDOWN에서 재판정을 무시해 추가 clear 0을 보내지 않았다.
  실제 로그에서 재정지 후 clear까지 약 153초 공백을 확인했다.
- `h753_vlm_gateway`에 `LocalDetectionRearmGate`를 추가했다. 최초 armed
  YOLO 0->1은 기존처럼 즉시정지하지만, 서버의 명시적인 stop 0으로
  한 번의 판정이 해제되면 로컬 YOLO 정지를 잠근다. 이후 `15초`
  쿨다운과 `/yolo/person_found`, `/yolo/blue_person` 모두 `2초` 연속 0을
  만족할 때만 재무장한다. 잠금 중 같은 인물의 0->1 흔들림은
  정지를 다시 발행하지 않지만, 서버의 명시적 stop 1과 기존
  scan/cmd_vel/joystick 안전 정지는 그대로 적용된다.
- 서버 원본이 발행하는 `/vlm/status`의 `detection_armed`를 gateway가 구독해
  서버와 로컬 재무장 시점을 동기화한다. 서버 상태가 `false`인 동안에는
  로컬 조건이 먼저 충족돼도 재무장하지 않는다. 서버 상태가 3초 넘게
  끊기거나 구버전 서버라 상태 토픽이 없을 때만 로컬 `15초 + clear 2초`
  조건으로 fallback한다. gateway 자체 상태는 충돌을 피하려고
  `/vlm/gateway/status`로 분리했으며 `local_detection_armed`, 남은 cooldown,
  clear 유지 시간, YOLO gate별 0/1, 서버 상태와 수신 age를 포함한다.
- YOLO에 `DetectionClearHold(2.0초)`를 적용했다. raw 인물 감지 1은
  즉시 출력하지만 0은 2초 연속 미감지 후에만 발행해 confidence/depth
  한두 frame 흔들림으로 거짓 1->0->1이 생기는 것을 줄였다.
  `blue_person`도 stable 값 변경 시에만 발행하고 status JSON에 raw/stable
  값을 모두 남긴다.
- Desktop에 전달된 서버 PC 원본 `vlm.py`를 검토했다. 이미 COOLDOWN 중
  `/vlm/injury_stop=0`을 1 Hz로 유지하고 `/vlm/status`에
  `detection_armed`, cooldown 남은 시간, clear 유지 시간을 발행한다.
  사용자 요청에 따라 이 서버 파일은 수정하지 않았고, Jetson gateway만
  기존 서버 계약을 소비하도록 맞췄다.
- 두 패키지 symlink-install 빌드, 핵심 회귀 `39 passed`, 전체
  `h753_can_odom 109 passed`, `h753_perception 9 passed`, 변경 파일 6개 flake8과
  `git diff --check`를 통과했다. 격리 테스트의 NVIDIA device/EGL 경고는
  GPU를 열지 않는 CPU 단위 테스트 환경 메시지이며 테스트 실패는 아니다.

지도만 화면에 표시하는 비하드웨어 시험:

```bash
./run_go2_amcl_test.sh launch_lidar:=false launch_odom:=false
```

장치 연결 후 정지 위치추정 시험:

```bash
./run_go2_amcl_test.sh
```

장치 연결 후 Xbox 수동 저속 주행:

```bash
./run_go2_amcl_manual_drive.sh
```

수동 localization 통과 후 Go2 지도 기반 짧은 Nav2 목표 주행:

```bash
./run_go2_amcl_navigation.sh
```

자율 실행에서는 먼저 RViz `2D Pose Estimate`를 다시 맞추고 scan/particle 수렴을
확인한다. 로봇 정면의 장애물 없는 직선 구간에 1~2 m 목표 하나만 지정한다. LB를
누르지 않은 상태가 Nav2 주행이며, LB는 수동 개입, `B`는 목표 취소와 정지 latch,
LB를 놓은 뒤 `A`는 latch 해제다. A만 눌러서는 이전 목표가 재개되지 않으므로 새
목표를 보내야 한다.

RViz에서 먼저 `2D Pose Estimate`를 맞춘다. 그 다음 주변이 비어 있는지 확인하고
`LB`를 누른 상태에서만 스틱을 움직인다. `B`는 즉시 정지 latch이고, 다시 주행할
때는 `LB`를 놓은 상태에서 `A`로 latch를 해제한다. 기존 `run_robot_modes.sh`와
`run_xbox_uart_drive.sh`는 장치 소유권이 겹치므로 동시에 실행하지 않는다.

RViz가 열리면 실제 위치와 방향을 Go2 지도에서 찾아 `2D Pose Estimate`로 지정한다.
초기 통과 기준은 `/map` 표시, `/scan`과 벽의 중첩, 녹색 AMCL particle 수렴,
`map -> odom -> base_link` TF 연속성이다. 이 네 항목을 통과하기 전에는 Go2
지도를 Mode 4 자율주행에 연결하지 않는다.

## 4. 주요 ROS 실행 모드

| Mode | 용도 | 명령 소스 |
| --- | --- | --- |
| 0 | Stop | 강제 zero |
| 1 | 수동 mapping | Xbox/키보드 |
| 2 | 자동 frontier mapping | Nav2, LB 유지 시 Xbox 우선 |
| 3 | 저장 지도 수동 localization | Xbox/키보드 |
| 4 | 저장 지도 goal navigation | Nav2, LB 유지 시 Xbox 우선 |
| 5 | 센서/구동 inspection | Xbox/키보드 |
| 6 | 저장 지도 기반 재난 재탐사 | Frontier/Nav2, LB 유지 시 Xbox 우선 |

자율 모드 2/4에서는 Xbox `LB`를 누르고 있는 동안에만 조이스틱 명령이
Nav2보다 우선한다. LB를 놓으면 현재 Nav2 goal을 취소하지 않고 자율주행으로
복귀한다. LB가 눌린 상태에서 `/joy` 입력이 끊기면 자동으로 복귀하지 않고
속도 명령을 0으로 유지한다. `B`와 Menu 2초 유지는 기존처럼 강제 정지다.
LB 수동 인계는 자율 안전 파라미터의 준비 상태와 무관하게 먼저 적용되지만,
출력은 계속 collision monitor와 UART의 scan/VLM/deadman guard를 통과한다.
collision monitor lifecycle/parameter 요청은 2초 안에 응답하지 않으면 재시도해
pending 상태가 영구 고정되지 않도록 한다.

모든 주행 mode 1~6은 궤도 정지마찰을 확실히 넘도록 동일한 주행 한계
`0.60 m/s`, `2.67 rad/s`를 사용한다. Mode 2/4/6의 Nav2 명령과 LB 수동 인계도
같은 정책을 사용한다. 저속 매핑용 mode별 clamp는 없으며, 최종 UART의 차체/track
속도 제한과 collision monitor, scan timeout, VLM stop 안전 계층은 그대로다.

mode 2는 저장 파일의 존재와 관계없이 항상 빈 posegraph에서 새 지도를 만든다.
mode 6은 `posegraph_file`의 `.posegraph`와 `.data`를 불러오고 첫 mapping 시작
위치(dock)를 기준으로 live mapping을 이어간다. 따라서 mode 6을 선택하기 전에
로봇을 원래 지도를 만들기 시작했던 위치와 방향에 놓아야 한다. 기존 지도에
unknown 영역이 남아 있으면 frontier가 자동 목표를 만들며, 완성된 지도처럼
frontier가 없으면 RViz Nav2 Goal 또는 LB 수동 인계로 기존 복도를 재탐사한다.
재난 전 원본을 보존하려면 결과는 반드시 다른 이름으로 저장한다.

```bash
./save_slam_map.sh maps/h753_map_after_earthquake
```

자동 frontier 목표는 free/unknown 경계 셀에 직접 놓지 않고 알려진 자유공간
안쪽으로 `0.35 m` 이동시켜 footprint가 미지 영역과 겹치지 않게 한다. Nav2가
일시적으로 action을 거절한 위치는 blacklist하지 않고, 실제 no-path 후보만
반경 `0.35 m`, `30초` 동안 제외한다. Jetson 부하로 lifecycle bond가 끊기지
않도록 Nav2 `bond_timeout`은 `15초`로 전달한다. local/global costmap의
초록색 planning footprint는 x축 길이 `0.35 m`, y축 폭 `0.40 m`이며,
costmap inflation radius는 현재 `0.30 m`다. collision monitor의 별도
정지·감속 안전 영역은 이 footprint보다 보수적으로 크게 유지한다.

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

## 4.5 정지(safety stop) 토픽 정리 (2026-07-16)

`h753_perception`(보드 YOLO)과 `h753_vlm_gateway`(보드↔노트북 VLM 게이트웨이)가
개입하는 모든 토픽을 하나로 정리한다. 실행 위치는 `보드`(Jetson,
`ros2_graduation_project_ws`) 또는 `노트북`(GPU 서버, 팀원 저장소
`jjproject_5_260716`의 `vlm.py`)이다.

| 토픽 | 타입 | 발행 | 구독 |
| --- | --- | --- | --- |
| `/camera/camera/color/image_raw` | Image | RealSense driver (보드) | `h753_perception` |
| `/camera/camera/aligned_depth_to_color/image_raw` | Image | RealSense driver (보드) | `h753_perception` (거리 계산) |
| `/camera/camera/color/image_raw/compressed` | CompressedImage | RealSense driver (보드) | `h753_vlm_gateway`(중계용), `vlm.py`(노트북, 직접 구독) |
| `/yolo/person_found` | Int32 | `h753_perception` (보드) | `h753_vlm_gateway`(보드, 즉시정지), `vlm.py`(노트북, indoor 게이트), mode manager(로그) |
| `/yolo/blue_person` | Int32 | `h753_perception` (보드) | `h753_vlm_gateway`(보드, 즉시정지), `vlm.py`(노트북, outdoor 게이트, 기본값) |
| `/yolo/target`, `/yolo/nearest_distance_m`, `/yolo/status`, `/yolo/detected_image/compressed` | 각각 다름 | `h753_perception` (보드) | 모니터링용, 정지 로직과 무관 |
| `/robot_mode` | UInt8 (latched) | `robot_mode_manager` (보드) | `h753_vlm_gateway` (활성 모드 3/4/5 판단) |
| `/vlm/request/image/compressed` | CompressedImage | `h753_vlm_gateway` (보드) | 원격 VLM 서버용 alias (현재 서버는 대신 `image_raw/compressed`를 직접 구독) |
| `/vlm/injury_stop` | Int32 | `vlm.py` (노트북) — 이제 판정 확정(`CONFIRMED`→1)·해제(`NORMAL`/쿨다운→0)만 발행 | `h753_vlm_gateway` (보드) |
| `/vlm/result`, `/vlm/result_detail` | String / String(JSON) | `vlm.py` (노트북) | `h753_vlm_gateway` → `/vlm/gateway/result(_detail)`로 재발행 |
| `/vlm/status` | String(JSON) | `vlm.py` (노트북) | `h753_vlm_gateway` (서버의 `detection_armed`, cooldown, clear 상태 동기화) |
| `/vlm/gateway/status` | String(JSON) | `h753_vlm_gateway` (보드) | GUI/터미널 모니터링 (정지 latch, 로컬·서버 재무장, 연결/지연 상태) |
| **`/safety/vlm_stop`** | Int32 (latched) | `h753_vlm_gateway` (보드) — ① YOLO 즉시정지(로컬), ② 서버 `injury_stop` 검증·중계(원격) 두 경로 병합 | `h753_cmd_vel_uart_bridge` (보드) |
| `/cmd_vel_safe` | Twist | collision monitor 등 | `h753_cmd_vel_uart_bridge` → STM UART |
| `/scan` | LaserScan | YDLIDAR driver (보드) | `h753_cmd_vel_uart_bridge` (stale guard) |

정지/해제 규칙 (`VlmSafetyGate`, `vlm_gateway_node.py`):

- **정지(OR)**: `/yolo/person_found` 또는 `/yolo/blue_person`이 `0→1`이 되거나,
  서버가 `/vlm/injury_stop=1`(또는 유효하지 않은 값)을 보내면 걸린다. 어느
  쪽이 먼저 오든 `/safety/vlm_stop=1`.
- **해제**: 오직 서버의 명시적 `/vlm/injury_stop=0`으로만 풀린다. YOLO
  신호가 다시 0이 되는 것만으로는 풀리지 않는다 — 부상 여부 최종 판단은
  항상 VLM(노트북)이 갖는다.
- **적용 모드**: `h753_vlm_gateway`는 mode `3/4/5`에서만 활성화. `h753_perception`
  (YOLO)은 mode `4/5`에서만 실행되므로 온보드 즉시정지는 사실상 4/5에서만
  동작한다. 정지가 걸린 뒤에는 모드가 벗어나도 서버가 `0`을 보낼 때까지
  통신·정지 상태를 유지한다.
- **가장 중요한 변경점**: 예전에는 YOLO 감지 즉시-정지 판단 자체를
  노트북(`vlm.py`)이 내리고 그 결과를 다시 보드로 돌려받는 방식이었다(보드→
  노트북→보드 왕복). 지금은 그 판단이 필요로 하는 데이터(`/yolo/*`)가 이미
  보드 안에 있으므로 `h753_vlm_gateway`가 로컬로 직접 정지시킨다. 노트북/VLM은
  "이게 진짜 부상 상황인지, 다시 출발해도 되는지"만 판단하는 역할로 좁혀졌다.
  Wi‑Fi/서버 단절 중에도 온보드 감지 즉시정지는 계속 동작한다(단, 그 상태의
  해제는 서버 복구 전까지 불가능하다 — fail-safe 방향으로 유지).

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
- VLM 서버와 로봇은 `ROS_DOMAIN_ID=30`을 사용한다. 현재 서버는 기존
  `/camera/camera/color/image_raw/compressed` 구독을 그대로 유지해도 된다.
  이 카메라 publisher는 로봇 workspace가 소유한다.
- 서버는 `/vlm/injury_stop`을 발행하지만 UART bridge는 이 토픽을 직접 구독하지
  않는다. gateway가 검증한 `/safety/vlm_stop`만 사용한다.
- 서버 연결이 끊겨도 이미 수신한 stop 상태는 명시적인 `0`까지 유지된다.
- `~/yolo_project/test_run.launch.py` 또는 `yycam`을 로봇 mode 프로그램과 함께
  실행하면 D435i 소유권이 중복되므로 사용하지 않는다.
- YOLO는 `run_robot_modes.sh`에서 mode 4 또는 5를 선택하면 자동 실행된다.
  별도 `~/yolo_project/run.sh`를 동시에 실행하지 않는다.
- H753 Nucleo `LD4 COM`의 빨간색/점멸은 ST-LINK 통신 활동이며 CAN 오류 표시가
  아니다. overcurrent LED는 `LD6`이다.
- PID는 아직 비활성화 상태다. encoder/SLAM 검증 전에 활성화하지 않는다.
- Xbox 도구용 `.venv`는 정리했으므로 standalone 도구 사용 시 다시 생성한다.

## 7. 다음 우선순위

1. 필터 적용 전 D435i raw IMU와 wheel odom의 정지/직진/회전 rosbag 기준
   데이터를 수집한다.
2. PSD/Allan deviation 분석으로 진동 주파수와 회전 속도별 noise variance를
   구하고 Hampel/Butterworth 후보 파라미터를 확정한다.
3. 필터를 기본 비활성 파라미터 뒤에 구현하고 pure-function unit test와
   raw/filtered topic A/B 검증을 먼저 수행한다.
4. 360도 회전과 직사각형 SLAM loop로 filtered IMU 및 adaptive fusion을
   단계적으로 검증한다.
5. mode 2/4/6에서 `/cmd_vel -> /cmd_vel_selected -> /cmd_vel_safe -> UART`를 확인하고
   collision monitor의 감속/정지를 실공간에서 검증한다.
6. 저장 map localization과 Nav2 goal 주행을 검증한 뒤 frontier exploration을
   시험한다.
