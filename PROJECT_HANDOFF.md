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
  -> /scan + IMU + 640x480@15 RGB
  -> SLAM Toolbox / Nav2 + h753_vlm_gateway

h753_vlm_gateway
  -> optional /vlm/request/image/compressed
  <- /vlm/injury_stop, /vlm/result, /vlm/result_detail
  -> validated /safety/vlm_stop -> UART bridge
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
  - 연결/지연 상태: `/vlm/status` (`std_msgs/msg/String`, JSON)
  - gateway는 mode `3/4/5`에서만 활성화되고 mapping mode `1/2/6`에서는 제외된다.
  - `0`: 주행 허용
  - `1` 또는 잘못된 non-zero 값: UART 모터 명령 강제 정지
  - publisher가 없거나 아직 메시지를 받지 않은 경우: 인터록 제외, 기존 주행 허용
  - 한 번 정지되면 서버가 명시적으로 `0`을 보낼 때까지 정지 상태 유지
  - 정지 중에도 영상 전달과 VLM 탐지는 `0`을 받을 때까지 계속한다.
- D435i normal profile: color `640x480@15`, depth/align/pointcloud 비활성화
- RTAB-Map 전용 profile에서만 depth와 align을 활성화한다.
- Nav2 inflation radius: `0.35 m`
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
   `/yolo/detected_image/compressed`이다. YOLO는 안전 정지 토픽을 직접 쓰지
   않으며, 강제 정지는 계속 `/vlm/injury_stop -> h753_vlm_gateway ->
   /safety/vlm_stop -> UART bridge` 경로만 사용한다.
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
costmap inflation radius는 `0.35 m`를 유지한다. collision monitor의 별도
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
