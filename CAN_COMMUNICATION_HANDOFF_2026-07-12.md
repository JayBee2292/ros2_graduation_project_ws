# H753 CAN Communication Handoff

## 2026-07-13 펌웨어 진단 업데이트

사용자가 CAN 물리 배선을 다시 확인했다. 전원 차단 상태 CANH-CANL은 61 ohm,
공통 GND와 CANH/CANL 연속성은 멀티미터로 확인됐으며 사용 중인 외부 CAN FD
모듈에는 별도 STB/EN/SILENT 제어 핀이 없다. 따라서 아래 2026-07-12 결론의
"종단/공통 GND/배선이 주원인"은 더 이상 확정 결론으로 사용하지 않는다.

STM32 펌웨어를 다시 CAN FD+BRS, nominal 1 Mbps/data 5 Mbps로 복원했다.
최종 설정은 `FDCAN_FRAME_FD_BRS`, data prescaler 1, TSEG1 20, TSEG2 3,
SJW 3, TDC offset 21 mtq이다. 최종 ELF를 OpenOCD로 flash/verify/reset했고
`Verified OK`를 확인했다.

bus-off 원인을 확인하기 위해 상태 프레임 reserved 영역에 아래 진단값을 추가했다.

- bus-off count와 error warning/passive event count
- PSR LEC/DLEC, EP/EW/PXE, TDCV
- ECR TEC/REC/CEL
- 마지막 error-status interrupt bits

동일 값은 bus-off/error event 변화 시 ST-LINK UART `CANDBG` 한 줄에도 출력된다.
ROS `can_odom_node.py`도 새 payload를 decode해 원인명을 포함한 WARN을 출력한다.

### 2026-07-13 단계별 계층 재검증

- Jetson은 ST-LINK V3를 정상 열거했고 OpenOCD SWD로 H753 코어에 연결됐다.
  타깃 전압은 약 3.26 V였고 코어는 running 상태였다.
- 실제 플래시에 사용된 `build/Debug/h753_ros_humble.elf`와 MCU flash
  89748 bytes를 OpenOCD `verify_image`로 비교해 완전 일치를 확인했다.
- ST-LINK VCP `/dev/ttyACM0`, 921600 baud에서 STM 상태 출력을 수신했고,
  Jetson이 보낸 모터 비동작 명령 `h`에 STM이 도움말을 회신했다. UART는
  양방향 정상이다.
- normal mode 1 Mbps/5 Mbps 8초 시험에서는 64-byte BRS 프레임을 다수
  수신했지만 STM 진단이 `TEC=133`, `EP=1`, `EW=1`, `error_event_count=5`,
  `DLEC=ack/bit1`까지 증가했다. CANable error register는 0이었다.
- 새로운 A/B로 H753을 잠시 `FDCAN_MODE_EXTERNAL_LOOPBACK`으로 만들고
  telemetry를 자체 활성화했다. CANable은 silent/listen-only로 두어 ACK를
  전혀 보내지 않았다. 15초 동안 H753 상태 프레임의 `bus_off_count`,
  `error_event_count`, `TEC`가 모두 0으로 유지됐다. 따라서 H753 FDCAN의
  TX -> CA-IS2062A -> RX 자체 감시 경로와 TDC는 5 Mbps/64-byte에서 정상이다.
- 같은 외부 루프백 시험에서 silent CANable 사용자 공간 수신 sequence에는
  `0x2C -> 0x2F`, `0x2F -> 0x31` 누락이 관측됐다. 이 누락만으로 CANable의
  CAN RX와 USB 출력 중 어느 쪽인지 단정할 수는 없지만, normal mode에서만
  오류가 생기는 사실과 합치면 남은 범위는 외부 peer의 수신/ACK 체인,
  즉 CA-IS2062A 송신 파형과 CANable Pro S ADM3050E 수신·ACK 조합이다.
- 시험 직후 임시 루프백 변경을 모두 원복하고 normal-mode 1M/5M FD+BRS
  ELF를 다시 build/flash/verify/reset했다. 복원 ELF SHA-256은
  `d164dd5281561630e7d9159a9cbbcbfc8830be8a7dab0b2eb2322f8cec3f3288`이다.

실기 결과:

- 5 Mbps, TDC offset 21: 40초에 bus-off count 18
- D435i와 xone/하위 USB 허브를 모두 분리한 5 Mbps 대조 시험: 42초에
  bus-off count 16, error event count 36; 기존과 같은 빈도와 오류 형태
- 동일 5 Mbps에서 STM32 송신 DLC만 임시 64바이트에서 8바이트로 줄인 시험:
  32초에 bus-off count 4; 프레임 길이가 짧아지자 오류 빈도가 약 3배 감소
- 방향 반전 시험: CANable이 ID 0x1FF, 64바이트 BRS 5 Mbps 프레임 400개를
  송신했고 STM32 RAM 수신 카운터가 정확히 400(0x190)이었다
- 5 Mbps, TDC offset 20 A/B 시험: 30초에 bus-off count 12; 유의미한 개선 없음
- 임시 2 Mbps/Y2 대조 시험: 30초에 bus-off count 6; 감소했지만 미해결
- 주 오류는 data phase `DLEC=bit1` 또는 `DLEC=ack`
- TEC는 warning/passive 시 약 96 -> 128 이상, bus-off 직전 약 250까지 증가
- REC는 계속 0
- 5 Mbps TDCV 43~44, 2 Mbps TDCV 73; `TDCV-TDCO`는 약 23 mtq로 일관됨

이는 STM32 수신 오류보다 STM32 송신 프레임의 데이터 위상에서 CANable의 정상
수신/ACK가 간헐적으로 실패하는 형태다. 디렉터리 이동은 FDCAN 바이너리에 영향을
주지 않으며, D435i는 USB 3.x 트리, CANable/ST-LINK는 USB 2.0 트리에 연결돼 있다.
시험 중 CANable USB reset/disconnect 기록은 없었다. CANable `V` 명령의
`16e7497-dirty github.com/normaldotcom/canable2.git`과 USB 제품 문자열의
`b158aa7` 불일치는 아래에서 설명한 것처럼 Makerbase 공식 배포 바이너리
자체의 특성으로 확인됐다.

D435i/xone 분리 시험 중에도 CANable/ST-LINK USB reset이나 disconnect는 없었으나
bus-off가 그대로 재현됐다. 따라서 D435i IMU 드라이버, D435i USB 트래픽, xone USB
오류는 현재 CAN FD bus-off의 직접 원인에서 배제한다.

방향 반전 400/400 성공으로 nominal/data bitrate 자체, 공통 배선, STM32 수신 경로,
CANable 송신 경로는 정상임을 확인했다. 남은 범위는 STM32 쪽 CA-IS2062A 송신 경로
(특히 전원/파형) 또는 CANable 수신 경로다. TDCV 44에서 TDCO 21을 빼면 측정 loop
delay는 23 mtq, 약 192 ns로 CA-IS2062A 사양(typ 165 ns, max 255 ns) 범위 안이다.

Makerbase 공식 `CANable-MKS` 저장소의 V2.0 SLCAN 배포본
`canable2-b158aa7.bin` SHA-256은
`40a913e61d9dfb848e498d72a3053d0512eae565a97686e90ea6d07b83210122`이다.
이 공식 binary 내부에 USB 문자열
`CANable2 b158aa7 github.com/normaldotcom/canable2.git`과 `V` 명령 문자열
`16e7497-dirty github.com/normaldotcom/canable2.git`이 둘 다 있다. 현재 실기의
응답과 정확히 일치하므로 혼합/손상 펌웨어라는 추정은 폐기하고,
CANable 재플래시도 중단했다. `16e7497`에서 `b158aa7`까지의 실제 소스
차이도 printf 추가와 USB VID/PID 변경뿐이고 CAN/FDCAN 처리 코드 변경은 없다.
따라서 새 upstream binary를 굽는 것은 현재 CAN 오류의 유효한 A/B 시험이
아니다.

실제 구매 모델은 디바이스마트 상품번호 15775525의
`Makerbase CANable V2.0 Pro S`다. `S`는 케이스형, `Pro`는 일반 V2.0과
다른 절연형이다. Makerbase 제품 소개에서 Pro는 ADM3050E 절연 CAN FD
트랜시버로 구분된다. 판매 페이지의 `TJA1051T/3` 요약은 일반 V2.0
사양이 혼입된 것으로 보이며, Pro S 상품 키워드에는 절연형/전원 절연이
명시돼 있다. 케이스 외부에 BOOT 버튼이 없는 사용자 실물과도 일치한다.
일반 V2.0 매뉴얼의 외부 BOOT 점퍼 안내를 Pro S에 그대로 적용하지
않으며, 케이스를 열지 않는다.

Pro S를 확인한 후 최종 1 Mbps/5 Mbps 상태를 15초 재시험했다.
STM32 telemetry enable을 1초마다 보내면 0x100/0x101 64바이트 BRS 프레임은
일부 수신되지만 STM32 bus-off count가 0에서 6, error event count가
17까지 증가했다. 마지막 상태는 DLEC=bit1, TEC=134, REC=0,
EP=1, EW=1, TDCV=44였다. 동시에 CANable sticky error register는 `0`이었다.
즉 Pro S로 모델을 바로잡은 후에도 기존의 방향 의존적 물리 오류는 그대로
재현됐다. 남은 경로는 STM32 CA-IS2062A TX 파형과 CANable Pro S
ADM3050E RX 파형의 조합이다.

추가 소프트웨어 A/B 후보로 2025~2026년에 적극 개발 중인 오픈소스
`Elmue/CANable-2.5-firmware-Slcan-and-Candlelight`를 확인했다. 정확히
MKS Makerbase STM32G431 Multiboard를 지원하고, 기존 SLCAN 명령과 하위
호환되며, 사용자 지정 bit timing/sample point, 개선된 오류 보고, CANable
Pro S 실기 검증을 제공한다. 현재 소스(2026-06-18)의 Makerbase용
SLCAN 이미지를 `/tmp/canable-2.5-fw`에서 성공적으로 build했고 binary는
`Build_STM32G431_Slcan_Multiboard/STM32G431_Slcan2.5_Multiboard_0x260618.bin`,
SHA-256은 `9287a1310d4b7e6052139bfc80aca10dc022b6f496bdf9b9f36e036156081dc9`이다.
다만 현재 legacy SLCAN USB descriptor는 CDC 제어/데이터 2 interface뿐이고 DFU
runtime interface가 없으며 `X` 명령도 TODO이다. 2.5를 한 번 올리고 나면
소프트웨어 DFU 진입이 가능하지만, 최초 플래시는 현재 장치를 DFU로
진입시킬 별도 방법이 필요하다. 사용자 확인 없이 케이스를 열거나 플래시하지
않았다.

방향 반전 시험 준비 중 ID 0x1FF 필터가 DLC 검사 전에 HAL 수신 복사를 수행하지만
수신 버퍼가 8바이트뿐인 결함을 발견했다. 64바이트 FD 입력 시 스택 오버플로가 날
수 있어 `can_telemetry.c` RX 버퍼를 64바이트로 수정했다. 임시 8바이트 송신과 RAM
카운터 코드는 제거했으며, 최종 STM32는 다시 64바이트/5 Mbps로 build, flash,
verify, reset 완료했다.

최종 MCU에는 임시 2 Mbps가 아니라 요청한 5 Mbps 진단 펌웨어가 flash돼 있다.

전원 복구 후 CANable Pro S를 다시 확인했다. 빠른 USB 재연결로는 STM32 ROM DFU
VID:PID `0483:df11`이 나타나지 않았고, 기존 SLCAN 정상 모드 `16d0:117e`
(`CANable2 b158aa7`)로 재열거됐다. 따라서 이 과정에서도 CANable 펌웨어는
flash하지 않았다. `/dev/ttyACM1` 생성 후 nominal 1 Mbps/data 5 Mbps로 5초간
telemetry를 재시험한 결과 64바이트 BRS 프레임은 일부 정상 수신됐지만 STM32
진단은 `bus_off_count=2`, `DLEC=4`(Bit1/recessive error), `TEC=248`, `REC=0`,
`CEL=17`, `EP=1`, `EW=1`을 기록했다. CANable error register는 `0`이었다.
전원 재인가 뒤에도 기존의 STM32 송신 방향 오류가 동일하게 재현된 것이다.

- 작성 시각: 2026-07-12, Asia/Seoul
- 워크스페이스: `/home/jyl1015/ros2_graduation_project_ws`
- STM32 펌웨어: `/home/jyl1015/ros2_graduation_project_ws/h753_ros_humble`
- ROS 2 패키지: `/home/jyl1015/ros2_graduation_project_ws/src/h753_can_odom`

## 2026-07-12 당시 결론 (위 2026-07-13 업데이트로 폐기)

아래 내용은 진단 이력을 보존하기 위한 당시 기록이다. 현재 설정과 판단은 문서
맨 위의 2026-07-13 업데이트를 기준으로 한다.

소프트웨어 측에서 발견된 포트 오선택, 중복 CAN 노드, 비독점 serial open, 오래된 ST-LINK 경로, 잘못된 lock 경로, CANable USB 출력 과부하 구조는 수정했다. 최종 펌웨어도 빌드 후 ST-LINK로 flash/verify 완료했다.

그러나 실기 시험에서 STM32 FDCAN bus-off가 반복됨을 새 진단 카운터로 확인했다. CAN FD BRS를 끄고 전체 프레임을 1 Mbps로 낮춰도 bus-off가 남으므로, 현재 남은 주원인은 ROS 설정이 아니라 CAN 물리 계층이다. 종단저항, 공통 GND, CANH/CANL 배선, 외부 CAN 트랜시버 전원 및 STB/EN 상태를 확인해야 한다.

## 2026-07-12 당시 장치 매핑

- CANable2: `/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209F33833630-if00` -> `/dev/ttyACM1`
- ST-LINK VCP: `/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_0036002C3235511837333439-if02` -> `/dev/ttyACM0`
- LiDAR CP2102: `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` -> `/dev/ttyUSB0`

최종 확인 시 `/dev/ttyACM0`만 PID 23449 `cmd_vel_uart_bridge_node`가 사용 중이었다. CANable `/dev/ttyACM1`은 비어 있고, 지속 실행 중인 `can_odom_node`는 없다. ROS 노드는 `/h753_cmd_vel_uart_bridge`만 남아 있다.

## 최초 장애 원인

1. `can_odom_node`가 3개 동시에 실행되어 같은 CANable serial 스트림을 나눠 읽고 있었다.
2. 기존 UART 자동 선택 fallback이 ST-LINK를 못 찾으면 `/dev/ttyACM1`을 선택할 수 있어 CANable에 UART 주행 바이트를 쓸 위험이 있었다.
3. CANable serial open에 exclusive lock이 없어 중복 실행을 차단하지 못했다.
4. mode manager의 ST-LINK by-id가 과거 장치 serial을 가리켰다.
5. UART control lock 경로가 현재 워크스페이스와 달랐다.
6. STM32가 상태 64바이트와 엔코더 64바이트를 연속 전송해 CANable SLCAN USB TX 큐를 압박했다.
7. bus-off callback이 telemetry enable까지 끄도록 작성되어 복구 후 다음 1초 heartbeat까지 불필요하게 멈췄다.

## 2026-07-12 당시 수정 파일

### ROS 2

- `src/h753_can_odom/h753_can_odom/can_odom_node.py`
  - CANable을 `exclusive=True`로 open한다.
  - 종료 시 telemetry disable 후 channel close를 수행한다.
  - 1초마다 telemetry enable heartbeat를 전송한다.
  - 최종 안전 설정에 맞춰 enable 명령도 CAN FD without BRS로 전송한다.
  - 상태 payload의 `bus_off_count`를 decode하고 증가 시 WARN을 출력한다.
- `src/h753_can_odom/h753_can_odom/cmd_vel_uart_bridge_node.py`
  - STMicroelectronics/ST-LINK by-id 장치만 자동 선택한다.
  - CANable/Openlight/CP210 계열은 명시적으로 제외한다.
  - raw `/dev/ttyACM*` fallback을 제거했다.
  - UART도 `exclusive=True`로 open한다.
  - lock 기본 경로를 현재 워크스페이스로 변경했다.
- `src/h753_can_odom/h753_can_odom/robot_mode_manager_node.py`
  - CANable과 ST-LINK 식별 hint를 분리했다.
  - 오래된 by-id가 설정되어도 현재 연결된 동일 계열 by-id를 찾도록 보강했다.
- `src/h753_can_odom/config/h753_can_odom.yaml`
  - CANable by-id, `S8`, `Y5`, poll 20 ms를 사용한다.
  - 현재 펌웨어는 BRS를 끈 상태라 `Y5` 데이터 속도 preset은 실제 telemetry frame에 적용되지 않는다.
- `src/h753_can_odom/config/h753_cmd_vel_uart_bridge.yaml`
  - lock 경로를 `h753_ros_humble/tools/logs/xbox_uart_control.lock`로 수정했다.
- `src/h753_can_odom/config/h753_robot_mode_manager.yaml`
  - ST-LINK by-id를 현재 serial `0036002C3235511837333439`로 수정했다.

### STM32 펌웨어

- `h753_ros_humble/App/Src/can_telemetry.c`
  - telemetry는 host enable 수신 전까지 꺼진 상태로 시작한다.
  - 상태와 엔코더를 같은 tick에 연속 전송하지 않는다.
  - 40 ms slot 5개 중 4개는 encoder, 1개는 state를 전송한다.
  - 최종 목표 전송률은 encoder 20 Hz, state 5 Hz이다.
  - bus-off interrupt에서 `CCCR.INIT`를 clear해 복구를 시작한다.
  - bus-off 때 telemetry enable은 유지하고 `bus_off_count`만 증가시킨다.
  - 상태 payload reserved 영역에 `bus_off_count`를 넣는다.
- `h753_ros_humble/App/Inc/can_telemetry.h`
  - 변경된 상태 payload와 CAN FD without BRS 동작을 문서화했다.
- `h753_ros_humble/Core/Src/fdcan.c`
  - 최종 `FDCAN_FRAME_FD_NO_BRS` 설정이다.
  - nominal 1 Mbps: 120 MHz / (5 * (1 + 20 + 3)), sample point 87.5%.
  - data timing register는 기존 5 Mbps 설정을 보존하지만 BRS가 꺼져 현재 프레임은 전체가 1 Mbps로 전송된다.

주의: `h753_ros_humble.ioc`에는 FDCAN의 전체 수동 설정이 저장되어 있지 않고 nominal 계산값만 있다. CubeMX code generation을 다시 실행하면 `Core/Src/fdcan.c`의 수동 설정이 덮어써질 수 있다. 다음 작업자는 `.ioc`를 먼저 동기화하거나 FDCAN 설정을 다시 대조해야 한다.

## 빌드와 플래시

펌웨어 빌드:

```bash
cd /home/jyl1015/ros2_graduation_project_ws/h753_ros_humble
cmake --build build/Debug --parallel
```

최종 빌드 결과:

- ELF: `h753_ros_humble/build/Debug/h753_ros_humble.elf`
- FLASH: 88644 bytes
- RAM: 75912 bytes

ROS 패키지 빌드:

```bash
cd /home/jyl1015/ros2_graduation_project_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select h753_can_odom
```

플래시 명령:

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program /home/jyl1015/ros2_graduation_project_ws/h753_ros_humble/build/Debug/h753_ros_humble.elf verify reset exit"
```

최종 flash 결과는 `Programming Finished`, `Verified OK`, `Resetting Target`이었고 target voltage는 약 3.262 V였다.

## 실기 검증 결과

초기 중복 프로세스를 제거하고 수정 노드를 단독 실행했을 때 `/odom` 수신 자체는 복구됐다. 50 Hz encoder 중심 설정에서는 순간 평균 약 47 Hz까지 나왔지만 0.66~1.20초 공백이 반복됐다.

CANable `E` 명령 결과는 `CANable Error Register: 40`이었다. 공식 CANable2 펌웨어의 `error.h` 기준 `0x40`은 bit 6 `ERR_FULLBUF_USBTX`이며 CAN bus-off 값이 아니다. 이 레지스터는 부팅 후 sticky이므로 CANable USB 전원을 재연결하기 전까지 과거 오류도 남는다.

상태/엔코더를 분리하고 출력률을 낮춘 뒤에도 공백이 남아 STM32 bus-off 카운터를 추가했다. 최종 CAN FD without BRS 시험 로그:

- CANable open: `S8/Y5`
- encoder baseline 수신 성공
- STM32 bus-off count가 2, 4, 6으로 증가
- encoder dt 0.52, 0.56, 0.72초 공백 발생
- `/odom`은 동작하지만 장시간 안정적이지 않았고 시험 후반 평균 약 14.5 Hz

따라서 현재는 코드가 CAN 프레임을 송수신할 수 있으나, 물리 오류로 STM32가 반복 bus-off에 진입하는 상태다.

## 다음 작업 우선순위

1. 전원을 완전히 끄고 CANH-CANL 저항을 측정한다.
   - 정상적인 양 끝 120 ohm 종단이면 약 60 ohm이 측정되어야 한다.
   - 약 120 ohm이면 종단 하나가 빠졌을 가능성이 높다.
   - 무한대에 가까우면 종단/배선 단선, 0 ohm에 가까우면 단락을 의심한다.
2. CANable GND와 STM32 외부 CAN 트랜시버 GND가 연결되어 있는지 확인한다.
3. CANH/CANL 뒤바뀜, 느슨한 점퍼선, 브레드보드 접촉을 확인한다.
4. STM32에 연결된 외부 트랜시버가 CAN FD 지원 모델인지 확인한다.
5. 트랜시버 전원 전압과 STB/EN/SILENT 핀이 정상 모드인지 확인한다.
6. CANH/CANL은 짧은 twisted pair로 연결하고 모터 전원선과 떨어뜨린다.
7. 모터 전원을 끈 상태에서 먼저 CAN만 시험해 EMI 영향을 분리한다.
8. 물리 수정 후 CANable USB도 재연결해 sticky `0x40` 레지스터를 초기화한다.

## 재검증 명령

CAN odom 노드는 반드시 하나만 실행한다.

```bash
cd /home/jyl1015/ros2_graduation_project_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run h753_can_odom can_odom_node \
  --ros-args --params-file install/h753_can_odom/share/h753_can_odom/config/h753_can_odom.yaml
```

다른 터미널:

```bash
source /opt/ros/humble/setup.bash
source /home/jyl1015/ros2_graduation_project_ws/install/setup.bash
ros2 topic hz /odom
```

정상 기대값:

- `/odom` 약 20 Hz
- `Encoder dt out of range` 경고 없음
- `STM32 FDCAN bus-off count` 경고 없음
- `fuser -v /dev/ttyACM1`에 CAN odom 프로세스 하나만 표시

## LD4 COM LED

NUCLEO-H753ZI의 `LD4 COM`은 CAN 오류 LED가 아니라 ST-LINK 통신 상태 LED다. 현재 `cmd_vel_uart_bridge_node`가 ST-LINK VCP `/dev/ttyACM0`으로 20 Hz UART 패킷을 계속 전송하므로 빨간색으로 계속 켜지거나 빠르게 점멸해 보일 수 있다. 실제 overcurrent 경고 LED는 `LD6`이다.

## 참고 링크

- ST NUCLEO-H753ZI 사용자 매뉴얼: https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf
- NUCLEO-H753ZI 제품 페이지: https://www.st.com/en/evaluation-tools/nucleo-h753zi.html
- CANable2 공식 펌웨어: https://github.com/normaldotcom/canable2-fw
- CANable2 error flags: https://github.com/normaldotcom/canable2-fw/blob/main/inc/error.h
- CANable2 SLCAN 명령 처리: https://github.com/normaldotcom/canable2-fw/blob/main/src/slcan.c
