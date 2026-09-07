# 서버 PC DB/VLM/GUI 개발 요청서 — 1차 최소 범위

- 작성일: 2026-09-07
- 전달 대상: 서버 PC 개발 담당자
- Jetson workspace: `/home/jyl1015/ros2_graduation_project_ws`
- 서버 저장소: `https://github.com/Yooourimmm/jjproject_archive`
- 확인한 서버 기준: `jjproject_10_260904` / `461f9045bb72559937d3530ecec7d5c11a4d6f8f`
- 이 문서가 1차 통합 범위와 서버 작업의 최신 기준이다.

## 1. 이번 개발의 최종 목표

로봇이 이동한 경로와 YOLO가 사람을 탐지한 순간의 **로봇 위치**를 서버 DB에
저장하고, 서버 GUI에서 사람별로 다음 정보를 확인할 수 있게 한다.

- 사람 1, 사람 2처럼 탐지 순서에 따른 사람 목록과 선택 버튼
- 선택한 사람을 탐지할 때까지 로봇이 실제로 지나온 경로
- 탐지 당시 로봇의 `map` 좌표 `x, y, yaw`를 나타내는 빨간 점
- 탐지 대표 이미지
- VLM 부상 판정과 관찰 내용
- 필요한 구호물품과 구조 상태

YOLO 최초 탐지 시 로봇 정지는 Jetson이 즉시 수행한다. VLM 판단이 끝난 뒤에는
현재 방식 그대로 서버가 `/vlm/injury_stop=0`을 발행하여 주행을 재개한다.

## 2. 이번 범위에서 개발하지 않는 기능

아래 기능은 서버와 Jetson 모두 이번 1차 완료 조건에서 제외한다.

- RGB-D를 이용한 사람 자체의 실제 지도 좌표 계산
- 빨간 점을 실제 사람 위치로 표시하는 기능
- A*, Dijkstra, costmap 등을 이용한 최적 안전 귀환 경로 생성
- 장기 동일인 추적·자동 병합
- 서버 단절 후 디스크 기반 작업 복구와 장애 복구 완료시험
- `detection_id`를 검사하는 엄격한 주행 해제와 `/vlm/decision`
- Mode 5의 지도 좌표 기록
- WebSocket, 다중 로봇, PostgreSQL/PostGIS 전환

기존 API idempotency나 Jetson outbox 코드를 제거할 필요는 없다. 다만 이번 작업에서
새로운 서버 복구 시스템을 추가하거나 별도 완료시험을 만들 필요는 없다.

## 3. ID와 데이터 소유권

| 데이터 | 생성 주체 | 서버 처리 |
| --- | --- | --- |
| `robot_id` | Jetson | 받은 값 보존 |
| `mission_id` | Jetson | 자체 생성 금지, 받은 값으로 mission 조회 |
| `detection_id` | Jetson | 자체 생성 금지, detection·image·VLM 결과의 공통 FK로 사용 |
| `route_seq` | Jetson | `(mission_id, route_seq)` 순서와 중복 방지 유지 |
| `map_id` | Jetson 계산, 서버 검증 | 기존 content hash 검증 유지 |
| `image_id` | Jetson 로컬 correlation ID | 서버 저장 ID는 기존 방식 사용 가능, ACK로 반환 |
| `victim_id` | 서버 | detection 저장 시 생성, GUI의 사람 항목과 연결 |

핵심 규칙은 **서버 VLM이 새 mission이나 detection UUID를 만들지 않는 것**이다.
한 번의 탐지는 Jetson이 생성한 하나의 `detection_id`로 아래 레코드가 연결돼야 한다.

```text
missions
  └─ route_points
  └─ detections ─ images
                 └─ vlm_assessments
                 └─ victims ─ supplies
```

## 4. Jetson에서 이미 제공하는 입력

### 4.1 ROS 2 탐지 이벤트

- Topic: `/mission/detection_event`
- Type: `std_msgs/msg/String`
- Payload: JSON
- QoS: reliable, transient-local, keep-last 10

필수 필드는 다음과 같다.

```json
{
  "schema_version": 1,
  "robot_id": "h753_jetson_01",
  "mission_id": "<Jetson mission UUID>",
  "detection_id": "<Jetson detection UUID>",
  "detected_at": "<ISO-8601 timestamp>",
  "map_id": "<map content ID>",
  "frame_id": "map",
  "robot_pose": {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0,
    "stamp_ns": 0
  },
  "route_end_seq": 0,
  "image_id": "<Jetson image correlation UUID>"
}
```

실제 이벤트에는 `yolo` 정보도 포함될 수 있다. 서버는 알 수 없는 선택 필드를
무시해도 되지만 위 필수 ID와 좌표 필드는 그대로 유지해야 한다.

### 4.2 VLM용 카메라

- 현재 서버 호환 입력: `/camera/camera/color/image_raw/compressed`
- Jetson gateway 별칭: `/vlm/request/image/compressed`
- Type: `sensor_msgs/msg/CompressedImage`
- 내용: VLM 판단용 raw 컬러 영상

1차 통합에서는 현재 서버 카메라 입력을 유지해도 된다. 두 토픽을 동시에 구독해
같은 프레임을 중복 추론하지 않는다. GUI 대표 이미지는 별도로 Jetson의
`/yolo/detected_image/compressed`에서 저장한 뒤 REST API로 전송된다.

### 4.3 REST API 전송

Jetson uploader는 다음 순서로 전송하며 서버 ACK가 확인된 뒤 다음 항목을 보낸다.

1. `GET /api/v1/health`
2. `POST /api/v1/maps:upload` — `yaml_file`, `pgm_file`
3. `POST /api/v1/missions`
4. `POST /api/v1/missions/{mission_id}/route-points:batch`
5. `POST /api/v1/detections`
6. `POST /api/v1/detections/{detection_id}/images` — `file`, `checksum`
7. `PATCH /api/v1/missions/{mission_id}`

인증 헤더는 `X-API-Key`이고 Jetson에서는 실제 값을
`H753_MISSION_API_KEY` 환경 변수로 주입한다. 비밀값을 문서나 Git에 넣지 않는다.

## 5. 서버 PC에서 개발할 내용

### S1. 서버 VLM의 중복 mission·경로·detection 생성 제거

대상: `vlm.py`

- 시작할 때 자체 `mission_id`를 생성하거나 ACTIVE mission을 선택하지 않는다.
- 서버에서 `map → base_link` TF를 샘플링해 `route_points`를 만들지 않는다.
- `pending_route_points`와 임시 경로 전송은 VLM 실행 흐름에서 제거하거나 비활성화한다.
- VLM 판정 시 `uuid.uuid4()`로 새 `detection_id`를 생성하지 않는다.
- VLM이 `detections`를 직접 새로 생성하지 않는다. detection은 Jetson REST 업로드가
  생성한 서버 DB 레코드를 사용한다.

완료 기준: 같은 주행에 Jetson mission과 서버 VLM mission이 따로 생성되지 않고,
경로점도 Jetson 업로드분만 존재한다.

### S2. Jetson 탐지 ID를 VLM 판정 컨텍스트로 사용

대상: `vlm.py`

- `/mission/detection_event`를 구독한다.
- 서버의 기존 YOLO gate가 VLM 추론을 시작하더라도, 결과 저장에는 가장 최근의
  유효한 Jetson `mission_id/detection_id`를 사용한다.
- gate가 event보다 먼저 도착할 수 있으므로 짧은 정상 동기화 대기는 허용한다.
- event가 끝내 없으면 새 ID를 만들지 않는다. VLM 결과와 주행 해제는 처리하되
  DB에는 다른 탐지와 잘못 연결된 assessment를 저장하지 않고 오류를 기록한다.
- 한 event당 VLM 판정은 최대 한 번만 저장한다.

완료 기준: `/mission/detection_event`에서 본 `detection_id`와 DB의
`vlm_assessments.detection_id`가 동일하다.

### S3. VLM 결과·구호물품 저장

대상: `vlm.py`, 필요 시 `init_db.py` 또는 서버 내부 저장 모듈

- VLM 결과는 같은 `detection_id`의 `vlm_assessments`에 저장한다.
- 저장 전 해당 detection이 Jetson REST 업로드로 DB에 생성됐는지 확인한다.
- REST 업로드와 VLM 완료 순서가 엇갈리면 짧게 대기·재시도하되 새 detection을
  만들거나 다른 detection에 붙이지 않는다.
- 부상 category, AI 관찰 내용, confidence, 모델 정보와 raw output을 보존한다.
- VLM이 결정한 구호물품을 해당 detection의 `victim_id`에 연결해 `supplies`에
  저장한다. 같은 사람·같은 물품이 중복 INSERT되지 않게 한다.
- NORMAL 판정도 이력으로 남길 수 있으나 불필요한 구호물품은 생성하지 않는다.

완료 기준: 사람 버튼 하나를 선택했을 때 이미지·VLM 판정·구호물품이 동일한
`detection_id/victim_id`로 조회된다.

### S4. 기존 주행 재개 방식 유지

대상: `vlm.py`

- VLM 판단 시작·진행 중에는 현재 정지 상태를 유지한다.
- 판정 처리가 끝나면 기존 방식대로 `/vlm/injury_stop`에 `Int32(data=0)`을 발행한다.
- 부상자 확정 시 기존의 짧은 확인 유지 시간이 필요하다면 그 시간이 끝난 뒤 0을
  발행한다.
- `/vlm/decision` 신규 개발은 하지 않는다.
- 어떤 정상 탐지 event도 결과 없이 조용히 버려서 로봇이 계속 정지하게 만들지
  않는다. 처리·오류 어느 경우든 로그에서 최종 상태를 확인할 수 있어야 한다.

완료 기준: 현재 Jetson 설정 `require_correlated_decision=false`에서 판단 완료 후
주행이 정상 재개된다.

### S5. GUI를 실제 업로드 데이터에 연결

대상: `map_view.py`, 필요 시 `ui_dashboard.py`

- 기존 파란 경로와 빨간 점 표시를 유지한다.
- 파란 경로는 `route_seq <= route_end_seq`인 실제 이동경로만 표시한다.
- 빨간 점은 `detections.robot_x/robot_y`이며 라벨을 **탐지 당시 로봇 위치**로
  명확히 표시한다.
- 사람 1과 사람 2 버튼 또는 목록을 제공하고 선택 항목을 지도 빨간 점과 연동한다.
- 상세 패널에 탐지 시간, 로봇 `x/y/yaw`, 대표 이미지, VLM category·관찰 내용,
  필요한 구호물품, 구조 상태를 표시한다.
- GUI가 임시 경로 또는 서버 VLM이 따로 만든 detection을 표시하지 않게 한다.

완료 기준: 서로 다른 장소에서 탐지한 두 사람을 선택하면 각자의 빨간 점과 서로
다른 `route_end_seq`까지의 경로가 표시된다.

### S6. 서버 자동시험과 정상 연결 통합시험

최소 자동시험:

- 같은 mission/detection 요청 재전송 시 레코드 중복 없음
- 같은 `detection_id`에 이미지와 VLM assessment가 연결됨
- 서버 VLM이 자체 mission/detection UUID를 만들지 않음
- VLM 결과와 supplies가 올바른 victim에 연결됨
- 사람 1/2의 경로가 각자의 `route_end_seq`에서 잘림
- VLM 판정 한 건당 최종 `/vlm/injury_stop=0` 발행 확인

이번 시험에서 제외:

- 서버 네트워크 단절·재시작 복구 시험
- 실제 사람 좌표 정확도 시험
- 안전 귀환 경로 시험
- `/vlm/decision` correlation 시험

## 6. 서버 구현 완료 후 Jetson과 함께 할 시험

1. 서버 IP·포트와 새 API key를 Jetson에 설정한다.
2. `GET /api/v1/health`와 ROS 2 토픽 discovery를 각각 확인한다.
3. Mode 4에서 짧은 경로를 주행한다.
4. 첫 장소에서 사람 1을 탐지하고 즉시 정지, REST 저장, VLM 판정을 확인한다.
5. VLM 완료 후 `/vlm/injury_stop=0`으로 주행이 재개되는지 확인한다.
6. 다른 장소에서 사람 2를 탐지해 별도 `detection_id/victim_id`인지 확인한다.
7. GUI에서 사람 1/2의 경로, 빨간 점, 이미지, 판정과 물품을 비교한다.
8. RViz의 탐지 당시 로봇 위치와 GUI 빨간 점 오차가 `0.10 m` 이내인지 확인한다.

## 7. 서버 담당자 회신 형식

개발 완료 후 아래 형식으로 Markdown 체크리스트 한 파일을 전달한다.

```markdown
# 서버 DB/VLM/GUI 개발 회신

- 작업일:
- 실제 저장소 URL:
- branch:
- commit SHA:
- 실행 명령:
- API base URL:
- 사용한 환경 변수 이름(값은 적지 않음):

## 변경 파일
| 파일 | 변경 내용 | 완료 여부 |
| --- | --- | --- |

## S1~S6 결과
| 항목 | PASS/FAIL | 구현 근거 파일·함수 | 시험 근거 |
| --- | --- | --- | --- |

## ID 일치 증거
- mission_id:
- detection_id:
- detection DB row:
- image DB row:
- vlm_assessment DB row:
- victim_id / supplies row:

## 자동시험
- 실행 명령:
- 결과: passed / failed

## 수동 GUI 확인
- 사람 1 결과:
- 사람 2 결과:
- 경로와 빨간 점 스크린샷 파일명:

## 남은 문제
- 없음 또는 구체적인 증상·재현 절차
```

회신에는 실제 운영 API key, 운영 DB, 실제 환자 이미지 같은 비밀·민감 데이터를
포함하지 않는다. API 계약을 변경했다면 같은 commit에서 생성한 `openapi.json`도
함께 전달한다.

## 8. 전체 완료 판정

- [ ] Jetson이 생성한 mission ID 하나만 사용됨
- [ ] 각 탐지에서 Jetson detection ID 하나만 사용됨
- [ ] 실제 지나온 경로만 서버 DB와 GUI에 표시됨
- [ ] 빨간 점이 탐지 당시 로봇 위치로 정확히 표시됨
- [ ] 사람 1/2의 이미지·VLM 결과·구호물품이 섞이지 않음
- [ ] VLM 판단 완료 후 기존 방식으로 주행 재개됨
- [ ] 정상 연결 상태의 두 사람 시나리오가 통과함
- [ ] 제외 기능이 완료 조건이나 서버 작업에 다시 포함되지 않음
