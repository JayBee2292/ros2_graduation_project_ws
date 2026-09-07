# Jetson-서버 DB/VLM/GUI 통합 개발계획

- 작성일: 2026-09-07
- 기준 Jetson workspace: `/home/jyl1015/ros2_graduation_project_ws`
- 서버 저장소: `https://github.com/Yooourimmm/jjproject_archive`
- 실제 원격 기본 branch(2026-09-07 확인): `jjproject_10_260904`
- 실제 원격 기준 commit: `461f9045bb72559937d3530ecec7d5c11a4d6f8f`
- 주의: 서버 회신에 적힌 `integration-checklist-fixes-260904` / `6a52809`는
  원격 refs에서 확인되지 않았다. 아래 계약 대조는 실제 원격 commit을 기준으로 한다.
- 기준 서버 회신: `SERVER_TEAM_통합체크리스트_회신_260904.md`
- 적용 범위: 로봇 이동경로, 사람 탐지 당시 로봇 좌표, 기존 VLM 판단 후 재주행

## 0. 현재 범위 결정

2026-09-07 사용자 결정으로 1차 범위를 단순화했다.

- 필수: 로봇이 지나온 경로 저장·전송·GUI 표시
- 필수: YOLO가 사람을 탐지한 당시의 로봇 `x, y, yaw` 저장·빨간 점 표시
- 필수: 기존 VLM 판단 완료 후 `/vlm/injury_stop=0`으로 주행 재개
- 제외: RGB-D 사람 자체 좌표, 최적 안전 귀환경로, 장기 동일인 추적
- 제외: 서버 단절 후 자동 복구 개발과 장애 복구 완료시험
- 제외: detection ID 기반 엄격한 주행 해제

이미 구현된 outbox와 `/vlm/decision` 수신 코드는 제거하지 않지만 추가 개발하지
않는다. `require_correlated_decision=false`를 유지해 기존 주행 재개 방식을 쓴다.

## 1. 목표

Jetson이 생성한 임무와 탐지 ID로 로봇의 실제 이동경로와 사람 탐지 당시 로봇
위치를 서버 DB와 GUI에 연결한다. VLM 판단 완료 후 주행 재개는 현재 사용 중인
`/vlm/injury_stop=0` 방식을 그대로 유지한다.

목표 흐름은 다음과 같다.

```text
Jetson YOLO 0→1
  → 로컬 즉시 정지
  → Jetson mission_id/detection_id 생성 및 SQLite commit
  → 지도/임무/경로/탐지/이미지 순서로 서버 전송
  → 서버 VLM이 같은 detection_id로 판정
  → 서버 victim 생성 및 GUI 표시
  → 기존 /vlm/injury_stop=0으로 주행 재개
```

## 2. 현재 기준 상태

### Jetson 완료

- Mode 3/4의 `map → base_link` 실제 경로 기록
- Jetson `mission_id`, `detection_id`, `image_id`, `route_seq` 생성
- 탐지 당시 로봇 pose, `route_end_seq`, 대표 이미지 저장
- SQLite WAL 기반 outbox와 프로세스 재시작 복구
- 지도·임무·경로 batch·탐지·이미지 HTTP uploader
- `PENDING/FAILED/SENT`, retry backoff와 서버 ACK JSON 저장
- 서버와 동일한 Go2 map ID `map_e5c5c16c93333f71` 계산
- `X-API-Key`, `/api/v1/maps:upload` 기본 계약 반영
- 신뢰성 있는 transient-local `/mission/detection_event` 발행
- YOLO 감지 즉시 `/safety/vlm_stop=1`을 거는 로컬 안전 경로
- `h753_can_odom` 자동시험 `138 passed`와 빌드 통과

### 서버 완료 보고

- DB schema v3와 연결별 foreign key 강제
- mission/route/detection/map/image/victim API
- `PATCH /api/v1/missions/{mission_id}`
- UUID idempotency conflict `409`
- 이미지 checksum 중복 방지
- detection 저장 시 victim 영속 생성
- ACTIVE mission 복구
- `/vlm/result_detail`에 `mission_id/detection_id` 추가
- `X-API-Key` 활성화

### 아직 통합되지 않은 부분

- 서버 VLM은 여전히 자체 `mission_id/detection_id`를 생성할 수 있음
- Jetson과 서버의 실제 request/response를 맞춘 자동 계약시험이 없음
- detection 이미지 multipart의 정확한 field 계약을 서버 소스로 확인하지 못함
- HTTP 오류를 재시도 가능 오류와 영구 오류로 구분하지 않음
- `/vlm/injury_stop`은 Int32라 어떤 detection의 결정인지 직접 증명할 수 없음
- 서버의 실제 IP와 새 API key가 Jetson에 배포되지 않음
- 실제 로봇·서버·GUI를 함께 사용한 두 사람 탐지시험이 없음

## 3. 통합 결정 사항

### 3.1 ID 소유권

- `robot_id`: `h753_jetson_01`
- `mission_id`: Jetson이 최종 발급
- `detection_id`: 원래 계획의 B안, 즉 Jetson이 최종 발급
- `image_id`: Jetson은 로컬 대표 이미지 correlation ID를 발급한다. 현재 서버는
  저장 시 별도 `image_id`를 발급하므로, checksum과 서버 ACK의 `image_id`를
  Jetson outbox `ack_json`에 함께 보존한다.
- `map_id`: 양쪽이 같은 content hash를 계산하고 서버가 파일에서 재검증
- `victim_id`: 서버가 detection 저장 후 발급

서버 VLM은 자체 mission/detection을 새로 만드는 대신 Jetson 이벤트의 ID를
사용해야 한다. 서버가 먼저 실행됐다는 이유로 별도 ACTIVE mission을 만들지 않고
Jetson mission을 기다리도록 한다.

### 3.2 안전 소유권

- YOLO 최초 감지 즉시 정지는 계속 Jetson 로컬에서 수행한다.
- DB/API 응답을 기다린 뒤 정지하는 구조로 바꾸지 않는다.
- 서버는 `/cmd_vel`이나 UART를 직접 제어하지 않는다.
- 최종 주행 해제는 현재 active `mission_id/detection_id`에 해당하는 서버 결정만
  수용하도록 correlation을 강화한다.

### 3.3 비밀값

- 서버 회신 문서에 노출된 기존 API key는 폐기하고 새로 발급한다.
- 새 값은 문서, YAML, Git, 터미널 로그에 기록하지 않는다.
- Jetson에서는 `H753_MISSION_API_KEY` 환경 변수로만 주입한다.
- 저장소에는 비밀값이 없는 환경 변수 이름과 설정 예시만 둔다.

## 4. 단계별 Jetson 개발

### Phase 0 — 보안 조치와 서버 기준 고정

- [ ] 서버가 노출된 API key를 폐기하고 새 key 발급
- [x] 서버 회신 보관본에서 평문 key 제거 및 비밀 파일 ignore 적용
- [x] 실제 원격 기준을 `jjproject_10_260904` / `461f9045...`로 확인
- [x] 기준 commit의 `api_server.py`, `init_db.py`, `vlm.py`, `run.sh`,
  `map_view.py` 소스 대조
- [ ] 같은 commit의 `openapi.json` 확보
- [ ] 서버 IP, 포트, 새 key 전달 채널 확정
- [ ] 서버의 SQLite schema와 API request model을 Jetson fixture로 보관

완료 기준: 비밀값이 Git 대상 파일에 없고, 서버 소스와 OpenAPI가 같은 commit을
가리킨다.

### Phase 1 — API 계약 고정과 자동시험

대상 파일:

- `mission_upload_core.py`
- `test_mission_upload_core.py`
- 신규 server-contract fixture 또는 test

작업:

- [x] `/health`, map upload, mission POST/PATCH, route batch, detection와 image의
  실제 method/path/content-type 대조
- [x] `yaml_file`, `pgm_file`, detection image `file`, `checksum` field 확정
- [x] 서버 request model에 맞춰 전송 payload에서 로컬 전용 필드 제거
- [x] 성공 ACK의 mission/detection/map ID와 image checksum 검증
- [ ] 같은 UUID+같은 payload는 성공, 다른 payload는 `409`가 되는 계약시험
- [x] Go2 지도 test vector가 양쪽에서 같은 map ID를 만드는지 고정시험
- [ ] 서버 OpenAPI가 바뀌면 테스트가 실패하도록 fixture version 기록

완료 기준: 하드웨어 없이 Jetson client와 실제 서버 API의 모든 쓰기 endpoint가
자동시험을 통과한다.

### Phase 2 — outbox와 임무 수명주기 보강

대상 파일:

- `mission_data_core.py`
- `mission_uploader_node.py`
- `h753_mission_data.yaml`

작업:

- [x] 네트워크/timeout/HTTP `408, 429, 5xx`는 `FAILED` 후 재시도
- [x] HTTP `400, 401, 403, 404, 409, 413`은 자동 무한 재시도하지 않고
  `BLOCKED` 또는 dead-letter 상태로 격리
- [x] 인증·schema·ID 충돌 원인을 `/mission/upload/status`에 구분해 표시
- [x] 설정 수정 후 BLOCKED 항목을 운영자가 다시 시도할 service 제공
- [x] map 변경 종료 상태 `MAP_CHANGED`를 서버 허용값 `ABORTED`와
  `end_reason=MAP_CHANGED`로 분리
- [ ] mission 종료 PATCH의 ACK와 최종 서버 status 확인
- [x] 지도 ACK 전 mission, mission ACK 전 route/detection, detection ACK 전 image
  순서 회귀시험 확대
- [ ] ACK 유실 뒤 같은 UUID 재전송 시 기존 서버 row를 성공으로 인정
- [ ] SENT 데이터 보관 기간, 디스크 상한과 안전한 정리 정책 추가

완료 기준: 재시작과 네트워크 장애 후에도 데이터 순서와 ID가 유지되고 영구 오류가
무한 요청으로 서버를 압박하지 않는다.

### Phase 3 — Jetson detection ID와 VLM DB 연결

대상 파일:

- `mission_data_recorder_node.py`
- `vlm_gateway_node.py`
- `vlm_gateway_state.py`
- 관련 YAML과 test

서버 협업 대상:

- 서버 `vlm.py`

작업:

- [x] Jetson gateway가 `/mission/detection_event`를 VLM 판단의 active context로 추적
- [x] payload에 `schema_version`, `robot_id`, `mission_id`, `detection_id`,
  `map_id`, 탐지시각, robot pose, `route_end_seq`, `image_id` 포함 확인
- [ ] 서버 VLM의 자체 mission/detection UUID 생성 경로 제거
- [ ] 서버가 event를 받기 전에 YOLO 신호만으로 별도 DB detection을 만들지 않게 함
- [ ] 서버 VLM 결과가 입력과 같은 mission/detection ID를 반환하도록 시험
- [x] `/vlm/decision` 검증 코드는 구현됐지만 현재 범위에서는 사용하지 않음
- [x] `require_correlated_decision=false` 유지
- [ ] 서버 VLM 판단 결과를 같은 Jetson detection ID의 DB 정보에 연결

완료 기준: 한 번의 Jetson detection ID가 서버 detections, images,
vlm_assessments, victim과 Jetson 결과 로그에서 모두 동일하다.

서버는 VLM 결과를 Jetson detection ID에 연결해 DB와 GUI에서 같은 사람 정보로
조회할 수 있게 한다. 로봇 주행 재개 신호는 기존 `/vlm/injury_stop=0`을 사용한다.

### Phase 4 — 네트워크와 배포 설정

- [ ] Jetson `api_base_url`을 서버 실제 IP와 포트로 설정
- [ ] 격리된 개발 LAN에서만 `allow_insecure_http=true` 사용
- [ ] 운영망은 HTTPS, VPN 또는 접근 제한된 전용 공유기를 사용
- [ ] `curl http://<server>:8000/api/v1/health` 연결 확인
- [ ] 양쪽 `ROS_DOMAIN_ID=30`, 시간 동기화와 DDS discovery 확인
- [ ] `/mission/detection_event`, `/vlm/status`, `/vlm/result_detail`, 카메라 토픽
  양방향 도달 확인
- [ ] 학교 Wi-Fi의 client isolation 또는 multicast 차단 시 전용 공유기·유선 LAN,
  VPN 또는 DDS discovery server 사용
- [ ] API key 누락·오류 시 모터 상태와 무관하게 업로드만 차단되는지 확인

완료 기준: ROS 2 discovery와 REST 연결을 각각 독립적으로 확인하고 네트워크가
끊겨도 로컬 정지와 outbox 기록이 유지된다.

### Phase 5 — 비하드웨어 통합시험

- [ ] 서버 DB 복사본과 임시 파일 저장소 사용
- [ ] 실제 Go2 YAML/PGM 업로드 및 동일 map ID 확인
- [ ] mission 시작, 경로 batch, detection, 이미지, mission 종료 전 과정 실행
- [ ] 동일 요청 2회 전송 시 DB row와 파일이 하나만 생기는지 확인
- [ ] 잘못된 key, map ID, schema, checksum과 UUID 충돌 격리 확인

완료 기준: 실제 모터와 센서를 사용하지 않고 정상 연결 상태의 지도·임무·경로·
탐지·이미지 계약이 통과한다.

### Phase 6 — 실제 Mode 4 통합시험

시험은 로봇 주변을 확보하고 첫 목표를 짧게 설정한 상태에서 순서대로 진행한다.

- [ ] RViz의 AMCL pose와 `/mission/path` 일치 확인
- [ ] 첫 장소에서 사람 1 감지, 로컬 즉시 정지와 detection 저장 확인
- [ ] VLM 판정, victim 생성, 대표 이미지와 GUI 빨간 점 확인
- [ ] 명시적 허가 후 주행 재개
- [ ] 다른 장소에서 사람 2 감지 및 별도 ID/victim 확인
- [ ] 사람 1 선택 시 첫 `route_end_seq`까지, 사람 2 선택 시 두 번째까지 경로 확인
- [ ] GUI 경로와 RViz 경로의 목표 위치 오차 `0.10 m` 이내 확인
- [ ] 같은 사람이 화면에 계속 남아 있을 때 재탐지·재정지하지 않는지 확인

완료 기준: 두 사람의 경로·탐지·이미지·VLM·victim 데이터가 서로 섞이지 않고,
VLM 판단 완료 후 기존 방식으로 정상 주행을 재개한다.

### Phase 7 — 문서와 Git 정리

- [ ] 서버 회신 MD의 비밀값 제거와 기준 commit 수정
- [ ] `PROJECT_HANDOFF.md`에 날짜별 구현·시험 결과 추가
- [ ] 원래 계획서의 완료/부분 완료 상태 갱신
- [ ] Git diff에서 DB, image, API key와 환경 파일 제외 확인
- [ ] Jetson과 서버 각각 검증된 commit SHA 기록
- [ ] 최종 통합 태그 또는 release 기준점 생성

## 5. 서버에 보낼 Jetson 답변

| 서버 질문 | Jetson 답변 |
| --- | --- |
| 실제 `robot_id` | `h753_jetson_01` |
| detection ID A/B | B: Jetson이 생성한 ID를 서버 VLM이 사용 |
| 지도 uploader 존재 여부 | 구현 완료, 실제 서버 계약시험 필요 |
| 대표 이미지 uploader 존재 여부 | 구현 완료, multipart 계약시험 필요 |
| `api_base_url` | 서버의 Jetson에서 접근 가능한 IP를 받은 뒤 설정 |
| API key | 노출된 값 폐기 후 새 값을 별도 보안 채널로 수신 |
| mission 종료 | Jetson은 PATCH uploader 구현 완료, 실제 서버와 시험 필요 |

## 6. 서버에 추가 요청할 사항

- 원격에 실제 배포할 branch/commit을 확정하고 회신 문서의 잘못된 ref 정정
- 같은 commit에서 export한 `openapi.json`
- 현재 서버 이미지 ACK `image_id`를 최종 server image ID로 사용할지 확인
- 서버 VLM이 Jetson `/mission/detection_event`를 수신하도록 변경할 담당 범위
- 자체 mission/detection 생성과 임시 route sampler 제거 계획
- pytest 기반 API·migration 자동 회귀시험
- 새 API key와 서버 IP는 문서가 아닌 별도 보안 채널로 전달

## 7. 완료 판정

다음 항목을 모두 만족해야 Jetson-서버 1차 통합 완료로 판정한다.

- [ ] 모든 테이블에서 한 탐지의 `mission_id/detection_id/map_id`가 동일
- [ ] 사람 1과 사람 2가 별도 victim으로 영속 저장
- [ ] 재전송으로 mission, route, detection, image가 중복 생성되지 않음
- [ ] 서버 ACK 전 Jetson 데이터가 삭제되지 않음
- [ ] 서버 미연결 중에도 YOLO 즉시 정지와 로봇 안전 계층 정상
- [ ] GUI와 RViz 경로·탐지 위치 일치
- [ ] 비밀값과 운영 DB·실제 탐지 이미지가 Git에 포함되지 않음

## 8. 이번 1차 통합 범위 밖

- RGB-D 기반 실제 사람 위치 계산
- 장기 동일인 자동 병합과 tracker 적용
- 최적 안전 귀환경로 생성
- 서버 단절 후 자동 복구와 장애 복구 시험
- detection ID 기반 엄격한 주행 해제
- Mode 5의 map 좌표 기록
- WebSocket GUI와 다중 로봇
- PostgreSQL/PostGIS 전환
