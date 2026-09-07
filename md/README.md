# 프로젝트 문서 안내

이 디렉터리는 H753 자율주행 로봇 프로젝트에서 직접 작성하고 유지하는 문서를
한곳에서 관리한다.

## 문서 목록

- [`PROJECT_HANDOFF.md`](./PROJECT_HANDOFF.md): 날짜별 개발일지, 현재 검증 상태,
  실행 모드, 안전 규칙과 다음 우선순위의 단일 기준 문서
- [`SERVER_PC_DB_VLM_GUI_DEVELOPMENT_REQUEST_20260907.md`](./SERVER_PC_DB_VLM_GUI_DEVELOPMENT_REQUEST_20260907.md):
  2026-09-07에 확정한 최소 범위 기준의 서버 DB/VLM/GUI 개발 요청서.
  **현재 서버 PC 담당자에게 전달할 기준 문서는 이 파일 하나다.**
- [`ROBOT_ROUTE_DATABASE_GUI_DEVELOPMENT_PLAN.md`](./ROBOT_ROUTE_DATABASE_GUI_DEVELOPMENT_PLAN.md):
  Jetson과 서버의 경로·탐지·DB·VLM·GUI 장기 통합 계획과 이전 논의 기록.
  현재 서버 작업 지시는 위 최신 최소 범위 요청서를 우선한다.
- [`SYSTEM_ARCHITECTURE_CURRENT_TARGET.md`](./SYSTEM_ARCHITECTURE_CURRENT_TARGET.md):
  현재 구현과 목표 시스템 구조, Jetson/서버 역할 분담
- [`JETSON_SERVER_INTEGRATION_DEVELOPMENT_PLAN_20260907.md`](./JETSON_SERVER_INTEGRATION_DEVELOPMENT_PLAN_20260907.md):
  서버 회신을 반영한 Jetson 개발·통합시험 실행 계획
- [`DB_개발현황_서버연동_노션기록_20260907.md`](./DB_개발현황_서버연동_노션기록_20260907.md):
  현재 DB 구조, 날짜별 개발 내용, 서버 GitHub 확인 결과, 최신 서버 작업과
  공동 통합시험을 Notion용 Markdown으로 정리한 최신 기록

서버 PC에서 전달받은 원본 체크리스트·메신저 메모와 Markdown 전환 전 TXT는
로컬 참고 자료로만 보존하며 GitHub에는 올리지 않는다. 해당 자료의 검토 결과와
필요한 계약만 위 개발계획·개발일지에 정리한다.

## 관리 기준

- 날짜별 실제 개발·검증 결과는 `PROJECT_HANDOFF.md`에 추가한다.
- 데이터 계약이나 역할 분담 변경은 DB/GUI 개발 계획서에 먼저 반영한다.
- 전체 구성 요소나 데이터 흐름이 바뀌면 시스템 아키텍처 문서를 갱신한다.
- 외부에서 전달받은 체크리스트는 로컬에 원본 상태로 보존하고 Git에는 포함하지
  않으며, 검토 결과만 개발 계획서에 기록한다.
- 패키지별 `README.md`, YDLIDAR·micro-ROS 외부 문서와 pytest 자동 생성 문서는
  각 패키지의 원래 위치에서 관리한다.
