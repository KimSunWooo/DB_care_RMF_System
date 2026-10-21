# Care-Move RMF Starter · 관리자 관제 보일러플레이트

원본 기획안: **Care-Move — LLM 및 다중 관제 기반 요양원 자율주행 휠체어** (첨부 PDF, 1~5페이지).

교육·통합개발을 위한 **최소 관리자 대시보드**. Open-RMF 학습을 시작하려는 2인 관제팀(9일 스프린트)을 위해 ROS/Isaac Sim이 없어도 켜지는 Mock 앱으로 설계했습니다.

**절대로 실제 휠체어나 엘리베이터에 직접 연결해 사용하지 마세요.** 의료/승강기/승객 안전 요구사항을 검증한 소프트웨어가 아닙니다.

## 1. 구성·기능

```text
Web Admin (single-page HTML/CSS/JS)
 ├── REST POST /api/tasks ───────────┐
 ├── REST POST /api/tasks/preview    │
 └── WebSocket /ws/state            ▼
                             FastAPI Care-Move
                               ├── MOCK mode (default)
                               │   ├── 3 robot simulators
                               │   └── single mock elevator
                               └── RMF mode (opt-in)
                                   └── RMF-Web API :8000
                                       ├── POST /tasks/dispatch_task
                                       └── POST /tasks/robot_task
                                        (External Fleet Adapter / Nav2 / Isaac Sim)
```

- 대시보드: 1F/7F 개념 지도, 로봇 3대 위치(모의 모드), 상태, 임무 목록, 승강기 상태.
- 임무: Fleet 자동 배정 또는 `pinky1/2/3` 선택 후 목적지 `patrol` 요청.
- 로봇 확장: `config/robots.json` 배열에 새 로봇을 추가하면 서버 재시작 후 반영. **실제 RMF mode에서는 Fleet Adapter에도 반드시 등록해야 합니다.**
- 엘리베이터: Mock으로 1F↔7F 상태 변화 실습; RMF 모드의 직접 운전 버튼은 비활성화.
- WebSocket: 브라우저에 약 0.6초 간격으로 상태 스냅샷 전송 (실제 ROS 2 이벤트 스트림 아님).

## 2. 설치 및 실행 (Ubuntu 24.04 / Python 3.12 예시)

```bash
cd care-move-rmf-starter
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# 1) 가상 로봇 모드 (환경 변수 생략 시 기본값)
export CARE_MOVE_MODE=mock
uvicorn app.main:app --host 127.0.0.1 --port 8010
```

브라우저에서 http://127.0.0.1:8010 을 열고 **로봇/목적지 선택 → 임무 요청 전송**을 클릭합니다.

운영자 UI가 작동하는데도 RMF를 설치해야 하는 것은 아닙니다. Mock 모드에서는 RMF 없이 앱 동작을 배웁니다.

API Swagger UI: http://127.0.0.1:8010/docs

## 3. API 학습

### 요청 미리보기 (실제 Open-RMF에 보내지 않음)

```bash
curl -X POST http://127.0.0.1:8010/api/tasks/preview \
  -H 'Content-Type: application/json' \
  -d '{"robot_id":"pinky1", "places":["N_GYM"],"rounds":1}'
```

`body`가 Open-RMF용 `robot_task_request` JSON으로 반환됩니다.

### Mock 배차

```bash
curl -X POST http://127.0.0.1:8010/api/tasks \
  -H 'Content-Type: application/json' \
  -d '{"robot_id":null, "places":["N_ROOM102","N_EV1","N_GYM"],"rounds":1}'
```

로봇 한 대가 Mock 경로의 각 지점을 이동하고 `1F → 7F`에서 Mock 엘리베이터 상태가 바뀝니다. 이때 **교통 충돌 회피·실제 탑승·문 센서 인터락·정밀 도킹은 구현하지 않았습니다.**

### Mock 엘리베이터 단독 시험

```bash
curl -X POST http://127.0.0.1:8010/api/elevator/mock-travel \
  -H 'Content-Type: application/json' \
  -d '{"robot_id":"pinky1","target_floor":"7F"}'
```

### 3대 → 4대 확장

`config/robots.json`의 `robots` 배열에 예를 들어 아래를 추가합니다. `config/waypoints.json`에 지정된 출발 노드만 사용해야 합니다.

```json
{"id":"pinky4", "label":"Pinky #4", "initial_waypoint":"N_DOCK1", "battery":100}
```

서버 재시작 후 `GET /api/state`에서 개수가 4가 됩니다. Fleet 자동 배정은 Mock에서 단순히 첫 번째 유휴 로봇을 선택합니다. 이는 Open-RMF의 경매 기반 배차와 다릅니다.

## 4. 실제 Open-RMF API 연동 (추가 시스템 구동 필요)

Open-RMF/RMF-Web API 서버가 실행 중이고, Fleet Adapter가 `care_move` Fleet과 `pinky1~3`을 등록하며 `patrol` 작업을 지원해야 합니다.

```bash
# 두 번째 쉘: 실제 RMF 작업 요청 모드, 서버 재시작
export CARE_MOVE_MODE=rmf
export RMF_API_URL=http://127.0.0.1:8000   # 이 실험실에서 확인된 API. 코드는 포트를 가정하지 않음
export RMF_FLEET_NAME=care_move
# 인증 모드는 .env 의 RMF_AUTH_MODE. .env.example 은 읽지 않는다.
uvicorn app.main:app --host 127.0.0.1 --port 8010
```

프로세스는 시작 시 `care-move-rmf-starter/.env`만 읽습니다. 이미 셸에 같은 변수가 있으면 그 값이 우선합니다. `.env`를 바꾼 뒤나 이 코드를 처음 반영할 때는 uvicorn을 한 번 재시작해야 합니다. 그 다음부터 `local_jwt` 모드의 토큰 교체는 프로세스 메모리에서 이루어지며 `.env`를 다시 고치거나 서버를 재시작할 필요가 없습니다. `uvicorn --env-file .env`로 띄워도 `.env.example`은 사용하지 않습니다. 주소 변수명은 기존 `RMF_API_URL`입니다.

`RMF_AUTH_MODE=static`은 기존 `RMF_API_TOKEN`만 사용합니다. `local_jwt`는 `RMF_JWT_SECRET`으로 HS256 JWT를 발급하고, 설정이 빠지면 static 모드로 바뀌지 않습니다. 기본 유효기간은 3600초, 교체 주기는 2700초, 만료 안전 여유는 60초입니다. 서명 비밀값은 `.env`에만 둡니다. 조회 검증은 `scripts/verify_rmf_jwt.py`를 사용합니다.

`GET /api/rmf/status`가 `ready`이고 Fleet `care_move`와 navigation graph waypoint가 확인되기 전에는 RMF로 작업 POST를 보내지 않는다. 8080은 이 환경의 API 포트가 아니다. 연결 실패, 인증 오류, RMF의 `success: false`는 작업 성공으로 기록하지 않는다.

### 꼭 확인할 점

1. `http://127.0.0.1:8000/docs`에서 RMF API 서버 실행·작업 엔드포인트 확인.
2. RMF 그래프 waypoint 이름이 이 앱 `config/waypoints.json`과 일치할 것. **PDF 개념도만으로는 RMF navigation graph가 생성되지 않습니다.**
3. `POST /api/tasks` 202는 RMF의 작업 접수 응답이며 완료를 보장하지 않음.
4. RMF 모드의 로봇 위치는 `GET /fleets` 폴링으로만 갱신한다. 샘플이 없으면 `unverified`, 수신이 끊기면 `stale`이며 이전 좌표를 현재 위치로 유지하지 않는다. 개념 지도에는 Mock 로봇만 그린다.
5. **엘리베이터 실제 제어도 미구현**. ROS 2 `rmf_lift_msgs`와 별도 Lift Adapter가 필요함.
6. 인증 없음: 서버 기본 바인딩 127.0.0.1 유지. 외부 노출 금지.

작업/리프트 상세 검증 사항: `docs/INTEGRATION.md`, `docs/ELEVATOR_PROTOCOL.md`.

## 5. 전체 파일 구조

```text
care-move-rmf-starter/
├── app/
│   ├── __init__.py
│   ├── main.py               # FastAPI, RMF HTTP 명령, Mock simulator
│   ├── rmf_token.py          # 서버 측 JWT 발급·교체
│   └── static/index.html     # 관리자 관제 UI
├── config/
│   ├── robots.json           # 로봇 3대 / 증설
│   └── waypoints.json        # PDF 1F/7F 기반 개념도 (실측 좌표 아님)
├── docs/
│   ├── INTEGRATION.md        # Isaac Sim + Nav2 + RMF Adapter 연결 로드맵
│   └── ELEVATOR_PROTOCOL.md  # HTTP/WS/Serial/ROS2 lift 통신 전략
├── tests/test_api.py         # Mock API 스모크 테스트
├── requirements.txt
├── .env.example
└── README.md
```

## 6. 테스트

```bash
PYTHONPATH=. pytest -q
```

### 2인 × 9일 스프린트에 적용

| 스프린트 | Backend/Open-RMF 담당 | Frontend/관제 담당 |
|---|---|---|
| D1–3 | RMF API 데모, graph·waypoint·Task 계약 검증 | 이 화면 커스터마이징, 1F/7F 맵·상태 카드 |
| D4–6 | Isaac Sim Fleet Adapter 연동, RMF task 상태 조회 | 실로봇 위치·상태 WebSocket 연결, 임무 UI 개선 |
| D7–9 | Lift Adapter 시험, 다중로봇 시나리오, 통합 로그 | 엘리베이터 UI, 오류/재연결, 통합 E2E 시연 |

**우선순위: 실제 RMF 작업 요청 성공 > 실제 로봇 상태 수신 > 층간 lift 시뮬레이션 통합 > 예외 처리.** 화장실 Lock, 경로 재계획, 도킹 제어는 이 스타터에 포함되지 않으며 다른 담당자와 책임 분리를 먼저 확정합니다.

## 공식 참고

- https://github.com/open-rmf/rmf-web (RMF API/server/dashboard)
- https://github.com/open-rmf/rmf_api_msgs (task_request/robot_task_request/dispatch_task_request JSON schema)
- https://github.com/open-rmf/rmf_ros2 (ROS2 RMF integration)
- https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg
- https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/robot_control/tutorial_ros2_multi_navigation.html
