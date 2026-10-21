# 실제 Open-RMF + Isaac Sim 통합 체크리스트

## 0. 2026-10-20 실행 환경에서 확인한 사실

이 절은 해당 시점의 호스트를 조회한 결과다. 이후 프로세스가 바뀌면 다시 확인한다.

| 항목 | 확인 결과 |
|---|---|
| `care-rmf-dashboard` | 실행 중. 이미지 `ghcr.io/open-rmf/rmf-web/demo-dashboard:jazzy-nightly`. `http://127.0.0.1:3000` |
| `care-rmf-api` | 실행 중. 이미지 `ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly`. `http://127.0.0.1:8000` |
| 8080 | 리스닝하지 않음. API 주소로 쓰지 않는다. |
| 8006 trajectory | 대시보드 환경변수만 있고 프로세스는 없다. |
| OpenAPI | `GET /openapi.json`, `GET /docs`는 인증 없이 200. 제목 `RMF API Server`. |
| 인증 | `GET /fleets`, `/tasks`, `/lifts`, `/building_map`은 Bearer 없이 401 `Not authenticated`. |
| 작업 경로 | `POST /tasks/dispatch_task`, `POST /tasks/robot_task`, `POST /tasks/cancel_task`, `GET /tasks/{task_id}/state`가 스키마에 있다. |
| 실시간 | `/socket.io` 핸드셰이크는 된다. Care-Move는 추가 의존성 없이 HTTP 조회로 상태를 읽는다. |
| ROS | 호스트 `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`에서 노드 `/rmf_api_server`만 보인다. |
| Fleet | `/fleet_states` 없음. Fleet Adapter 프로세스 없음. `navigate_to_pose` 액션 없음. |
| 맵/시계 | `/map` 발행자 0. `/clock` 샘플 없음. API 설정은 `use_sim_time:=true`이고 `GET /time`은 `0`. |
| 패키지 | `rmf_fleet_adapter`, `rmf_fleet_adapter_python`은 설치됨. `free_fleet`은 `ros2 pkg list`에 없다. |

Care-Move는 `RMF_API_URL`이 비어 있으면 포트를 추측하지 않는다. `.env.example`은 예시일 뿐 로드하지 않는다. 파일을 수정한 뒤에는 프로세스를 한 번 재시작한다. 401이면 기본 30초 동안 인증 조회를 반복하지 않는다. Fleet `care_move`, 로봇 상태, navigation graph waypoint가 확인되기 전에는 `POST /api/tasks`가 RMF로 나가지 않는다. 인증 성공만으로 운행 준비가 되지 않는다.

## 0.3 서버 측 JWT

실행 중인 `care-rmf-api`는 `jwt_public_key`가 없고 HMAC 비밀값으로 검증한다. sqlite 로컬 설정은 `db_url`, 캐시, `use_sim_time`, 로그만 덮어쓰고 JWT 항목은 이미지 기본값 그대로다. issuer는 `stub`, audience는 `rmf_api_server`, 사용자 claim은 `preferred_username`이다. DB 사용자 `admin`은 `is_admin=1`이다. 알고리즘 후보는 서버가 여러 개를 받지만 Care-Move는 `RMF_JWT_ALGORITHM=HS256`으로 고정한다. 잘못된 bearer는 401 `Not authenticated`다. 관리자 전용 라우트는 `is_admin`이 아니면 403이며, Care-Move는 403에서 토큰을 다시 발급하지 않는다.

`RMF_AUTH_MODE=static`은 `RMF_API_TOKEN`을 그대로 보낸다. `local_jwt`는 프로세스마다 메모리에 토큰을 두고 3600초 유효, 2700초마다 교체, 만료 60초 전에 재발급한다. 교체 시각은 monotonic clock, `iat`/`exp`는 UTC wall clock이다. ROS `/clock`은 쓰지 않는다. 여러 프로세스를 띄우면 캐시도 프로세스별로 나뉜다. 비밀값은 Git에 넣지 않는다.

```bash
.venv/bin/python scripts/verify_rmf_jwt.py
RMF_JWT_TTL_SECONDS=90 RMF_JWT_ROTATE_INTERVAL_SECONDS=30 RMF_JWT_EXPIRY_SKEW_SECONDS=10 \
  .venv/bin/python scripts/verify_rmf_jwt.py --watch
```

2026-10-21 재확인: JWT `local_jwt`로 `GET /fleets`와 `GET /tasks`는 200 빈 목록, `GET /building_map`은 404, `GET /time`은 `0`이다. Nav2 액션과 Isaac Sim 프로세스는 없다. `rmf_traffic_schedule`은 이 확인 중에 `use_sim_time:=false`로 새로 띄웠고, API 서버의 `use_sim_time`은 true라 `/clock` 샘플은 없다. Fleet Adapter 구현과 미완료 항목은 `docs/INTEGRATION_STATUS.md`에 있다.

## 0.1 Fleet Adapter 선택

Pinky Nav2는 이름공간 아래의 상대 액션 `navigate_to_pose`를 사용한다. 기본 launch 인자 `namespace`는 빈 문자열이다. `/pinky1/navigate_to_pose`는 그 인자로 띄웠을 때의 후보이며, 이번 조회에서는 액션 서버가 없었다.

| 기준 | `free_fleet` | `rmf_fleet_adapter_python` 커스텀 어댑터 |
|---|---|---|
| Pinky Nav2 | 로봇 쪽에 Free Fleet 클라이언트가 더 필요함 | `NavigateToPose` 액션에 직접 연결 가능 |
| 현재 설치 | 패키지 없음 | 호스트에 패키지 있음 |
| Jazzy | 이 머신에서 미확인 | ROS 2 Jazzy 패키지로 설치되어 있음 |
| 다중 로봇 | 로봇마다 Free Fleet 엔드포인트 | 이름공간 `pinky1`부터 설정으로 추가 |
| 상태 피드백 | Free Fleet 상태 채널 | 어댑터가 `RobotUpdateHandle`로 RMF에 보고 |
| 소요 | 프로토콜 어댑터를 새로 맞추는 시간 | 템플릿 구현과 그래프/맵 정합이 주 작업 |

다음 연결은 **Pinky 1대, 이름공간 `/pinky1`, 액션 `/pinky1/navigate_to_pose`** 로 `rmf_fleet_adapter_python` 기반 어댑터를 만드는 쪽이 맞다. `free_fleet`은 패키지도 없고 Pinky 스택이 그 프로토콜을 말하지 않는다. 3대 확장은 같은 어댑터 설정에 `pinky2`, `pinky3` 이름공간을 더하는 작업이다. 어댑터를 띄우기 전에 필요한 것:

1. Isaac Sim 또는 동등 시뮬에서 `/pinky1/navigate_to_pose`가 실제로 보일 것.
2. RMF building map / navigation graph. waypoint 이름이 작업 요청과 같아야 한다. `config/waypoints.json`의 x,y는 화면 좌표다.
3. Fleet 설정 이름 `care_move`, 로봇 이름 `pinky1`.
4. API 서버와 어댑터의 `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 일치.
5. `use_sim_time`을 쓸 경우 `/clock` 발행. 지금은 API만 sim time이고 clock 샘플이 없다.
6. `RMF_API_TOKEN`. 대시보드가 `/fleets`에 200을 받는 것과 Care-Move 토큰은 별개다.

## 0.2 엘리베이터

실제 `LiftRequest`는 보내지 않는다. 상태 이름과 Gateway 미리보기 JSON은 `app/elevator.py`와 `GET /api/elevator/interface`에 있다. QR 성공은 탑승 완료가 아니다. 확인 항목은 탑승 허가, 문 닫힘, 목적층 도착, 하차다.

이 저장소는 **Open-RMF 설치 패키지나 Nav2 Fleet Adapter 자체가 아닙니다**. 실제 모드에서는 RMF-Web API에 작업 요청을 보내는 얇은 관제 클라이언트입니다.

## A. 학습 단계 (권장 순서)

1. `CARE_MOVE_MODE=mock`에서 UI → 서버 → 3대 Mock 상태 갱신 확인.
2. `/api/tasks/preview`로 RMF 요청 JSON 관찰.
3. 공식 Open-RMF demos + RMF-Web API 서버 별도로 구동. API 서버 `http://127.0.0.1:8000/docs`에서 `POST /tasks/dispatch_task` 확인.
4. `CARE_MOVE_MODE=rmf`로 서버 재시작하고 **실제 RMF graph에 존재하는 waypoint만** 요청.
5. Pinky(또는 Isaac Sim 지원 로봇)의 Nav2와 Fleet Adapter 연결 및 상태 전송 통합.
6. RMF lift adapter ↔ Isaac Sim 엘리베이터 상태/물리 제어 통합.

## B. 필수 외부 구현 (이 ZIP에 없음)

- Isaac Sim scene, ROS 2 Bridge, /clock(use_sim_time), Pinky 3대 네임스페이스(`/pinky1..3`).
- robot TF, odom, localization, Nav2 action 서버 각각의 데이터·명령 채널.
- RMF navigation graph / fleet config / lifts 및 1F-7F 층간 연결.
- Fleet Adapter가 RMF의 이동 요청을 해당 로봇 Nav2 action으로 전달하고 실제 피드백을 RMF로 송신.
- RMF-Web API 상태 조회를 Care-Move UI에 실시간 반영하는 별도의 상태 Adapter.
- elevator `rmf_lift_msgs` bridge. `docs/ELEVATOR_PROTOCOL.md` 참고.

**위 항목 없이는 대시보드 버튼을 눌러도 Isaac Sim 로봇이 실제 이동하지 않습니다.**

## C. 그래프/좌표 매칭

`config/waypoints.json`은 원본 제안서 4~5페이지를 바탕으로 만든 화면 비율 좌표입니다. 값은 **Nav2 Pose나 SLAM 좌표가 아닙니다.** 실제 RMF 모드의 이름과 1F/7F floor/map name을 일치시키고, RMF graph를 먼저 구축해야 합니다. 이름이 맞더라도 Fleet Adapter가 patrol task category를 지원하지 않으면 RMF에서 거절될 수 있습니다.

## D. RMF API 작업 요청 예시

Fleet 자동 배정:

```json
{
  "type": "dispatch_task_request",
  "request": {
    "category": "patrol",
    "description": {"places": ["N_ROOM102", "N_EV1"], "rounds": 1},
    "requester": "care-move-dashboard",
    "fleet_name": "care_move"
  }
}
```

특정 로봇 지정:

```json
{
  "type": "robot_task_request",
  "robot": "pinky1",
  "fleet": "care_move",
  "request": {
    "category": "patrol",
    "description": {"places": ["N_GYM"], "rounds": 1},
    "requester": "care-move-dashboard",
    "fleet_name": "care_move"
  }
}
```

RMF API server는 작업 요청을 접수/검증하고 실패 시 오류를 반환합니다. 성공은 완료를 의미하지 않습니다. 해당 버전이 지원하는 API 경로와 스키마를 **실제 배포 서버의 `/docs`에서 우선 검증**하세요.

RMF 내부 실제 작업 상태: `GET /tasks/{task_id}/state` 등을 별도 연동해야 합니다. Starter는 RMF 응답 원문만 내부 기록에 남기고 **로봇 상태는 unverified**로 표시합니다.

시뮬레이션 `/clock`을 사용하는 RMF-Web API server라면 공식 지침에 따라 `RMF_SERVER_USE_SIM_TIME=true` 설정 검토.

참조:
- https://github.com/open-rmf/rmf-web
- https://github.com/open-rmf/rmf_api_msgs
- https://github.com/open-rmf/rmf_demos
- https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/robot_control/tutorial_ros2_multi_navigation.html
