# Gazebo Pinky1 Open-RMF 통합

2026-10-22, swPC. 시뮬레이션 전용이다. 운영 fleet `care_move`와 ROS 도메인 0은 그대로 두었다. 이동은 Gazebo `pinky1`만 대상이다.

## 적용된 RMF 구조

운영 RMF(도메인 0)는 이 검증에서 실행하지 않았다. `docker ps`에 기존 `care-rmf-api`는 없었고, 도메인 0의 ROS 노드도 없었다.

시뮬레이션 RMF는 별도 이름과 별도 도메인이다.

| 구성요소 | 위치 | 도메인 |
|---|---|---|
| `rmf_traffic_schedule` | 호스트, `use_sim_time:=false` | 77 |
| `building_map_server` | `config/rmf/gazebo_test_room/gazebo_test_room.building.yaml` | 77 |
| RMF-Web API | 컨테이너 `care-move-gazebo-rmf-api`, `api-server:jazzy-nightly`, host network, 포트 8000 | 77 |
| Fleet adapter | `scripts/gazebo_fleet_adapter.py` | 77 |
| Nav2 gateway | `scripts/gazebo_nav_gateway.py`, TCP `127.0.0.1:17442` | 42 |
| Gazebo / Nav2 | `sim/gazebo/pinky1.launch.py`, `sim/gazebo/nav2.launch.py` | 42 |
| Care-Move API | `uvicorn app.main:app`, `127.0.0.1:8011`, `CARE_MOVE_MODE=rmf` | HTTP만 |

Fleet 이름은 `gazebo_pinky1`이다. 운영 이름 `care_move`와 겹치지 않는다. `config/rmf/care_move_fleet.yaml`의 `allow_motion: false`, `simulation_confirmed: false`, `graph_alignment: placeholder`는 바꾸지 않았다. Gazebo fleet 파일의 `care_move.allow_motion`도 `false`이다. 이동을 허용하는 조건은 YAML 전역 스위치가 아니라, 도메인 77 adapter와 도메인 42 gateway가 각자 검사하는 실행 조건이다.

API 서버의 `use_sim_time`은 `AppConfig` 필드가 아니다. 컨테이너 설정은 `ros_args`로 `--ros-args -p use_sim_time:=false`만 추가한다. Nav2와 Gazebo만 sim time을 쓴다. 도메인 사이 clock bridge는 없다.

스케줄 미러는 약 20초마다 `Requesting new schedule update because update timed out`을 남긴다. 이 로그가 있어도 task 계획과 완료 보고는 이어졌다.

## Fleet Adapter

`scripts/gazebo_fleet_adapter.py`는 시스템 Python과 ROS 2 Jazzy의 `rmf_adapter.easy_full_control`를 쓴다. `.venv`로 띄우지 않는다.

기동 조건:

- `ROS_DOMAIN_ID`가 77이 아니면 종료한다.
- `/dev/ttyAMA0` 또는 `/dev/ttyAMA4`가 있으면 종료한다.
- gateway `health`가 action, scan, clock, pose를 주지 않으면 fleet를 만들지 않는다.

등록:

- `Adapter.make("gazebo_pinky1_fleet_adapter")` 후 `add_easy_fleet`.
- 설정 파일은 `config/rmf/gazebo_test_room/fleet.yaml`와 `nav.yaml`.
- `SIM_RMF_SERVER_URI=ws://127.0.0.1:8000/_internal`일 때만 API 서버 URI를 보낸다.
- 로봇 이름은 `pinky1`. `RobotConfiguration` 파이썬 바인딩은 charger 이름 목록만 받는다. 값은 `["GZ_SPAWN"]`.
- merge 거리는 waypoint 0.3 m, lane 0.5 m, 최소 lane 길이 `1e-8`.

Easy Full Control 콜백:

- `navigate(destination, execution)`는 waypoint 이름을 gateway `go`로 보낸다. `destination.position`, `name`, `map`는 메서드가 아니라 property다.
- map가 `gazebo_test_room`이 아니면 `finished()`를 호출하지 않는다.
- goal이 수락되면 `execution.identifier`를 한 번 저장하고, 이후 `RobotUpdateHandle.update`에 같은 activity를 넘긴다. activity를 빼면 약 10초 뒤 command handle unresponsive로 재계획되고 Nav2 goal이 취소된다.
- `execution.finished()`는 gateway phase가 `succeeded`이고 generation이 같을 때만 호출한다. Nav2 status 4가 그 값이다.
- `stop`은 gateway `cancel`만 호출한다. cancel 응답만으로 RMF 완료를 만들지 않는다.
- `action_executor`는 로그를 남기고 `finished()`를 호출하지 않는다. patrol은 navigate만 사용한다.

## Domain Gateway

RMF 도메인을 42로 바꾸지 않는다. DDS topic을 통째로 중계하지 않는다. 도메인 77 adapter와 도메인 42 Nav2 사이의 연결은 localhost TCP 한 줄 JSON이다.

`scripts/gazebo_nav_gateway.py`:

- `ROS_DOMAIN_ID`가 42가 아니면 기동하지 않는다.
- Pinky 시리얼 장치가 있으면 기동하지 않는다.
- `/cmd_vel`을 publish하지 않는다.
- Action은 `/pinky1/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`).
- TF는 `/tf` → `/pinky1/tf`, `/tf_static` → `/pinky1/tf_static`. `use_sim_time`은 true.
- scan은 `/pinky1/scan`, best effort.
- 소켓 스레드는 명령을 큐에 넣고, rclpy spin은 메인 스레드만 한다.

명령은 `health`, `poll`, `go`, `cancel`이다. `go`는 waypoint 이름이 gazebo graph에 있고, 요청 좌표와 graph 좌표 차이가 5 cm 이하이며, 셀이 비어 있을 때만 Nav2 goal을 만든다. 그래프 밖 이름은 goal을 만들지 않는다. 이미 goal이 있으면 새 goal을 거절한다.

`fleet_adapter/live_nav.py`의 `GoalTracker`가 generation을 올린다. 다른 generation의 result는 `stale`로 버리고 phase를 바꾸지 않는다. Nav2 status 4는 succeeded, 5는 canceled, 6은 aborted다. cancel을 요청한 직후의 `result_confirmed`는 false이고, Nav2 status가 들어온 뒤에만 true가 된다.

gateway TCP가 끊기면 adapter는 `gateway_lost`를 남기고 위치를 완료로 보고하지 않는다. 연결이 돌아온 뒤 새 goal을 스스로 만들지 않는다.

## Nav2

기존 launch와 파라미터는 그대로다. 확인된 action 서버는 `/pinky1/navigate_to_pose`이고 `xy_goal_tolerance`는 0.2 m이다. 맵은 `sim/nav2/map/pinky1_room.yaml`, origin `[-4, -4]`, resolution 0.05 m이다.

이번 통합이 Nav2 goal을 성공으로 보는 조건은 action result status 4뿐이다. RMF가 먼저 completed를 찍어도 gateway phase가 succeeded가 아니면 그 주행은 성공이 아니다.

## 그래프 좌표

RMF 레벨 이름과 Nav2로 넘기는 맵 이름은 `gazebo_test_room`이다. 변환은 identity다. `tx = ty = yaw = 0`, `scale = 1`. 이 값이 아니면 `nav_pose`가 거절한다.

| Waypoint | x | y | 역할 |
|---|---|---|---|
| `GZ_SPAWN` | 0.0 | 0.0 | 스폰, charger, parking |
| `GZ_EAST` | 1.2 | 0.0 | 동쪽 목적지 |

Lane은 양방향이고 속도 제한은 0.2 m/s이다. 로봇 footprint 0.3 m, vicinity 0.5 m, 선속도 한계 0.2 m/s, 각속도 한계 0.5 rad/s. building map의 lane에는 `graph_idx`가 있어야 nav graph에 남는다. `coordinate_system`은 `cartesian_meters`라 y를 뒤집지 않는다.

1F·7F 개념 waypoint와 `config/rmf/care_move_fleet.yaml` graph는 이 좌표로 덮지 않았다. Care-Move `POST /api/tasks`는 `CARE_MOVE_MODE=rmf`일 때 개념 waypoint와 live graph 이름을 둘 다 허용한다. mock 모드에서는 개념 waypoint만 허용한다.

## Task 상태

```text
POST /tasks/robot_task
        |
        v
queued → pinky1 배정 → underway
        |
        v
adapter navigate → gateway go → Nav2 accepted
        |
        +-- status 4 → execution.finished() → RMF completed
        |
        +-- cancel → Nav2 status 5 → RMF canceled
        |
        +-- status 6 또는 gateway 단절 → finished() 호출 없음
```

Care-Move는 RMF 상태를 자기 task에 복사한다. `completed: true`는 RMF status가 `completed`일 때만이다. 접수 응답의 `success: true`는 주행 완료가 아니다.

대시보드 `GET /`는 `/api/state`의 task 목록을 보여 준다. 목적지 목록에는 개념 waypoint 뒤에 live `graph_waypoints`(`GZ_EAST`, `GZ_SPAWN`)를 붙인다. 상단 배너의 "주행 검증: 아님"은 운영 motion 플래그이며, task 행의 `completed`와 별개다.
