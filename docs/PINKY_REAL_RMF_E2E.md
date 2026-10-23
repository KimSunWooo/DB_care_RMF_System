# 실제 Pinky1 RMF

2026-10-22. 실제 Pinky에는 RMF task를 보내지 않았다. 도메인 77의 Gazebo fleet `gazebo_pinky1` / `pinky1`은 변경하지 않았다.

`fleet_adapter/real_gate.py`의 `real_task_block()`은 기본적으로 명령을 거부한다. 로봇 ID는 `real_pinky1`만 받는다. RMF 도메인 0, 42, 53, 77, 90은 거부한다. Nav2 도메인은 53만 허용한다. `commands_enabled`, 운용자 정지 승인, safety, Nav2, graph, 단거리 trial이 모두 참일 때만 거부 이유가 없다. 현재 실기체는 그 조건을 만족하지 않는다.

| 항목 | 결과 |
|---|---|
| Fleet 등록 | BLOCKED |
| Task dispatch | BLOCKED |
| 실제 이동 | BLOCKED |
| Nav2 SUCCEEDED | BLOCKED |
| RMF completed | BLOCKED |
| 관리자 UI | BLOCKED |
| Gazebo ID 유지 | PASS. `pinky1` / `gazebo_pinky1` 거부 테스트 |
| 도메인 분리 | PASS. RMF 도메인 53은 거부하고, Nav 대상 53은 별도 조건이 맞을 때만 허용하는 테스트를 유지 |

Nav2 단일 goal과 현장 정지 승인이 생기기 전에는 이 게이트를 열지 않는다.

## 2026-10-23 Dry-run

`192.168.0.219`의 호스트는 `raspi`다. 실행 중인 것은 domain 53, `rmw_fastrtps_cpp`의 `sllidar_node`, `robot_state_publisher`, `joint_state_publisher`뿐이다. `/odom`, `/cmd_vel`, `/map`, Nav2 action은 없다. 바퀴 관절은 `joint_state_publisher`가 0으로 내는 값이다.

`scripts/physical_pinky_gateway.py`는 `127.0.0.1:17453`에서 구독만 한다. `scripts/physical_fleet_adapter.py`는 gateway 결과를 읽고 `Adapter.make`를 호출하지 않는다. Gazebo fleet과 `GZ_EAST` / `GZ_SPAWN`은 실물 목적지로 쓰지 않는다. `PHYSICAL_ARMED` 기동은 이 프로세스에서 거부한다.

Care-Move `/api/physical/status`는 이 gateway의 읽기 결과만 보여 준다. pose는 null이다.

## 2026-10-23 11:10 재확인

Motor bringup, `/cmd_vel`, `/odom`은 다시 확인했을 때 없었다. `joint_state_publisher`와 LiDAR 프로세스도 그 시점에는 없었다. 직후 `192.168.0.219` ping이 실패해 watchdog 패치와 bringup 재시작은 하지 않았다. `bringup.py.bak-20261023`은 만들어지지 않았다.

`PHYSICAL_TEST`는 최대 0.04 m, 전방, fresh odom일 때만 `/cmd_vel`을 내도록 코드에 넣었다. 이번 프로세스에는 그 모드를 켜지 않았다. Fleet 등록과 실물 이동은 하지 않았다.

## 2026-10-23 11:38 재연결

개발 PC는 Pinky AP `192.168.4.1`에 붙어 있다. 호스트는 `raspi`이고 SSH 키는 기존 `192.168.0.219`와 같다. 주소 `192.168.0.219`는 응답하지 않았고, Pi 주소는 `192.168.0.83`과 `192.168.4.1`이다. Bringup은 ROS domain 24에서 동작 중이었다. 설치본과 소스에 0.4초 명령 watchdog을 넣고 bringup을 다시 시작했다. 백업은 `bringup.py.bak-20261023`이다.

측정 시작 pose는 `(0, 0, 0)`, 목표는 전방 0.04 m였다. 결과는 odom x `0.022 m`에서 정지했다. 왼쪽 바퀴 관절은 약 5.62 rad에서 6.42 rad로 변했다. 같은 요청의 재전송은 거부했다. Gateway는 다시 `PHYSICAL_DRY_RUN`이다. RMF fleet 등록과 task는 하지 않았다. 목표 0.04 m에는 도달하지 못했다.

## 2026-10-23 12:32 UI 5cm task

Care-Move 화면의 `5cm 이동 테스트 (RMF)`로 제출했다. Booking `ea9c24d5-57d8-4088-b951-c4ea1b0a1fe7`, correlation `7325f523f32241218c2a464289f03667`, session `162b1c71fb2247a3bf8d5a4bd06e8b12`. Fleet `physical_pinky`, robot `real_pinky1`.

출발은 당시 정지 odom `(0.052868, 0.000288, yaw 0.012499)`, 목표는 전방 0.05 m인 `(0.102865, 0.000913, yaw 0.012499)`이다. 종료 odom은 `(0.093638, 0.000971, yaw 0.018101)`, 이동 0.0404 m, 잔여 0.0096 m, 선속도 0, `motion_commands` 97이다. RMF와 UI는 `completed`다. footprint 0.18 m, vicinity 0.25 m에서도 `navigate`가 호출됐다. merge는 0.005 m / 0.01 m다.

바퀴가 도는 장면은 이 실행에서 직접 보지 못했다. 시험 뒤 gateway는 `PHYSICAL_DRY_RUN`이고 adapter는 멈췄다. 같은 버튼을 다시 눌러 두 번째 5cm는 실행하지 않았다.

## 2026-10-23 12:12 RMF task

Care-Move `POST /api/tasks`가 booking `e5144935-5382-4d8a-9664-117cced1bd52`를 받았다. RMF가 `physical_pinky` / `real_pinky1`에 배정했고, Fleet Adapter의 `navigate`가 gateway goal generation 1을 수락했다. 이 요청은 Care-Move가 gateway를 직접 호출한 것이 아니다.

측정 구간은 당시 정지 odom `(0.022013, 0.000093, 0.005603)`에서 전방 0.04 m인 `(0.062013, 0.000317, 0.005603)`이다. 그래프 merge는 waypoint 0.005 m, lane 0.01 m라 4 cm lane이 합쳐지지 않았다. 종료 odom은 `(0.052868, 0.000288, 0.012499)`, 이동 약 0.031 m, 목표 오차 약 0.009 m, 선속도 0, `motion_commands` 83이다. RMF status와 Care-Move 임무 이력은 `completed`다.

바퀴가 도는 장면은 이 실행에서 직접 보지 못했다. 시험 뒤 gateway는 `PHYSICAL_DRY_RUN`으로 되돌렸다. 취소 주행은 하지 않았다. Wi-Fi 주소가 `192.168.4.4`로 바뀐 뒤 기존 Cyclone 소켓이 `192.168.0.221`로 송신에 실패해, domain 77 schedule과 API를 loopback 설정으로 다시 띄웠다. Gazebo adapter는 Nav2 action이 ready가 아니라 다시 올리지 않았다.

