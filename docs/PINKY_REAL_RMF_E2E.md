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

Nav2 단일 goal과 현장 정지 승인이 생기기 전에는 이 게이트를 열지 않는다.
