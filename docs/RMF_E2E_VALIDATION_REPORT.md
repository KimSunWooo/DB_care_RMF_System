# Gazebo RMF E2E 검증 보고

일시: 2026-10-22, swPC. 대상은 Gazebo Pinky1 한 대와 시뮬레이션 RMF다. 운영 fleet `care_move`와 도메인 0에는 task를 보내지 않았다. Git commit과 push는 하지 않았다.

## 결과

| 검증 항목 | 결과 | 증거 |
|---|---|---|
| RMF Core 실행 | PASS | 도메인 77에서 `rmf_traffic_schedule`(`use_sim_time:=false`)와 `building_map_server`가 동작. API 컨테이너 안에 `/rmf_api_server`. 도메인 0 노드 목록은 비어 있음 |
| RMF API 인증 | PASS | `GET http://127.0.0.1:8000/openapi.json` 200. Care-Move `/api/rmf/status` 인증 성공, fleet `gazebo_pinky1`. 토큰은 기록하지 않음 |
| RMF Navigation Graph | PASS | `GZ_SPAWN (0,0)`, `GZ_EAST (1.2,0)`, identity 변환. building map이 두 waypoint를 반환. 1F·7F 파일은 수정하지 않음 |
| Fleet 등록 | PASS | adapter `started: true`, fleet `gazebo_pinky1`. API fleet 조회에 이 이름만 존재 |
| Pinky1 등록 | PASS | `robots_added: 1`, `server_uri_sent: true`. task가 `assigned_to.name = pinky1`으로 배정되고 pose update가 이어짐 |
| Domain 0↔42 연결 | 범위 제외 | 운영 도메인 0은 비어 있고 사용하지 않음. 실제 연결은 도메인 77 adapter ↔ TCP `127.0.0.1:17442` ↔ 도메인 42 Nav2 |
| Task Dispatch | PASS | Care-Move `rmf-request-a165059a` → RMF `993eb4fc-af9d-4fc8-9f59-1f6491792c75`, 장소 `GZ_EAST`, 초기 상태 `queued` |
| Nav2 Goal Accepted | PASS | 같은 task의 gateway generation 3, phase `accepted`, goal id `[175 104 86 28 147 68 79 28 178 213 229 120 193 173 2 88]` |
| Gazebo Pose 변화 | PASS | 출발 x≈0.183. 진행 중 x=0.340, yaw=1.417. 도착 x=1.017, y=-0.019 |
| Nav2 Goal SUCCEEDED | PASS | generation 3 `navigation_finished`, pose `[1.0168, -0.0185, -2.765]`. 목표 (1.2, 0)까지 약 0.184 m, tolerance 0.2 m |
| RMF Task COMPLETED | PASS | `/api/state`의 `rmf_status: completed`, `completed: true`, 배정 `gazebo_pinky1/pinky1` |
| Care-Move API 반영 | PASS | 로컬 id `rmf-request-a165059a`가 RMF id `993eb4fc-af9d-4fc8-9f59-1f6491792c75`와 연결되고 상태가 `completed` |
| Dashboard 완료 표시 | PASS | `http://127.0.0.1:8011/` 임무 이력: `pinky1 → GZ_EAST`, `completed · RMF · 993eb4fc-af9d-4fc8-9f59-1f6491792c75 · completed`. 목적지 선택에 `RMF graph · GZ_EAST`가 있음. 배너 "주행 검증: 아님"은 운영 플래그라 완료 행과 별개 |
| Task Cancel | PASS | RMF `119710a4-9067-413e-a93c-c9b37e946d3d`. x=1.026에서 출발해 x=0.872에서 goal 수락 후 취소. Nav2 phase `canceled`, `result_confirmed: true`, 정지 pose x=0.859. RMF `canceled`, `completed: false`. `GZ_SPAWN`에는 도달하지 않음 |
| Hardware Isolation | PASS | gateway `hardware_port: false`, `publishes_cmd_vel: false`. `/dev/ttyAMA0`, `/dev/ttyAMA4` 없음. 이번 프로세스의 도메인은 42와 77뿐. 도메인 53·90에 이동 명령을 보내지 않음. 도메인 53 `rviz2` PID 44166은 유지. 모터 드라이버 검증은 하지 않음 |
| 전체 테스트 | PASS | `env -u PYTHONPATH -u AMENT_PREFIX_PATH PYTHONNOUSERSITE=1 .venv/bin/python -m pytest -q` → 94 passed. 기존 93에 `GoalTracker` stale result 테스트 1건이 추가됨. 안전 테스트는 삭제하거나 완화하지 않음 |

## 이번 검증에서 실제로 이어진 경로

```text
Care-Move POST /api/tasks
  → RMF-Web POST /tasks/robot_task
  → 도메인 77 schedule / fleet adapter
  → TCP gateway
  → /pinky1/navigate_to_pose
  → Gazebo pinky1 이동
  → Nav2 SUCCEEDED
  → execution.finished()
  → RMF completed
  → /api/state 와 대시보드 임무 이력
```

직접 RMF에 넣어 Nav2 성공과 RMF completed가 같이 확인된 앞선 주행도 있다. task `f05c1df0-4419-409a-88d2-14b8a13dd8f8`, 목적지 `GZ_SPAWN`, 최종 pose x=0.183, y=0.002, gateway generation 2 `navigation_finished`.

## 성공으로 세지 않은 시도

`fce2f4cf`는 `destination.position`을 함수로 호출해 `TypeError`가 났고 Nav2 goal이 나가지 않았다. property로 읽도록 고친 뒤 그 task는 `standby`로 남았다.

`d9ff1b53`는 로봇이 x≈0.76까지 움직인 뒤, adapter가 update에 activity를 넘기지 않아 RMF가 command handle unresponsive로 재계획했다. merge 거리 때문에 그 위치를 `GZ_EAST` 도착으로 처리해 RMF status는 completed였지만 gateway phase는 `canceled`였다. Nav2 SUCCEEDED가 아니므로 실패다. activity를 저장하고 merge를 waypoint 0.3 m, lane 0.5 m로 낮춘 다음 프로세스만 다시 띄웠다.

## 추가로 확인한 실패 경로

- `POST /api/tasks` 장소 `NOT_EXIST`는 HTTP 400 `Unknown waypoint(s): NOT_EXIST`.
- gateway `go` 장소 `N_ROOM102`는 `waypoint is not on the gazebo test graph`이고 goal generation은 그대로다. pose도 변하지 않았다.
- 소유한 gateway만 종료했을 때 포트 17442는 거절되고 adapter 로그에 `gateway_lost`가 남았다. 같은 설정으로 다시 띄운 뒤 pose는 정지 위치 `[0.8587, -0.0413, -3.105]`, goal phase는 `idle`이었다. 끊긴 동안 새 task는 넣지 않았다.

스케줄 미러 timeout 로그는 완료 후에도 남는다. 완료 판정을 바꾸지는 않았다.

## 아직 운영 경로가 아닌 것

도메인 0의 운영 RMF와 fleet `care_move`는 연결하지 않았다. `allow_motion`, `simulation_confirmed`, `graph_alignment: placeholder`는 그대로다. 실제 Pinky의 모터, encoder, `/cmd_vel`, Nav2 goal은 이번 범위 밖이다. 대시보드의 1F·7F 그림은 개념 지도이고 Gazebo 방의 SLAM 맵이 아니다.

## 남겨 둔 프로세스

검증 후 아래는 종료하지 않았다. 도메인 53 `rviz2` PID 44166도 종료하지 않았다.

- Gazebo, Nav2 (도메인 42)
- schedule, building map, `care-move-gazebo-rmf-api` (도메인 77)
- gateway PID 96278, fleet adapter, Care-Move `127.0.0.1:8011`

종료 순서는 `docs/RMF_E2E_RUNBOOK.md`의 마지막 절이다.
