# 통합 상태 (2026-10-21)

| 항목 | 결과 | 검증 근거 |
|---|---|---|
| RMF API JWT 인증 | PASS | `local_jwt`로 `GET /fleets` 200, `GET /tasks` 200. 목록 길이는 0 |
| Open-RMF 실행 | 부분 | API 서버와 traffic schedule 노드는 있다. `/clock` 발행자 0, `GET /time`은 0. 작업 dispatcher 별도 프로세스는 없다 |
| Navigation Graph | 부분 | placeholder 2개 waypoint를 `parse_graph`가 읽었다. 실측 맵과 `/building_map`은 없다(404) |
| Fleet Adapter | 부분 | 프로브가 schedule에 연결되고 로봇 0대를 추가한 뒤 종료. API fleet 목록은 비어 있음 |
| Pinky1 등록 | FAIL | `GET /fleets` 길이 0. 위치 소스가 없어 `add_robot` 하지 않음 |
| Nav2 Action 연결 | FAIL | `ros2 action list` 비어 있음. Isaac/Nav2/Gazebo 프로세스 없음 |
| Isaac Sim 주행 | 미검증 | 시뮬레이터 없음. goal 미전송 |
| RMF Task 완료 | 미검증 | 작업 POST 없음. `GET /tasks` 길이 0 |
| Isaac Sim 실행 | 미검증 | 6.2.0-rc.14가 `/home/sw/isaacsim`에 있다. 가용 RAM이 8 GiB 미만이라 SimulationApp을 시작하지 않음 |
| 가상 pinky1 Nav2 도착 | 미검증 | Nav2 액션 서버 없음. placeholder 좌표로 goal을 보내지 않음 |

pytest: `.venv/bin/python -m pytest -q` → 50 passed.

이동 명령, 작업 생성, 승강기 제어는 보내지 않았다.
