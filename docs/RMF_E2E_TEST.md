# 단일 로봇 E2E

목표 흐름은 대시보드에서 pinky1, `N_DOCK1`에서 `N_ROOM102`로 작업을 보내고 Nav2와 Isaac Sim까지 완료를 보는 것이다.

아래가 확인되기 전에는 `POST /tasks/dispatch_task`, `POST /tasks/robot_task`, Nav2 goal을 보내지 않는다.

- Isaac Sim 또는 동등 시뮬이 실제 하드웨어와 분리되어 실행 중
- `/pinky1/navigate_to_pose` 액션 서버
- 신선한 `/pinky1/amcl_pose` 또는 동등한 위치
- `/clock`이 진행하거나, API와 어댑터의 `use_sim_time`이 같음
- 실측 navigation graph. `graph_alignment: surveyed`, `allow_motion: true`, `simulation_confirmed: true`
- `GET /fleets`에 `care_move`와 `pinky1`의 현재 위치

2026-10-21에는 이 조건이 하나도 충족되지 않았다. 작업 접수와 작업 완료는 모두 미검증이다. `GET /tasks`는 200 빈 목록이다.

조회만 다시 할 때:

```bash
.venv/bin/python scripts/rmf_preflight.py
```
