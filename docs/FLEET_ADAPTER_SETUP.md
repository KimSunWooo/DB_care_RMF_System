# care_move Fleet Adapter

설정은 `config/rmf/care_move_fleet.yaml`이다.

- Fleet 이름 `care_move`
- `rmf_fleet.robots`에는 `pinky1`만 있다. charger는 `N_DOCK1`
- `pinky2`, `pinky3`는 `care_move.robots`에서 `enabled: false`
- Nav2 후보 액션은 `/pinky1/navigate_to_pose`
- 위치 후보 토픽은 `/pinky1/amcl_pose`
- API 상태 채널은 실행 중인 서버의 `ws://127.0.0.1:8000/_internal`이다. 이 경로는 소스와 WebSocket 101 응답으로 확인했다. 인증 헤더는 없다
- 목표 전달은 `fleet_adapter/safety.py`가 막는다. placeholder 그래프, `allow_motion: false`, 시뮬레이션 미확인이면 goal을 만들지 않는다

코드:

- `fleet_adapter/config.py` 설정 로드
- `fleet_adapter/nav2_link.py` 로봇별 액션 상태. ROS 메시지는 보내지 않는다
- `fleet_adapter/safety.py` 주행 차단
- `scripts/probe_fleet_adapter.py` 스케줄에만 붙고 로봇을 추가하지 않는다. `server_uri`도 비운다

프로브는 venv가 아니라 ROS Python으로 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
python3 scripts/probe_fleet_adapter.py
```

2026-10-21 프로브: `Adapter.make` 성공, Easy Full Control fleet `care_move` 구성, `robots_added=0`. 프로세스는 `stop` 후 종료했다. 이후 `GET /fleets`는 여전히 빈 목록이다. `/pinky1/navigate_to_pose`는 없다.

신선 위치가 없고 그래프가 실측이 아니므로 로봇을 `add_robot` 하지 않았다. 등록 성공으로 보지 않는다.
