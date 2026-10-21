# Gazebo Harmonic 시험 보고

2026-10-21, swPC (`hostname` = `swPC`, 사용자 `sw`). 작업 디렉터리는 `/home/sw/DevSpace/care-move-rmf-starter`, 브랜치 `master`. 시험 전 `git status --short`는 비어 있었다. 시험 후 추적 파일의 diff는 없고, 추가된 것은 `sim/gazebo/`와 `tests/test_gazebo_contract.py`, 이 문서들이다. `/home/sw/DevSpace/pinky`의 git status도 비어 있다.

실제 로봇과 승강기에는 명령을 보내지 않았다. `allow_motion`과 `simulation_confirmed`는 false로 두었다.

## 환경

| 확인 | 결과 | 근거 |
|---|---|---|
| 실행 머신 | PASS | `hostname` → `swPC` |
| 프로젝트 접근 | PASS | `pwd`가 저장소 루트, 소스 읽기 가능 |
| Pinky ROS 2 | PASS | `/home/sw/DevSpace/pinky/src/pinky_pro/pinky_gz_sim`. overlay 후 `pinky_gz_sim` prefix 확인 |
| Gazebo Harmonic | PASS | `gz sim --version` → Gazebo Sim 8.15.0. 패키지 `libgz-sim8` 8.15.0 |
| ROS 2 | PASS | Jazzy. `ros_gz`, `nav2_*`, `rmf_*`가 설치되어 있음 |
| 기존 Git 변경 | PASS | 시험 시작 시 작업 트리가 비어 있어 보존할 수정은 없었다 |

시험 중 가용 RAM은 약 1.8 GiB였다. Isaac Sim bringup의 8 GiB 게이트는 여전히 이 메모리에서 실행을 거부한다. Gazebo 서버는 그 게이트와 별도로 기동했다.

## 시뮬레이터

`source sim/gazebo/env.sh` 뒤 preflight:

```json
{"gz_sim_installed": true, "ros_domain_id": 42, "rmw": "rmw_cyclonedds_cpp", "block_reason": null, "simulation_confirmed": false}
```

`ros2 launch sim/gazebo/pinky1.launch.py` 로그:

- Gazebo Sim Server v8.15.0
- 월드 `/home/sw/DevSpace/care-move-rmf-starter/sim/gazebo/worlds/pinky1_empty.sdf`
- `spawn_pinky1`: `Entity creation successful.`
- bridge가 아래 토픽을 생성

```text
/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)
/tf (gz.msgs.Pose_V) -> /tf (tf2_msgs/msg/TFMessage)
/pinky1/cmd_vel (geometry_msgs/msg/Twist) -> /pinky1/cmd_vel (gz.msgs.Twist)
/pinky1/odom (gz.msgs.Odometry) -> /pinky1/odom (nav_msgs/msg/Odometry)
```

도메인 42의 `ros2 topic list`:

```text
/clock
/pinky1/cmd_vel
/pinky1/joint_states
/pinky1/odom
/pinky1/robot_description
/pinky1/scan
/tf
/tf_static
```

같은 시각 도메인 0의 노드는 `/rmf_api_server`와 `/rmf_traffic_schedule_node_a2c8603e_42e9_403a_9b84_b4b5b7a2da20`뿐이었다. Gazebo 노드는 도메인 0에 보이지 않았다.

## 토픽과 이동

| 항목 | 결과 | 근거 |
|---|---|---|
| `/clock` | PASS | publisher `pinky1_bridge`. 샘플 `sec: 51`. 이동 시험 동안 sim time 69.765 → 75.675 |
| `/tf` | PASS | `pinky1/odom` → `pinky1/base_footprint`. 이동 시험 중 TF 메시지 531건 |
| `/pinky1/cmd_vel` | PASS | 타입 `geometry_msgs/msg/Twist`. 구독자는 bridge 1개. 네임스페이스 없는 `/cmd_vel`은 없었다 |
| `/pinky1/odom` | PASS | 타입 `nav_msgs/msg/Odometry`. 스폰 직후 x ≈ 0, vx ≈ 0. 네임스페이스 없는 `/odom`은 없었다 |
| `/pinky1/scan` | 이후 PASS | 첫 실행은 센서 시스템이 없어 FAIL. 같은 날 센서 시스템을 넣은 뒤 640개 range가 10 Hz로 수신됨. `docs/GAZEBO_NAV2_VALIDATION.md` |
| 전진 | PASS | 0.12 m/s를 2.0초 발행. dx = 0.284 m, dy ≈ 0, 샘플 시점 vx = 0.12 |
| 정지 | PASS | 0 Twist 이후 vx = 0.0, 유지 구간 이동 0.0 m. 정지 위치 x = 0.2886 m |

전진 거리가 0.24 m보다 큰 이유는 diff-drive가 마지막 비영 속도를 0 Twist가 올 때까지 유지하기 때문이다. 샘플은 발행이 끝난 뒤 0.4초 시점이었고, 0 명령을 보낸 뒤에 속도가 0이 되었다.

`sim/gazebo/motion_check.py` 결과:

```json
{"clock_seen": true, "odom_seen": true, "tf_seen": true, "forward_sent": true, "stop_sent": true, "passed": true, "dx": 0.28409999999340124, "dy": 2.1578479361336824e-13, "moved_vx": 0.12000000000000455, "stop_vx": 0.0, "stop_drift_m": 0.0, "tf_messages": 531}
```

이 명령은 도메인 42의 가상 pinky1에게만 갔다. Nav2 goal, fleet task, 승강기 호출은 없다.

## Nav2와 Fleet

| 항목 | 결과 | 근거 |
|---|---|---|
| Nav2 액션 | 이후 PASS | 첫 확인 때는 액션이 없었다. 이후 `/pinky1/navigate_to_pose`가 생기고 goal 2회가 status 4로 끝났다 |
| 프레임 정합 | 이후 PASS | Gazebo 실행은 `pinky1/base_footprint`를 쓴다. 스텁 `pinky1_params.yaml`의 `base_footprint`는 실행에 쓰지 않는다 |
| 속도 타입 | PASS | `enable_stamped_cmd_vel: false`. Nav2가 `/pinky1/cmd_vel` Twist로 로봇을 움직였다 |
| Fleet Adapter | 부분 | 도메인 0에서 `scripts/probe_fleet_adapter.py`. `started: true`, `robots_added: 0`, `server_uri_sent: false`, fleet `care_move` |
| 모션 잠금 | PASS | `allow_motion: false`, `simulation_confirmed: false`, `graph_alignment: placeholder` 유지 |
| RMF 프로세스 | PASS | 시험 후 `rmf_traffic_schedule` PID 1359500과 `care-rmf-api` 컨테이너가 그대로 있었다 |

`simulation_confirmed`를 true로 바꾸지 않았다. cmd_vel 전진은 Nav2 주행이나 실측 그래프 확인이 아니다.

## 회귀

ROS PYTHONPATH를 뺀 상태에서:

```text
.venv/bin/python -m pytest -q
58 passed, 1 warning in 0.45s
```

56은 lidar 필터와 방 좌표 테스트를 넣기 전 결과다. 추가된 테스트 2개가 58에 포함된다. 기존 테스트는 지우지 않았다.

경고는 FastAPI TestClient의 Starlette deprecation 한 건이다. Isaac 게이트 테스트와 fleet 잠금 테스트는 통과했다.

## 종료

launch PID 1711369에 SIGINT를 보냈다. bridge와 robot_state_publisher는 정상 종료했다. ruby로 감싼 `gz sim`은 SIGTERM 이후에도 서버 PID 1711402가 남아 있어, 그 PID에만 SIGINT를 보냈다. 이후 `gz sim` 프로세스는 없고 도메인 42의 `ros2 node list`는 비어 있었다. RMF는 종료하지 않았다.
