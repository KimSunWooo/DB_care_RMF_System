# Gazebo Nav2 검증

2026-10-21, swPC, ROS 도메인 42, `rmw_cyclonedds_cpp`. Gazebo Harmonic 8.15.0. Pinky 원본과 Isaac Sim 코드, Fleet YAML, FastAPI는 수정하지 않았다. RMF task는 보내지 않았다.

## A. LiDAR

Pinky `pinky_gz.urdf.xacro`의 센서는 `gpu_lidar`이다. 부모 링크 이름은 `rplidar_link`이고, 토픽은 namespace `pinky1/` 기준으로 `pinky1/scan`, 주기는 10 Hz, 샘플 640개, 거리 0.05–12 m이다. `gz_frame_id`는 `pinky1/rplidar_link`이다.

처음에는 월드에 `gz-sim-sensors-system`이 없고, namespaced URDF의 `<gazebo reference="pinky1/rplidar_link">`가 실제 링크 이름 `rplidar_link`와 달라 측정이 없었다. 수정은 두 곳이다.

- `sim/gazebo/worlds/pinky1_empty.sdf`에 `gz-sim-sensors-system`과 `render_engine` `ogre2`를 넣었다. launch 인자는 `-r -s --headless-rendering`이다.
- `description_filter.py`가 스폰 URDF에서 lidar 참조를 `rplidar_link`로 고친다. 카메라와 IMU 센서 블록은 이 파일에서만 제거한다. Pinky xacro는 그대로다.

SDF로 변환하면 빈 링크가 `base_footprint`로 합쳐지지만, 센서 pose는 `(-0.017, 0, 0.125, yaw π)`로 남는다. `tf2_echo pinky1/base_footprint pinky1/rplidar_link`도 같은 값이다.

`scan_check.py` 결과:

```json
{"messages": 5, "frame_id": "pinky1/rplidar_link", "range_count": 640, "finite_count": 640, "nan_count": 0, "inf_count": 0, "finite_min_m": 2.0035359859466553, "finite_max_m": 4.97350549697876, "mean_period_sec": 0.09999999999999964, "passed": true}
```

최소 거리 약 2.0 m는 장애물 `(2.0, 1.2)` 방향과 맞다. 최대 약 5.0 m는 방 벽 거리 안에 있다. bridge QoS는 `SENSOR_DATA`(best effort)이다. Gazebo 토픽은 `/pinky1/scan`과 `/pinky1/scan/points`이다.

`gz_frame_id`는 SDF 표준 요소가 아니라는 경고가 한 번 난다. ROS header frame은 `pinky1/rplidar_link`로 들어온다.

## B. TF

실행 중인 트리:

```text
map
 └── pinky1/odom
      └── pinky1/base_footprint
           └── pinky1/base_link
                └── pinky1/rplidar_mount
                     └── pinky1/rplidar_link
```

`map` → `pinky1/odom`은 AMCL이 `/pinky1/tf`에 발행한다. `pinky1/odom` → `pinky1/base_footprint`는 Gazebo diff-drive가 `/tf`에 발행하고 `tf_relay.py`가 `/pinky1/tf`로 복사한다. 나머지 고정 변환은 `robot_state_publisher`의 `/tf_static`이다. 릴레이는 없는 프레임을 만들지 않는다.

`sim/nav2/pinky1_gazebo.yaml`의 `robot_base_frame`은 `pinky1/base_footprint`, `odom_frame_id`는 `pinky1/odom`, `global_frame`은 `map`이다. 예전 스텁 `pinky1_params.yaml`의 `base_footprint`는 이 실행에 쓰지 않는다.

`map` → `pinky1/base_footprint`를 도메인 42에서 확인했을 때 원점 근처의 단위 회전이었다. AMCL 초기 pose는 `(0, 0, 0)`이다.

## C. Nav2

월드와 맵은 `room_geometry.py`의 같은 박스에서 만든다. 방은 8 m, origin `[-4, -4]`, 해상도 0.05 m, 벽 네 면과 `(2.0, 1.2)` 장애물이다. `map_server`가 받은 맵은 160×160, origin x/y `-4.0`이다.

Lifecycle은 모두 `active [3]`이었다.

```text
/pinky1/map_server
/pinky1/amcl
/pinky1/controller_server
/pinky1/planner_server
/pinky1/behavior_server
/pinky1/bt_navigator
```

액션 `/pinky1/navigate_to_pose`의 타입은 `nav2_msgs/action/NavigateToPose`이다. 전역 costmap은 160×160이고, 샘플 시점에 lethal 2425, inflated 7208, unknown 0이었다.

목적지는 막힌 칸이 아니면 보내지 않는다. 결과 status 4는 `SUCCEEDED`이다. 목표 허용 오차는 파라미터 `xy_goal_tolerance: 0.2`이다.

| 목표 | 시작 (map) | 종료 (map) | 이동 | 오차 | 정지 | 결과 |
|---|---|---|---|---|---|---|
| `(1.2, 0.0)` | `(0, 0)` | `(1.006, 0.0016)` | 1.006 m | 0.194 m | 직후 샘플 vx 0.117, 이후 재측정 vx 0.0, x 1.008 | controller `Reached the goal!`, bt `Goal succeeded` |
| `(0.0, 0.0)` | `(0.997, -0.0006)` | `(0.196, 0.0041)` | 0.801 m | 0.196 m | vx 0.0, wz 0.0 | `nav_goal.py` `passed: true`, feedback 3095건, 마지막 남은 거리 0.200 m |

첫 목표의 스크립트 종료 코드는 정지 속도를 결과 직후에 읽어 실패로 나왔다. 같은 위치에서 다시 읽은 선속도는 0이다. 복귀 목표는 2.5초 대기 뒤의 속도까지 스크립트가 통과로 판정했다.

## D. 회귀와 Git

```text
.venv/bin/python -m pytest -q
58 passed, 1 warning in 0.45s
```

경고는 FastAPI TestClient의 Starlette deprecation이다. 새 단위 테스트는 lidar URDF 필터, 방 좌표, bridge의 `SENSOR_DATA`이다. Gazebo 통합 시험은 pytest 밖에 있다.

시험 종료 후 도메인 0 노드는 `/rmf_api_server`와 `/rmf_traffic_schedule_node_a2c8603e_42e9_403a_9b84_b4b5b7a2da20`이었다. `rmf_traffic_schedule` PID 1359500과 `care-rmf-api`는 남아 있다. `allow_motion: false`, `simulation_confirmed: false`, `graph_alignment: placeholder`는 그대로다.

수정·추가된 경로는 `sim/gazebo/`, `sim/nav2/pinky1_gazebo.yaml`, `sim/nav2/map/`, `sim/nav2/pinky1_params.yaml`의 안내 주석, `tests/test_gazebo_contract.py`, 이 문서들이다. `git status`에 커밋된 변경은 없다. Pinky 디렉터리는 git 저장소가 아니고, 이번 작업에서 그 소스를 고치지 않았다. `sim/isaac/`과 `app/`은 변경 목록에 없다.

## E. Fleet Adapter를 붙이기 전에

Nav2 목적지 이동은 도메인 42에서만 확인했다. Fleet Adapter와 RMF task 연동은 하지 않았다.

| 항목 | 현재 값 |
|---|---|
| Action | `/pinky1/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) |
| 도메인 | Gazebo/Nav2는 42. RMF API와 schedule은 0 |
| RMW | `rmw_cyclonedds_cpp` |
| Namespace | `pinky1` |
| 명령 | `/pinky1/cmd_vel`, `geometry_msgs/msg/Twist` |
| 위치 | `map` 기준 `pinky1/base_footprint`. 오도메트리 프레임 `pinky1/odom` |
| 취소 | 같은 액션의 cancel. 이번 시험에서는 완료까지 가서 cancel을 호출하지 않았다 |
| 그래프 | `care_move_placeholder.nav.yaml`은 이 8 m 방과 좌표가 맞춰져 있지 않다 |

다음에 필요하지만 아직 하지 않은 일:

- placeholder 그래프를 이 맵에 실측으로 맞추기 전에는 `allow_motion`을 켜지 않는다.
- Fleet Adapter를 도메인 42로 올릴지, RMF를 이 시뮬레이터 시계에 연결할지를 따로 정한다. 도메인 0 프로세스를 이 시험 때문에 재시작하지 않는다.
- 도착 오차는 약 0.19 m이다. 더 촘촘한 정지가 필요하면 `xy_goal_tolerance`를 낮춘 뒤 다시 측정한다.
- 카메라와 IMU는 스폰 설명에서 빼 두었다. 그 센서 검증은 별도다.
