# Gazebo Harmonic 전환

2026-10-21, swPC에서 확인. Isaac Sim 코드는 그대로 두었고, 가상 pinky1의 시뮬레이터 계층만 Gazebo Harmonic으로 추가했다. Pinky 원본 소스는 수정하지 않았다. Git commit과 push는 하지 않았다.

## 유지한 것

- Care-Move Dashboard, FastAPI, Mock 모드
- RMF-Web API와 JWT. 도메인 0의 `care-rmf-api` 컨테이너와 `rmf_traffic_schedule`은 종료하지 않았다
- Fleet `care_move`. `allow_motion: false`, `simulation_confirmed: false`, `graph_alignment: placeholder`
- 로봇 3대 확장 구조. 이번 실행은 pinky1 한 대
- 승강기 Gateway 계약. 승강기 명령은 보내지 않았다
- `sim/isaac/`의 실행 게이트와 bringup

## 추가한 파일

| 파일 | 역할 |
|---|---|
| `sim/gazebo/env.sh` | 도메인 42, Cyclone DDS, Pinky overlay, Gazebo 플러그인 경로 |
| `sim/gazebo/gates.py` | 도메인 0, 24, 25와 도메인 42 이외의 실행을 거부 |
| `sim/gazebo/preflight.py` | Gazebo를 시작하지 않고 게이트만 출력 |
| `sim/gazebo/pinky1.launch.py` | 서버 전용 Gazebo, lidar가 붙은 URDF 스폰, bridge |
| `sim/gazebo/pinky1_bridge.yaml` | `/clock`, `/tf`, `/pinky1/cmd_vel`, `/pinky1/odom`, `/pinky1/scan` |
| `sim/gazebo/description_filter.py` | namespaced URDF에서 `gpu_lidar` 링크 참조를 고치고 camera/IMU 센서를 제거 |
| `sim/gazebo/room_geometry.py` | 벽·장애물과 occupancy map이 같은 좌표를 쓰게 함 |
| `sim/gazebo/worlds/pinky1_empty.sdf` | 바닥, 벽, 장애물, `gz-sim-sensors-system` |
| `sim/gazebo/nav2.launch.py` | namespace `pinky1` Nav2. Gazebo는 시작하지 않음 |
| `sim/gazebo/tf_relay.py` | `/tf`를 `/pinky1/tf`로 복사. 프레임을 만들지 않음 |
| `sim/nav2/pinky1_gazebo.yaml` | 프레임 `pinky1/base_footprint`, `pinky1/odom` |
| `sim/nav2/map/pinky1_room.yaml` | 8 m 방, 해상도 0.05 m, origin `[-4, -4]` |
| `sim/gazebo/motion_check.py` | 가상 pinky1에 제한 속도 전진 후 정지 |
| `sim/gazebo/scan_check.py` | `/pinky1/scan` 측정값 검사 |
| `sim/gazebo/nav_goal.py` | 조건이 맞을 때만 `NavigateToPose` 1회 |
| `tests/test_gazebo_contract.py` | bridge, lidar 필터, 맵 좌표, fleet 잠금 |

## 통신 계약

Pinky `pinky_gz.urdf.xacro`의 diff-drive 플러그인을 그대로 쓴다. namespace는 `pinky1`이다.

| 항목 | 값 |
|---|---|
| `cmd_vel` 타입 | `geometry_msgs/msg/Twist` |
| 명령 토픽 | `/pinky1/cmd_vel` |
| 오도메트리 | `/pinky1/odom`, `nav_msgs/msg/Odometry` |
| 시계 | `/clock`, `rosgraph_msgs/msg/Clock` |
| TF | `/tf`와, Nav2용으로 중계한 `/pinky1/tf`. `map` → `pinky1/odom` → `pinky1/base_footprint` → `pinky1/rplidar_link` |
| LiDAR | Pinky `gpu_lidar`, Gazebo topic `/pinky1/scan`, ROS `/pinky1/scan` |
| Nav2 액션 | `/pinky1/navigate_to_pose` |
| 바퀴 반지름 | 0.028 m |
| 플러그인 `wheel_separation` | 0.0961 m. URDF joint 간격 0.0811 m와 다르며, 이번 작업에서 맞추지 않았다 |
| ROS 도메인 | 42. RMF는 도메인 0 |

네임스페이스가 없는 `/cmd_vel`, `/odom`은 만들지 않았다. Fleet 설정의 `ros_namespace: pinky1`과 로봇 3대 구분에 맞춘 것이다. Pinky 기본 launch의 빈 namespace와는 토픽 이름이 다르다.

`sim/nav2/pinky1_params.yaml`의 `base_footprint`는 예전 스텁이다. Gazebo Nav2는 `sim/nav2/pinky1_gazebo.yaml`의 `pinky1/base_footprint`를 쓴다. Pinky 원본 `nav2_params.yaml`은 바꾸지 않았다.

LiDAR는 Pinky URDF의 `gpu_lidar`이다. namespace가 붙으면 `<gazebo reference>`가 링크 이름과 어긋나므로, 스폰 직전에 `description_filter.py`가 참조를 `rplidar_link`로 고친다. 월드는 `gz-sim-sensors-system`과 `ogre2`, 실행 인자는 `--headless-rendering`이다. 카메라와 IMU 센서 블록은 스폰 URDF에서만 빼서 렌더 부하를 lidar에 둔다.

## 실행

```bash
cd /home/sw/DevSpace/care-move-rmf-starter
source sim/gazebo/env.sh
python3 -m sim.gazebo.preflight
ros2 launch /home/sw/DevSpace/care-move-rmf-starter/sim/gazebo/pinky1.launch.py
```

다른 셸에서 스캔, Nav2, 목적지를 순서대로 확인한다.

```bash
source sim/gazebo/env.sh
python3 sim/gazebo/scan_check.py
ros2 launch /home/sw/DevSpace/care-move-rmf-starter/sim/gazebo/nav2.launch.py
python3 sim/gazebo/nav_goal.py
```

`nav_goal.py`는 액션 서버, LaserScan, `map` → `pinky1/base_footprint`가 있을 때만 goal을 보낸다. 기본 목적지는 맵 좌표 `(1.2, 0.0)`이다. GUI는 띄우지 않는다. 자세한 측정은 `docs/GAZEBO_NAV2_VALIDATION.md`에 있다.

종료는 launch 프로세스에 SIGINT를 보낸다. `gz sim` 서버가 ruby 래퍼보다 오래 남으면 그 PID만 따로 종료한다. RMF PID는 종료 대상이 아니다.
