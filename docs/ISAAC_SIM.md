# Isaac Sim과 가상 pinky1

2026-10-21 조사. Isaac Sim을 새로 설치하지 않았다. 이미 있는 빌드를 사용한다.

| 항목 | 값 |
|---|---|
| 버전 | `6.2.0-rc.14+release.52805.02189a84.gl` |
| 경로 | `/home/sw/isaacsim` |
| 선택 이유 | Ubuntu 24.04에서 이 빌드의 실행기가 Jazzy를 고르고, `isaacsim.ros2.core/jazzy`와 Cyclone DDS가 들어 있다. 다른 버전 API를 섞지 않기 위해 이 RC를 유지한다. |
| 문서 | 설치 트리 `exts/isaacsim.ros2.bridge/docs/Overview.md`. 공개 문서 입구는 <https://docs.isaacsim.omniverse.nvidia.com/latest/index.html> |
| GPU | RTX 4060 Laptop, 드라이버 580.178.04, VRAM 8188 MiB 중 사용 약 81 MiB |
| RAM | 전체 14 GiB, 조사 시점 가용 약 1.8 GiB |
| GUI | `DISPLAY=:1`, X11 |

가용 RAM이 8 GiB 미만이면 `sim/isaac/pinky1_bringup.py`는 `SimulationApp`을 만들기 전에 종료한다. 이번 조사에서는 그 조건 때문에 시뮬레이터를 시작하지 않았다. Gazebo로 대체하지 않았다.

## Python을 나누는 위치

| 용도 | 실행 |
|---|---|
| Care-Move 테스트 | `care-move-rmf-starter/.venv/bin/python` |
| `ros2db` | `source ~/venv/db_care/bin/activate`. Isaac이나 ROS 패키지를 여기에 넣지 않는다. |
| ROS 도구 | `/usr/bin/python3`와 `/opt/ros/jazzy` |
| map 도구 | `PYTHONNOUSERSITE=1`. 사용자 site의 NumPy 2.5.3이 `/usr/lib` Shapely 2.0.3을 깨므로, 시스템 NumPy 1.26.4만 쓰게 한다. |
| Isaac Sim | `/home/sw/isaacsim/python.sh`. Care-Move venv와 합치지 않는다. |

`scripts/run_building_map_generator.sh`는 격리된 map 도구 실행이다. `--help`까지 확인했다. 건물 YAML은 변환하지 않았다.

## ROS 도메인

가상 pinky1 전용 셸은 `sim/isaac/env.sh`이다.

```bash
source sim/isaac/env.sh
```

`ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. 도메인 0의 RMF API·schedule과 `~/.bashrc`의 jazzy 도메인 24, patrol 도메인 25에는 명령을 넣지 않는다. RMF 컨테이너의 도메인은 바꾸지 않았다.

속도 명령 타입은 Pinky Nav2 설정과 Isaac Sim 6 Carter 예제와 같이 `geometry_msgs/msg/Twist`이다. `TwistStamped`로 바꾸지 않는다.

## Pinky 모델

URDF는 `/home/sw/DevSpace/pinky/src/pinky_pro/pinky_description/urdf/robot.urdf.xacro`이다. USD는 없다.

- 단위는 URDF 미터. 바퀴 반지름 0.028 m. 바퀴 축은 `0 1 0`.
- joint 원점 y는 ±0.04055 m라 중심 간격은 0.0811 m.
- Gazebo diff-drive 플러그인은 `wheel_separation` 0.0961 m를 따로 적는다. Isaac 구동 계수를 확정하기 전에 둘을 맞춰야 한다.
- 기본 namespace 인자는 빈 문자열이라 joint 이름은 `l_wheel_joint`, `r_wheel_joint`이다.
- LiDAR 프레임은 `rplidar_link`. 질량은 링크별 URDF 값이며 차대 0.8 kg이다.
- 화면 표시나 파일 존재만으로 구동 완료가 아니다.

## 아직 하지 않은 실행

Nav2 goal, `/clock` 진행, odom, scan, TF, 속도 명령은 보내지 않았다. `simulation_confirmed`, `allow_motion`, `graph_alignment`은 그대로 false/placeholder다. placeholder 그래프 좌표는 목표로 쓰지 않는다.

메모리가 충분해진 뒤의 순서:

```bash
source sim/isaac/env.sh
python3 -m sim.isaac.preflight
python3 -m sim.isaac.pinky1_bringup
```

preflight가 막히지 않은 다음에만 `/home/sw/isaacsim/python.sh sim/isaac/pinky1_bringup.py`로 SimulationApp을 붙인다. 그 전에 도메인 42의 `ros2 node list`가 비어 있어야 한다.
