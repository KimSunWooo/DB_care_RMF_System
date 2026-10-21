# Fleet Adapter와 실제 Pinky

새 Fleet Adapter 프로세스를 만들지 않았다. 기존 `fleet_adapter/nav2_link.py`가 goal 장부와 액션 경로를 맡고, `fleet_adapter/safety.py`가 그래프·`allow_motion`·`simulation_confirmed`·액션·pose를 막는다. 이번 추가는 그 앞뒤의 프로필과 하드웨어 차단이다.

## Robot ID

| 로봇 | namespace | Fleet `enabled` | rmf_fleet.robots | 하드웨어 도메인 |
|---|---|---|---|---|
| pinky1 | `pinky1` | true | 있음. charger `N_DOCK1` | 비어 있음 |
| pinky2 | `pinky2` | false | 없음 | 비어 있음 |
| pinky3 | `pinky3` | false | 없음 | 비어 있음 |

액션 경로는 `/{namespace}/navigate_to_pose`다. `Nav2Link("pinky1", "pinky1")`의 `action_path`는 `/pinky1/navigate_to_pose`다. pinky1 한 대의 Gazebo 성공을 3대 등록으로 세지 않는다. 현재 설정에서 RMF에 올라갈 수 있는 ID는 pinky1뿐이고, 그것도 `allow_motion: false`다.

## Goal 변환

`robot_profile/goals.py`의 `fleet_goal_command`는 waypoint 이름을 액션 경로와 `frame_id: map`으로만 넘긴다. `graph_alignment`가 `surveyed`가 아니면 좌표를 만들지 않고 `GoalRejected`다. 반환 dict에는 x, y가 없다.

실제 전송은 `goal_block_reason`이 먼저다. 기본 fleet 값(`placeholder`, `allow_motion` false, `simulation_confirmed` false)에서는 그래프 이유로 막힌다. fleet 조건을 모두 열어도 하드웨어 프로필은 별도 이유로 막힌다. Gazebo 프로필은 하드웨어 차단을 타지 않고, 기존 fleet 잠금만 탄다. 그 잠금은 아직 열려 있지 않다.

연결이 끊기면 `mark_disconnected`가 액션을 내리고, 진행 중 goal의 phase를 `failed`로 둔다. pose가 없거나 오래되면 기존 gate가 goal을 거절한다.

배터리는 하드웨어에서 `std_msgs/Float32` 두 토픽이다. RMF fleet YAML의 24 V, 20 Ah는 그 토픽과 연결하지 않았다. 관측값이 없으면 배터리 상태를 만들지 않는다.

## 좌표

RMF graph, Nav2 map, Gazebo 8 m 방, Pinky `my_map`, `pinklab`, 병원 1층, 병원 7층은 서로 다른 좌표다. 카탈로그에 검증된 병원 맵이 없고 fleet의 `graph_alignment`는 `placeholder`다. scale, rotation, 층별 transform, waypoint 좌표는 계산해 넣지 않았다. 비교 함수는 resolution과 origin이 다르면 승격을 거절한다.

## 등록 조건

로봇을 움직임을 허용하는 상태로 등록하려면 다음이 동시에 필요하다.

- `graph_alignment`가 실측된 맵에 대해 `surveyed`
- `allow_motion`과 `simulation_confirmed`가 그 시험에 맞게 true
- `/pinkyN/navigate_to_pose`가 있고 pose가 신선함
- 하드웨어라면 그에 더해 도메인, 센서, 독립 정지, 운용자 승인. 지금은 이 승인을 코드가 받지 않는다

이 조건이 열리기 전에는 RMF task POST를 하지 않는다. 도메인 0의 RMF 프로세스는 바꾸지 않았다.

## 연결 가능 여부와 남은 일

설정과 차단 경로는 pinky1 하드웨어 계약에 맞춰 두었다. 실제 등록은 아직 불가하다.

- 하드웨어 `ROS_DOMAIN_ID`가 없다. router도 없다.
- 1층·7층 occupancy map이 없다.
- graph와 맵이 맞춰지지 않았다.
- 실기체 scan, odom, TF, 배터리를 이 PC에서 받지 못했다.
- `sllidar_ros2`가 이 PC에 없다.
- 드라이버 joint 이름과 URDF joint 이름, 바퀴 반지름 0.027과 0.028이 다르다.
- TF 프레임은 로봇 ID prefix가 없다. 한 도메인에 3대를 넣으면 겹친다.
- 물리 비상정지는 확인하지 않았다.
