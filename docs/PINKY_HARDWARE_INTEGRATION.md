# 실제 Pinky Pro 연동

2026-10-21, swPC. Pinky 원본(`/home/sw/DevSpace/pinky`)은 읽기만 했다. 이 문서는 그 소스와 Care-Move Gazebo 계약의 차이를 적고, 실기체를 붙이기 전에 막는 조건을 적는다. 시리얼 포트는 Pinky bringup 기본값이다. IP는 소스에 없어서 넣지 않았다.

## 실기체 ROS 인터페이스

드라이버는 `pinky_bringup`. 토픽 이름은 상대 이름이다. `bringup_robot.launch.xml`에는 namespace 인자가 없다.

| 항목 | 실기체 소스 | 메시지 |
|---|---|---|
| cmd_vel | `cmd_vel` (`bringup.py` `TWIST_SUB_TOPIC_NAME`) | `geometry_msgs/msg/Twist` |
| odom | `odom`. frame `odom`, child `base_footprint` | `nav_msgs/msg/Odometry` |
| scan | SLIDAR C1 launch. frame `rplidar_link`. 토픽 이름은 드라이버 기본값 | `sensor_msgs/msg/LaserScan` |
| joint_states | `left_wheel_joint`, `right_wheel_joint` | `sensor_msgs/msg/JointState` |
| battery | `battery/percent`, `battery/voltage`. 5초. 노드 이름 `battery_publihser` | `std_msgs/msg/Float32` |
| TF | `odom` → `base_footprint`. prefix 없음 | `tf2_msgs` |
| 모터 | `/dev/ttyAMA4`, 1000000 baud, ID 1·2 | Dynamixel |
| LiDAR | `/dev/ttyAMA0`, scan mode `DenseBoost` | sllidar_ros2 |
| 바퀴 | launch 기본 radius 0.027 m, separation 0.0961 m | |
| 저전압 | 6.8 V 이하면 로그만 남긴다. 모터를 끄지 않는다 | |

`sllidar_ros2`는 이 PC의 Jazzy 설치에 없다. `slam_toolbox`는 `/opt/ros/jazzy`에 있다. `/dev/ttyAMA0`과 `/dev/ttyAMA4`는 swPC에 없다.

저전압 경고와 프로세스 종료 시 `driver.terminate()`가 소프트웨어로 확인할 수 있는 정지 경로다. 물리 비상정지가 구현된 것은 아니다.

## Gazebo와 실기체의 차이

| 항목 | Gazebo (`ROBOT_BACKEND=gazebo`) | 실기체 (`ROBOT_BACKEND=hardware`) |
|---|---|---|
| 도메인 | 42 | 비어 있음. 0·24·25·42는 거부 |
| `use_sim_time` | true | false |
| cmd_vel | `/pinky1/cmd_vel` Twist | 드라이버는 `cmd_vel`. wrapper가 namespace를 밀어 `/pinky1/cmd_vel`로 맞춘다 |
| odom | `/pinky1/odom`, frame `pinky1/odom` | `/pinky1/odom`을 기대. frame은 `odom` |
| scan | `/pinky1/scan`, frame `pinky1/rplidar_link`, 640 samples, 약 0.10초 | frame `rplidar_link`. 주기와 샘플 수는 미측정 |
| 바퀴 joint | URDF `l_wheel_joint`, `r_wheel_joint` | 드라이버 `left_wheel_joint`, `right_wheel_joint` |
| 바퀴 반지름 | Gazebo xacro 0.028 m | bringup 0.027 m |
| Nav2 goal | `/pinky1/navigate_to_pose` | 같은 액션 이름. 서버는 아직 기동하지 않음 |
| 배터리 | 없음 | `std_msgs/Float32` 두 토픽 |

상위 계약은 `/{robot_id}/cmd_vel`, `/{robot_id}/odom`, `/{robot_id}/scan`, `/{robot_id}/navigate_to_pose`이다. Gazebo는 이미 이 이름이다. 실기체 드라이버는 상대 이름과 prefix 없는 프레임을 쓴다. Care-Move는 Pinky 소스를 고치지 않고 `sim/hardware/pinky1_bringup.launch.py`에서 namespace `pinky1`을 밀어 토픽 이름만 맞춘다. 프레임 prefix는 드라이버 안에 하드코드되어 있어 이 wrapper만으로 `pinky1/odom`이 되지 않는다. 로봇 3대가 한 ROS 그래프에 들어가면 TF가 겹친다. 지금은 로봇마다 도메인을 나누는 쪽을 기본으로 두고, 도메인 값은 비워 둔다.

joint 이름과 바퀴 반지름 차이는 오돔 오차로 이어질 수 있다. 실측 전에 둘 중 하나를 고르지 않았다.

## Hardware bringup

순서:

1. `ROBOT_BACKEND=hardware`, `ROBOT_ID=pinky1`
2. `scripts/hardware_preflight.py`로 설정 검사. 이 스크립트는 토픽을 내지 않는다.
3. `ROS_DOMAIN_ID`가 프로필과 같고 0·24·25·42가 아닐 것
4. `PINKY_BRINGUP_SHARE`가 있을 것
5. 그 다음에만 `pinky_bringup/launch/bringup_robot.launch.xml`을 namespace 아래에서 include

현재 프로필의 하드웨어 도메인은 `null`이다. launch 파일은 그 상태에서 `RuntimeError`로 끝나고 드라이버를 include하지 않는다. Gazebo 플러그인과 이 launch를 한 프로세스에 넣지 않는다.

pinky2, pinky3는 같은 프로필의 `enabled: false`만 다르다. 네임스페이스는 로봇 ID와 같다.

## ROS Domain과 DDS

RMF는 도메인 0, Gazebo 연습은 도메인 42다. 둘 다 유지한다. 하드웨어 도메인이 정해지기 전에는 DDS router를 두지 않는다. 도메인이 0이나 42와 달라지면, 그 때 명시적인 router가 필요하다. 이번 설정은 router를 시작하지 않는다.

`RMW_IMPLEMENTATION`은 기존 Gazebo 환경과 같이 `rmw_cyclonedds_cpp`를 전제로 한다. 하드웨어 NIC와 로봇 IP는 비어 있다.

## 안전 조건

`robot_profile/safety.py`의 하드웨어 goal은 항상 거절한다. 프로필 파일에 `hardware_motion_approved: true`를 넣으면 로더가 오류를 낸다. 환경 변수로 승인을 켜도 이 프로세스는 그 값을 이동 허가로 받지 않는다.

이동 전에 따로 확인되어야 하는 것:

- 운용자 승인
- 로봇 ID가 설정과 같은지
- 독립된 모터 정지 수단
- 속도·가속도 제한
- 명령 타임아웃
- DDS가 끊기면 goal을 실패로 바꿈 (`Nav2Link.mark_disconnected`)
- Nav2 취소 후 속도가 0인지
- 위치 추정 실패와 센서 끊김 때 주행 중단

이 항목 중 물리 정지는 소프트웨어 플래그가 대신하지 못한다. `scripts/hardware_preflight.py`의 `motion_sent`는 false다.

## 연결 절차

1. 로봇과 이 PC의 도메인, RMW, NIC를 실측해 프로필의 `ros_domain_id`만 채운다.
2. 읽기 전용으로 `/pinky1/scan`, `/pinky1/odom`, TF, `battery/voltage`를 확인한다. `cmd_vel`은 구독자만 보고 메시지는 보내지 않는다.
3. 지도가 `surveyed`로 바뀌기 전에는 Nav2 goal을 보내지 않는다.
4. 저속 시험은 안전 담당자와 관찰자가 있는 공간에서, 별도 승인 뒤에 한다.

이번 실행에서는 1단계 전에 끝냈다. swPC에서 시리얼 장치가 없고, 하드웨어 launch는 기동하지 않았다.
