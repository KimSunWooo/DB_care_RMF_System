# Encoder odometry safety

Pinky 원본 `bringup.py`와 `dynamixel_driver.py`는 수정하지 않았다. Care-Move의 `hardware_safety`는 그 수식을 흉내 내되 시리얼 포트를 열지 않는다.

## 원본에서 확인한 동작

`initialize_motors`는 operating mode 1을 쓰고 `ADDR_TORQUE_ENABLE`(64)에 1을 쓴다. 그 다음에야 엔코더를 읽는다. `/cmd_vel`이 끊겨도 마지막 RPM이 남는다. 버스 watchdog 레지스터는 드라이버에 없다. 모델명이 확인되지 않아 레지스터를 넣지 않았다.

엔코더 차분은 `delta_r = -(raw_r - last_raw_r)`이다. 오른쪽 원시 값이 줄어야 직진이다. joint 이름은 `left_wheel_joint`, `right_wheel_joint`다. URDF는 `l_wheel_joint`, `r_wheel_joint`다.

`upload_robot.launch.py`의 `joint_state_publisher`는 `source_list: ['joint_states']`라서 자신이 내는 토픽과 같은 이름을 구독한다. bringup도 `/joint_states`를 낸다. 둘을 같이 켜면 바퀴 상태가 겹친다.

바퀴 반지름은 bringup 0.027 m, URDF와 Gazebo 플러그인 0.028 m다. 둘 중 하나를 보정값으로 고르지 않았다. Mock 수식은 bringup 파라미터를 그대로 쓰고 `bringup_parameter_unverified`로 표시한다.

## 모드

| 모드 | 하는 일 | 모터 |
|---|---|---|
| `SENSOR_ONLY` | LiDAR와 robot description 용 소스만 허용 | 포트 닫힘, cmd_vel 거절 |
| `ODOM_MOCK` | 가상 엔코더로 자세 계산 | 포트 닫힘, RPM 미전송 |
| `HARDWARE` | 설정 검사 | 포트 닫힘, goal과 cmd_vel 거절 |

순수 계산 세션은 메시지를 내지 않는다. ROS 발행은 도메인 90에서만 하고, 0·24·25·42·53이면 `rclpy.init` 전에 종료한다. 53은 지금 Raspberry Pi의 Fast DDS 도메인이다.

`sim/hardware/pinky1_bringup.launch.py`는 세 모드 모두에서 예외로 끝난다. `bringup_robot.launch.xml`을 include하지 않는다.

## 소프트웨어 정지의 한계

비정상 명령, 시간 역행, 샘플 간격 이상, 엔코더 점프, 피드백 공백은 `STOP_REQUIRED`를 거친 뒤 Fake Transport의 정지 쓰기 결과로 갈린다. 쓰기가 성공하면 `RECOVERY_REQUIRED`이고, 실패하면 `FAULT_LATCHED`다. 어느 쪽이든 이전 비영 속도는 지운다. `request_recovery()`는 `RECOVERY_REQUIRED`에서만 기준 엔코더를 새로 잡고, 예전 속도를 다시 넣지 않는다. `commanded_vx == 0`과 `stop_write_acked`는 소프트웨어 장부다. `physical_stop_confirmed`는 이 프로세스에서 항상 false다.

엔코더 허용 틱은 `0.25 m/s`와 `1.0 rad/s`로 바퀴 속도를 잡고, 실제 `dt`와 여유 1.5를 곱한다. 30Hz에서 이 값은 2048보다 작아서 2048틱 점프는 거절된다. 샘플 간격이 0.1초를 넘으면 그 간격으로 허용 틱을 키우지 않고 장애로 본다. 반지름 0.027 m는 bringup 파라미터이며 측정 보정값이 아니다.

Mock ROS 발행은 `scripts/odom_mock_ros_check.py`가 `ROS_DOMAIN_ID=90`일 때만 `rclpy.init`을 호출한다. 토픽은 `/pinky1/odom`, `/pinky1/joint_states`, `/tf`다. URDF는 namespace `pinky1/`일 때 joint 이름이 `pinky1/l_wheel_joint`이므로 JointState도 그 이름을 한 번만 붙인다. `frame_prefix`도 링크 프레임에 `pinky1/`를 붙인다. RViz 설정은 `sim/hardware/mock_odom.rviz`이고 Fixed Frame은 `pinky1/odom`이다. 이 파일을 도메인 53에서 열지 않는다.

## 읽기 전용 엔코더

`hardware_safety/read_only.py`의 `execute()`는 포트를 열기 전에 예외를 낸다. 드라이버 소스에 있는 주소는 operating mode 11, torque 64, goal velocity 104, present velocity 128, present position 132다. 모델명과 bus watchdog 주소는 소스에 없다. 현재 `initialize_motors`는 읽기 전에 reboot와 torque enable을 한다. 읽기만 하는 경로로 분리되어 있지 않고, 읽기 자체도 `/dev/ttyAMA4` 버스를 쓴다. 실행은 별도 승인 전이다.

## Raspberry Pi에 나중에 넣을 변경

이번 작업은 Pi에 배포하지 않는다. 모터를 켜기 전에 Pi 쪽 드라이버에 필요한 변경은 다음이다.

1. 초기화에서 토크를 켜기 전에 읽기 전용 엔코더 경로를 둔다.
2. `/cmd_vel`이 0.5초 이상 없으면 목표 RPM을 0으로 보내는 watchdog을 둔다. 쓰기가 실패하면 실패로 남긴다.
3. joint 이름을 `l_wheel_joint`, `r_wheel_joint`로 맞춘다.
4. 엔코더 joint를 낼 때는 `joint_state_publisher`를 켜지 않는다.
5. 32비트 짧은 차분만 쓰고, 허용 틱은 실제 샘플 간격과 0.25 m/s 한계로 계산한다. 30Hz에서 2048틱은 통과시키지 않는다.
6. 버스 watchdog은 서보 모델 매뉴얼의 레지스터를 확인한 뒤에만 설정한다.

## 다음 단계에서 `/odom`을 보는 방법

1. Pi에서는 지금처럼 LiDAR와 robot description만 둔다. bringup은 실행하지 않는다.
2. swPC에서 도메인 90의 프로세스 안에서 Mock을 실행한다. 도메인 53에는 노드를 만들지 않는다.
3. 읽기 전용 확인은 도메인 53에서 `ros2 topic echo --once /odom`이다. bringup이 꺼져 있으면 메시지가 없는 것이 맞다.
4. 토크를 켜는 시험은 물리 비상정지가 확인되고, watchdog 실패 시 모터가 실제로 멈추는 것이 확인된 뒤에 별도로 한다.
