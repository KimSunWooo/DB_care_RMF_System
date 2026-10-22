# Pinky encoder readiness

이번 문서는 Mock에서 확인한 결과, 실제 기기에서 확인한 사실, 아직 실행하지 않은 시험을 나눈다. 모터가 안전하다고 검증된 상태는 아니다. 시리얼 포트는 열지 않았고, torque enable, reboot, Goal Velocity 쓰기는 하지 않았다.

## Dynamixel 모델 식별 상태

모델명과 펌웨어는 확인되지 않았다. 추정해서 확정하지 않는다.

`/home/sw/DevSpace/pinky`의 `dynamixel_driver.py`와 bringup 소스에는 XL, XM, model number, firmware 문자열이 없다. 드라이버가 쓰는 주소는 아래와 같다.

| 항목 | 주소 | 드라이버가 하는 일 |
|---|---|---|
| Operating Mode | 11 | 초기화 때 값 1을 씀 |
| Torque Enable | 64 | 초기화 때 1을 씀. 종료 때 0을 씀 |
| LED Red | 65 | 초기화 때 1, 종료 때 0 |
| Goal Velocity | 104 | 속도 명령을 씀 |
| Profile Acceleration | 108 | 초기화 때 씀 |
| Present Velocity | 128 | 엔코더 읽기 |
| Present Position | 132 | 엔코더 읽기 |

`RPM_TO_VALUE_SCALE = 1/0.229`이다. 0.229 RPM은 ROBOTIS X-series Protocol 2.0 속도 단위와 같다. 이 단위와 위 주소는 XL430, XC330을 포함한 여러 모델의 공개 제어표에 같이 있다. 주소가 같다고 모델을 고르지 않는다.

공개 X-series 표에서 모델 번호는 주소 0의 2바이트, 펌웨어는 주소 6의 1바이트, ID는 주소 7의 1바이트다. XL430-W250 매뉴얼의 모델 번호 예시는 1060이다. 이 숫자는 이 로봇에서 읽은 값이 아니다. `hardware_safety/fake_bus.py`가 시험용으로 1060을 넣어도 `model_confirmed`는 false다.

주소 126은 모델에 따라 Present Load 또는 Present Current다. Pinky 드라이버는 이 주소를 쓰지 않는다.

Bus Watchdog은 공개 X-series 표의 주소 98, 1바이트다. XL430 매뉴얼에서 0은 비활성이고 1–127은 20 ms 단위다. Pinky 드라이버는 주소 98을 읽지도 쓰지도 않는다. 이 로봇이 그 레지스터를 가지는지는 모델 번호를 읽기 전에는 알 수 없다. `bus_watchdog_supported`는 None으로 둔다.

## 통신 프로토콜

드라이버는 Dynamixel SDK Protocol 2.0을 선택한다. 포트는 `/dev/ttyAMA4`, baudrate는 1000000, Motor ID는 1과 2다. 이 값은 소스와 기존 장치 정보이고, 이번 단계에서 PING으로 다시 확인하지 않았다.

이번 세션의 swPC에는 `/dev/ttyAMA0`과 `/dev/ttyAMA4`가 없다.

## Encoder와 Odometry

원본 `initialize_motors`는 reboot, Operating Mode 11에 1, Torque Enable 64에 1을 쓴 뒤에 Present Velocity와 Present Position을 읽는다. 읽기 전용 분기는 없다.

오른쪽 원시 엔코더는 `delta_r = -(raw_r - last)`다. 직진은 왼쪽 원시값이 늘고 오른쪽 원시값이 줄 때다. 바퀴 반지름 0.027 m와 간격 0.0961 m는 bringup 파라미터이며 보정값이 아니다. URDF와 Gazebo 플러그인의 반지름은 0.028 m다. 둘 중 하나를 측정값으로 고르지 않았다.

Mock joint 이름은 namespace `pinky1/`의 URDF와 같다. `pinky1/l_wheel_joint`, `pinky1/r_wheel_joint`를 한 번만 붙인다. 드라이버 원본 이름 `left_wheel_joint`, `right_wheel_joint`와는 다르다.

`commanded_vx == 0`은 소프트웨어 장부다. `physical_stop_confirmed`와 `bus_watchdog_verified`는 false다.

## 안전장치 상태

소프트웨어 쪽에서 확인한 것은 다음뿐이다.

- 비정상 명령, 시간 역행, 엔코더 점프, 피드백 공백은 이전 속도를 유지하지 않는다.
- Fake Transport의 정지 쓰기가 성공하면 `RECOVERY_REQUIRED`, 실패하면 `FAULT_LATCHED`다.
- 30 Hz에서 2048틱 점프는 거절된다.
- `ReadOnlyPlan.execute()`와 `FakeDynamixelBus.open("/dev/...")`는 포트 전에 예외를 낸다.
- Goal Velocity, Torque Enable, Operating Mode, Profile Acceleration, Bus Watchdog 쓰기는 fake bus에서도 예외다.

물리 비상정지는 확인하지 않았다. 원본 드라이버에는 `/cmd_vel` watchdog이 없다. 소프트웨어가 속도를 0으로 지워도 모터가 멈춘 것은 아니다.

## 실제 시리얼 readiness

준비되지 않았다.

- swPC에 모터 포트가 없다.
- 이번 테스트의 `port_opened`는 false다.
- 실제 PING과 READ는 실행하지 않았다.
- 모델 번호, 토크 바이트, 엔코더 값은 실측이 없다.

## Mock ROS 2 통신

도메인 90만 사용했다. 도메인 0, 42, 53에는 노드를 만들지 않았다. 전체 테스트는 93 passed다. 이전 88개에 프로세스 분리와 fake bus 시험이 추가됐다.

`scripts/odom_mock_ipc_check.py`는 발행 프로세스와 구독 프로세스를 나눈다. 구독 프로세스를 먼저 띄우고, 발행 프로세스가 끝난 뒤 다시 띄운다. 이번 실행 결과는 다음과 같다.

| 항목 | 결과 |
|---|---|
| Discovery와 타입 | `/pinky1/odom`은 `nav_msgs/msg/Odometry`, `/pinky1/joint_states`는 `sensor_msgs/msg/JointState`, `/tf`는 `tf2_msgs/msg/TFMessage` |
| QoS | 발행은 `RELIABLE` + `VOLATILE`, depth 10. 구독자가 본 reliability 값은 `1`이고 이는 reliable다 |
| 주기 | 평균 0.052 s. 재연결 공백은 평균에서 빼었다 |
| 직진 | `straight_before_turn` true, `max_x` 0.03313 m, forward twist 확인 |
| 회전 | `max_abs_yaw` 0.517 rad, turn twist 확인 |
| 정지 twist | 경로 끝에서 `saw_zero_twist` true |
| Frame | `pinky1/odom` → `pinky1/base_footprint` |
| Joint | `pinky1/l_wheel_joint`, `pinky1/r_wheel_joint` |
| TF와 pose | `tf_ok` true, 위치 차이 1 mm 미만 |
| 발행 종료 | `publisher_lost` true |
| 재연결 | `reconnected` true |
| 포트 | `port_opened` false |

## RViz2

정적 설정과 창 실행을 둘 다 했다.

`sim/hardware/mock_odom.rviz`의 Fixed Frame은 `pinky1/odom`이다. Odometry 토픽은 `/pinky1/odom`, RobotModel 토픽은 `/pinky1/robot_description`이다. `/scan`과 도메인 53 토픽은 없다. RobotModel이 토픽을 실제로 구독하도록 `Description Source: Topic`과 Transient Local QoS를 넣었다. 이 설정이 없으면 RViz 기본값은 파라미터를 찾고 토픽을 무시한다.

창은 `DISPLAY=:1`에서 열렸다. 도메인은 90이다. robot description은 로컬 `pinky_description` xacro를 `namespace:=pinky1/`로 풀어 `robot_state_publisher`만 도메인 90에 띄웠다. `frame_prefix`는 `pinky1/`다. `joint_state_publisher`는 켜지 않았다. 도메인 53의 description이나 scan은 가져오지 않았다. 이미 떠 있던 RViz는 `ROS_DOMAIN_ID=53`이었고 종료하지 않았다.

실행 중에 확인한 것:

- Global Status는 Ok, TF 표시는 Ok였다.
- `/tf_static`에 `pinky1/base_footprint`, `pinky1/base_link`, `pinky1/rplidar_link`를 포함한 14개 프레임이 있었다.
- 발행이 유지되는 동안 `tf2_echo pinky1/odom pinky1/base_footprint`는 이동 0.033 m, yaw -0.517 rad였다. 같은 시각의 `/pinky1/odom` pose x는 0.033133985018329856, y는 0이었다.
- 직진 구간의 화면에는 odometry 화살표가 하나 있었고, 회전 이후에는 화살표가 부채꼴로 쌓였다. 몸체 위치는 그 화살표를 따라 이동했다.
- RobotModel 항목은 오류 아이콘이었다. 몸체는 흰 상자로 보였고, `base_link.dae` 메쉬가 완전히 표시됐는지는 확인하지 못했다. RViz 로그에는 그 상태 문자열이 남지 않았다.

이 창 검증은 Mock TF와 Odometry에 한정된다. 실제 엔코더로 그린 자세가 아니다.

## 모델 번호를 확인하는 절차

아래는 설계다. 이번 단계에서 실행하지 않는다.

1. Pi에서 모터 드라이버가 없고 `/dev/ttyAMA4`를 연 프로세스가 없는지 확인한다.
2. 승인 뒤에만 Protocol 2.0, baudrate 1000000으로 ID 1과 2에 PING한다. broadcast reboot는 하지 않는다.
3. 주소 0을 2바이트, 주소 6을 1바이트, 주소 7을 1바이트 읽는다. ID가 요청과 다르면 중단한다.
4. 주소 64를 1바이트 읽는다. 0이 아니면 토크를 끄지 않고 중단한다.
5. 토크 바이트가 0일 때만 주소 132를 4바이트 읽는다.
6. 읽은 모델 번호의 제조사 제어표에서 주소 98이 있는지 본다. 표에 있을 때만 Bus Watchdog 지원으로 적는다. 주소 98 자체는 그 승인 전에는 읽지 않는다.
7. 응답 길이가 다르거나 통신이 실패하면 쓰기를 하지 않고 중단한다.

## 읽기와 토크 활성화

주소 132를 읽는 것과 주소 64에 1을 쓰는 것은 다른 명령이다. 소프트웨어로는 나눌 수 있다. `run_fake_identification`은 주입된 토크가 0일 때만 위치를 읽고, 0이 아니면 토크를 끄는 쓰기 없이 중단한다.

실제 서보가 토크가 꺼진 상태에서 위치를 돌려주는지는 측정하지 않았다. 원본 초기화는 그 읽기 전에 reboot와 토크 활성화를 한다. 실패한 읽기가, 이미 속도 모드인 모터를 멈추게 하지는 않는다.

## 실제 Encoder 시험의 진입 조건

여덟 가지가 모두 맞기 전에는 실행하지 않는다. 지금은 모두 미충족이다.

1. 실제 모델명과 모델 번호가 주소 0의 읽기로 확인될 것.
2. 물리 비상정지가 실제로 모터를 멈추는지 확인될 것.
3. 토크 상태를 주소 64 읽기로 확인하고, 0이 아니면 시험을 중단하는 경로가 실기에서 확인될 것.
4. 기존 모터 드라이버가 실행 중이 아닐 것.
5. `/dev/ttyAMA4`를 이 시험만 열고 있을 것.
6. 허용 명령이 PING과 위 읽기뿐일 것. Torque Enable, reboot, Goal Velocity, Operating Mode 쓰기는 없을 것.
7. 예상하지 못한 길이, ID, 통신 오류에서 즉시 중단할 것.
8. 운용자가 옆에 있고, 이번 문서와 별도의 실행 승인이 있을 것.

시험 코드가 필요하면 `hardware_safety/fake_bus.py`를 먼저 돌린다. `ReadOnlyPlan.execute()`는 승인 뒤에도 지금 코드 그대로는 포트를 열지 않는다.

## 남은 위험

- 모델이 공개 X-series 표와 다른 레지스터를 쓸 수 있다.
- 주소 98을 확인하기 전에는 버스 watchdog이 없다. `/cmd_vel`이 끊겨도 마지막 속도가 남을 수 있다.
- 반지름 0.027 m와 0.028 m가 달라 Mock 이동 거리 0.033 m는 실측 거리가 아니다.
- 엔코더 분해능 4096과 기어비는 소스 가정의 연장이다. 기어비는 소스에 없다.
- RobotModel 메쉬 표시는 오류 상태다.
- 물리 비상정지와 실제 토크 상태는 모른다.

## 이전에 실제 기기에서 확인했던 사실

이번 세션에서 Pi로 다시 접속하지는 않았다. 이전 확인은 LiDAR `/scan`, robot description, 정적 TF `base_footprint` → `rplidar_link`가 swPC에서 도메인 53으로 보였다는 것이다. 그때 모터 드라이버는 실행하지 않았다. 엔코더 값, 모델 번호, 토크 상태는 그때도 읽지 않았다.
