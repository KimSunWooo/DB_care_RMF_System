# 실제 Pinky Pro 하드웨어 시험

2026-10-22, swPC. 이번 시험은 도메인 53만 대상으로 했다. Gazebo 도메인 42, 시뮬레이션 RMF 도메인 77, mock 도메인 90, 운영 RMF 도메인 0은 변경하지 않았다. Pinky 원본과 Care-Move Hardware Safety 코드도 수정하지 않았다. Git commit은 하지 않았다.

모터 Torque Enable, Goal Velocity, `/cmd_vel`, Nav2 Goal은 보내지 않았다.

## 연결 결과

`pinky@192.168.0.219` 비밀번호 SSH로 호스트 `raspi`에 접속했다. SSH 서버 설정과 키는 바꾸지 않았고, 비밀번호는 저장하지 않았다.

Pi에서 도메인 53, RMW 미설정(Jazzy 기본 Fast DDS)으로 본 그래프는 swPC와 같다.

```text
nodes    /sllidar_node, /robot_state_publisher, /joint_state_publisher
topics   /scan, /joint_states, /robot_description, /tf, /tf_static
actions  없음
```

`/cmd_vel`와 `/odom`은 없다. `/dev/ttyAMA0`은 `sllidar_node` PID 2625가 사용 중이다. `/dev/ttyAMA4`를 연 프로세스는 없었다. 읽기 시험만 그 포트를 열었다가 닫았다. 레지스터 쓰기는 0회다.

## A. 하드웨어

Protocol 2.0, 1000000 baud, ID 1과 2. 모델 번호 1200은 ROBOTIS XL330-M288-T Control Table의 Model Number와 같다. 출처는 `https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/`.

| 항목 | ID 1 | ID 2 |
|---|---|---|
| PING 모델 번호 | 1200 | 1200 |
| Firmware Version | 52 | 52 |
| Torque Enable (64) | 0 | 0 |
| Hardware Error Status (70) | 0 | 0 |
| Bus Watchdog (98) | 0, 읽기 성공 | 0, 읽기 성공 |
| Present Velocity (128) | 1 (약 0.229 rpm) | 1 (약 0.229 rpm) |
| Present Position (132) | 2950, 0.3초 뒤에도 2950 | 2373, 0.3초 뒤에도 2373 |

속도 단위 0.229 rpm은 같은 매뉴얼의 Present Velocity 단위다. 이후 재읽기에서는 Torque 0, velocity 0, 위치 2964와 2244였다. 그 사이 모터 명령은 없었다. Bus Watchdog은 쓰지 않았다. 매뉴얼에서 이 기능은 Torque ON일 때 모든 Instruction Packet 간격을 본다. Encoder READ가 계속되면 `/cmd_vel`이 끊겨도 타임아웃이 나지 않는다.

물리 정지 장치의 작동은 확인하지 못했다. 모터 bringup은 실행하지 않았다. 원본 `initialize_motors`는 읽기 전에 reboot와 Torque Enable을 쓴다.

## B. ROS 2

| 항목 | 이번 조회 |
|---|---|
| Domain | 53. lidar, robot_state_publisher, joint_state_publisher 프로세스 환경도 53 |
| RMW | 프로세스 환경에 `RMW_IMPLEMENTATION` 없음. swPC는 `rmw_fastrtps_cpp`로 같은 그래프를 수신 |
| `/scan` | `/sllidar_node`, 이전 측정 약 10.0 Hz |
| `/joint_states` | `l_wheel_joint`, `r_wheel_joint`, caster 2개. 위치는 모두 0.0. Encoder가 아님 |
| `/tf_static` | `base_footprint` → `base_link` → `rplidar_link` 포함. `odom` 프레임 없음 |
| `/cmd_vel`, `/odom`, Action | 없음 |

소스 기준 실제 bringup은 namespace 없는 `cmd_vel`, `odom`, `joint_states`를 쓰고 TF는 `odom` → `base_footprint`이다. Gazebo의 `/pinky1/...`와 다르다.

URDF joint는 `${namespace}l_wheel_joint`, `${namespace}r_wheel_joint`이다. bringup이 발행하는 이름은 `left_wheel_joint`, `right_wheel_joint`이다. 실기체에서 어떤 쪽이 발행 중인지는 확인하지 못했다.

원본 운동학은 바퀴 반지름 0.027 m, 간격 0.0961 m, 4096 pulse이다. 오른쪽 바퀴 RPM과 encoder delta 부호를 반전한다. `/cmd_vel` timeout은 없다. 프로세스가 `SIGKILL`로 죽으면 `terminate()`의 RPM 0과 Torque OFF가 실행되지 않는다.

## C. 실제 이동 결과

| 테스트 | 결과 | 실제 측정값 |
|---|---|---|
| Encoder Read | PASS | 이전 2950/2373. 이번 재읽기 2964/2244, velocity 0, torque 0. 쓰기 0회 |
| Driver Init | 미실행 | bringup은 reboot와 torque enable을 하므로 시작하지 않음 |
| Motor Stop | 미검증 | Torque는 이미 0. 움직이는 모터를 멈추는 시험은 하지 않음 |
| Wheel 방향 | 미검증 | |
| `/cmd_vel` 발행 | 미실행 | |
| 5cm 직진 | 미실행 | |
| 10cm 직진 | 미실행 | |
| `/scan` | PASS | `/sllidar_node` 실행, `/dev/ttyAMA0` 점유 |
| `/odom` | 미검증 | 토픽 없음 |
| TF | 부분 확인 | 정적 TF만 있음. `odom` → `base_footprint` 없음 |
| Nav2 Goal | 미실행 | Action 서버 없음. Gazebo 좌표를 보내지 않음 |
| Goal Cancel | 미실행 | |
| 실제 최종 정지 | 미검증 | 이번 세션에서 모터를 구동하지 않음 |

주행을 시작하지 않은 이유:

1. `/cmd_vel` 타임아웃이 실제 Goal Velocity 0 쓰기까지 연결되어 있지 않다. Bus Watchdog은 0이고, Encoder READ만으로도 패킷 간격이 갱신된다.
2. 물리 정지 수단의 작동을 확인하지 못했다.
3. 원본 드라이버는 초기화 때 Torque를 켜고, `/cmd_vel`이 끊겨도 마지막 속도를 지우지 않는다.
4. 바퀴를 지면에서 분리한 상태인지 확인할 수 없다.

## D. 코드 변경

- 추가: `hardware_safety/xl330_bus.py`, `scripts/real_pinky_odom_once.py`. `real_driver.py`는 전원 차단을 관찰하기 전에는 arm하지 않는다
- Pinky 원본, Mock `OdomSession`, Gazebo fleet adapter는 수정하지 않음
- 2026-10-22 Torque OFF 발행: 60샘플, 이동 0.0 m, Operating Mode 1, 서보 입력 5.0V, 버스 쓰기 0회. swPC가 `/odom`과 `odom` → `base_footprint`를 수신
- `pytest -q` 103 passed
- 상태 표는 `docs/CARE_MOVE_IMPLEMENTATION_STATUS.md`

## E. 다음 단계

실제 Pinky1은 RMF task를 받을 상태가 아니다. 도메인 77이나 도메인 0에서 이 로봇으로 task를 배정하지 않는다.

남은 순서는 다음과 같다.

1. 모델 1200, Torque 0, Encoder 정지는 확인했다.
2. 주행 전에 `/cmd_vel` 0.3–0.5초 미수신 시 Goal Velocity 0 쓰기가 성공하는지 따로 검증한다. Bus Watchdog은 Encoder READ와 분리해서 본다.
3. 운용자의 가드·전원 스위치 확인은 기록했다. 전원을 끊어 구동부가 꺼지는 것을 관찰한 뒤에만 Torque를 켠다.
4. `/cmd_vel` 0.3–0.5초 timeout이 실제 모터 정지까지 확인되고, 물리 정지 수단이 확인된 뒤에만 바퀴를 띄운 짧은 회전을 한다.
5. 그 다음 0.03 m/s, 0.05 m, 5초 제한의 지면 주행을 한 번만 한다.
6. 실기체 지도, `map` → `odom`, AMCL, NavigateToPose 서버가 있을 때만 Nav2 goal을 검토한다. Gazebo의 `(1.2, 0)`은 사용하지 않는다.
