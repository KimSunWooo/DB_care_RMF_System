# Care-Move 진행 현황

2026-10-22 (Day 4). 다중로봇, 엘리베이터, 도킹, 대기열은 범위 밖이다. Day 3 보고서는 작성하지 않았다. Git commit은 하지 않았다.

Gazebo에서 관리자 UI → RMF → Fleet Adapter → Nav2 SUCCEEDED → RMF completed는 이전 검증 그대로다. 이번 변경은 그 경로를 수정하지 않았다.

운용자는 2026-10-22에 물리 가드와 전원 차단이 가능하다고 확인했다. 전원을 실제로 끊어 구동부가 꺼지는지는 관찰하지 못했다. 그 실측 전에는 Motor Arm과 이동, Nav2 Goal, RMF task를 실행하지 않는다.

| 검증 항목 | 결과 | 실행 로그·증거 |
|---|---|---|
| 안전장치 확인 | NOT_TESTED | 운용자는 가드와 전원 스위치가 있다고 확인했다. 전원을 끊어 구동부가 꺼지는 과정은 관찰하지 못했다 |
| Motor Arm Gate | PASS | `power_cut_observed`가 없으면 arm이 `power cut has not been observed`를 반환하고 torque 쓰기는 0이다. 단위 테스트로 확인 |
| Dynamixel Torque | PASS | 읽기 중 Torque 0, Operating Mode 1, 입력 전압 5.0V. 쓰기 0회 |
| Encoder READ | PASS | Torque OFF 4초, 60샘플, 이동 0.0 m |
| 실제 /odom | PASS | 도메인 53에서 swPC가 `/odom` `odom` → `base_footprint`, x=0, y=0, twist 0을 수신 |
| 실제 TF | PASS | `/tf`의 `odom` → `base_footprint` x=0, y=0, stamp 1792659111. 바퀴 joint는 기존 `joint_state_publisher`의 0.0이 유지됨 |
| 공중 바퀴 회전 | BLOCKED | 전원 차단 실측 전이라 torque를 켜지 않음 |
| 5cm 직진 | BLOCKED | |
| 10cm 직진 | BLOCKED | |
| 실제 정지 | NOT_TESTED | 움직이는 모터를 멈추는 시험은 하지 않음. 읽기 중 velocity는 0 |
| 실제 Nav2 Goal | BLOCKED | Action 서버 없음. Goal을 보내지 않음 |
| Fleet Adapter 등록 | BLOCKED | `real_pinky1` 명령 전달은 꺼져 있음 |
| 실제 RMF Task | BLOCKED | |
| Nav2 SUCCEEDED | BLOCKED | |
| RMF completed | BLOCKED | |
| 관리자 UI 표시 | BLOCKED | 실기체 task를 만들지 않음 |
| 전체 회귀 테스트 | PASS | `pytest -q` 103 passed |

이전 읽기 위치는 ID 1 = 2950, ID 2 = 2373이었다. 이번 읽기는 2964, 2244다. 두 번 모두 Torque는 0이었고 이번 속도 원시값은 0이다. 이 차이는 모터 명령으로 만든 이동이 아니다.

이번 실행에서 추가하거나 수정한 파일은 `hardware_safety/real_driver.py`, `hardware_safety/xl330_bus.py`, `scripts/real_pinky_odom_once.py`, `tests/test_real_pinky_driver.py`이다. Pinky 원본, Mock `OdomSession`, Gazebo adapter는 수정하지 않았다. `/dev/ttyAMA4`를 연 기존 프로세스는 없었고, 읽기 프로세스는 종료했다. `joint_state_publisher`는 그대로 두었다.
