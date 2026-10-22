# Care-Move 진행 현황

2026-10-22. 다중로봇, 엘리베이터, 도킹, 대기열은 범위 밖이다. Git commit은 하지 않았다.

Gazebo에서 관리자 UI → RMF → Fleet Adapter → Nav2 SUCCEEDED → RMF completed는 이전 검증 그대로다. 이번 변경은 그 경로를 수정하지 않았다.

실제 Pinky1의 모터 구동, Nav2 Goal, RMF task는 물리 정지 수단이 확인되지 않아 BLOCKED다. 일반적인 주행 허용은 그 확인을 대신하지 않는다.

| 검증 항목 | 결과 | 증거 |
|---|---|---|
| 실제 Motor Model | PASS | ID 1·2 PING 1200, firmware 52. XL330-M288-T. 쓰기 0회 |
| Encoder READ | PASS | 최신 위치 ID 1 = 2964, ID 2 = 2244, velocity 0, torque 0 |
| Safety Driver | PASS | `hardware_safety/real_driver.py`. 최신 샘플은 `identified`, torque 쓰기 0, 승인 없으면 arm 거부 |
| ROS Command Watchdog | PASS | 가짜 transport에서 0.4초 공백 시 stop 1회. 실패 시 `fault_latched` 후 trial 재시작 없음. 실모터에는 속도를 쓰지 않음 |
| Bus Watchdog | BLOCKED | 주소 98 읽기 값 0. 쓰지 않음. Encoder READ가 패킷 간격을 갱신하므로 cmd_vel watchdog으로 쓰지 않음 |
| 물리 정지 수단 | BLOCKED | 존재와 작동을 확인하지 않음 |
| 실제 Odometry | NOT_TESTED | `/odom` 미발행. 정지 샘플 적분은 단위 테스트에서 x=0 |
| 실제 TF | NOT_TESTED | `odom` → `base_footprint` 없음. 정적 `base_footprint` → `base_link`는 있음 |
| 공중 바퀴 회전 | BLOCKED | torque를 켜지 않음 |
| 실제 5cm 주행 | BLOCKED | |
| 실제 10cm 주행 | BLOCKED | |
| 실제 Nav2 Goal | BLOCKED | Action 서버 없음 |
| RMF Fleet 등록 | BLOCKED | `real_pinky1` 게이트는 명령 전달이 꺼져 있음 |
| RMF Task Dispatch | BLOCKED | 도메인 77과 Gazebo ID로는 거부 |
| 실제 Pinky 이동 | BLOCKED | |
| Nav2 SUCCEEDED | BLOCKED | |
| RMF completed | BLOCKED | |
| 관리자 UI 반영 | BLOCKED | 실기체 task를 만들지 않음 |
| 기존 94개 테스트 회귀 | PASS | `pytest -q` 102 passed. 기존 94에 실기체 게이트 8개가 추가됨 |

이전 읽기 위치는 ID 1 = 2950, ID 2 = 2373이었다. 이번 읽기는 2964, 2244다. 두 번 모두 Torque는 0이었고 이번 속도 원시값은 0이다. 이 차이는 모터 명령으로 만든 이동이 아니다.

추가된 파일은 `hardware_safety/real_driver.py`, `fleet_adapter/real_gate.py`, `tests/test_real_pinky_driver.py`이다. Pinky 원본, Mock safety, Gazebo adapter는 수정하지 않았다.
