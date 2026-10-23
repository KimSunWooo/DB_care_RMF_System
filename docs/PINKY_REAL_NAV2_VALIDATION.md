# 실제 Pinky1 Nav2

2026-10-22. 도메인 53에서 NavigateToPose Action 서버는 없다. `/odom`과 `odom` → `base_footprint`도 없다. 정적 TF `base_footprint` → `base_link` → `rplidar_link`와 `/scan`만 있다.

Nav2 Goal은 보내지 않았다. Gazebo의 `(1.2, 0)`도 보내지 않았다.

| 항목 | 결과 |
|---|---|
| `/scan` | PASS. `/sllidar_node`, 약 10 Hz |
| `/odom` | PASS. Torque OFF 4초 동안 swPC 수신, x=0, y=0, twist 0. 발행 프로세스는 종료됨 |
| `odom` → `base_footprint` | PASS. `/tf` stamp 1792659111, x=0, y=0 |
| `base_footprint` → `base_link` | PASS. `/tf_static` |
| 지도, AMCL, `map` → `odom` | NOT_TESTED |
| NavigateToPose | NOT_TESTED. Action 서버 없음 |
| Goal 실행 | BLOCKED. 단거리 주행과 현장 정지 승인이 없다 |

엔코더가 정지일 때 오도메트리 적분도 정지인 것은 `tests/test_real_pinky_driver.py`에서 확인했다. 그 계산을 도메인 53에 publish하지 않았다. `joint_state_publisher`가 `l_wheel_joint`, `r_wheel_joint`를 0.0으로 이미 내고 있어서 같은 토픽에 엔코더 값을 겹치지 않았다.
