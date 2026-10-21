# Pinky 하드웨어 준비 검증

2026-10-21, swPC, 사용자 `sw`. Git commit과 push는 하지 않았다. RMF 도메인 0과 Gazebo 도메인 42 설정은 바꾸지 않았다. `cmd_vel`과 Nav2 goal, 승강기 명령은 보내지 않았다.

구현은 코드와 단위 테스트가 있는 상태다. 검증은 그 코드를 실행했거나, 이전에 Gazebo에서 측정한 것만 적는다. 기체가 없어 보지 못한 항목은 검증란에 미실행이다.

| 항목 | 구현 | 검증 | 근거 |
|---|---|---|---|
| Gazebo Pinky | 유지. launch, bridge, 맵, Nav2 파라미터 | Gazebo에서 측정됨 | `docs/GAZEBO_NAV2_VALIDATION.md`. scan 640개, 주기 약 0.10초. goal `(1.2, 0)` → `(1.006, 0.002)`, `(0, 0)` → `(0.196, 0.004)`, 둘 다 status 4 |
| 실제 Pinky bringup | `sim/hardware/pinky1_bringup.launch.py`가 도메인 검사 뒤에만 Pinky launch를 include | 미실행. 도메인이 비어 있어 launch는 기동 전에 예외 | `hardware_launch_block`이 unset domain을 반환. 단위 테스트가 그 거절을 확인 |
| 실기체 ROS 통신 | 토픽 계약과 preflight | 미실행. swPC에 `/dev/ttyAMA0`, `/dev/ttyAMA4` 없음 | `scripts/hardware_preflight.py`. `serial_device_present` 실패. `motion_sent` false |
| LiDAR | Gazebo는 `/pinky1/scan`. 실기체 설정은 `/dev/ttyAMA0`, frame `rplidar_link` | Gazebo만 측정. 실기체 미실행 | `scan_check.py` passed true. `sllidar_ros2`는 이 PC Jazzy에 없음 |
| Odometry | Gazebo `/pinky1/odom`. 실기체 frame `odom` → `base_footprint` | Gazebo 주행에서 사용됨. 실기체 미실행 | Nav2 goal 두 번이 odom과 TF로 종료. 하드웨어 샘플 없음 |
| TF | Gazebo는 `pinky1/` prefix. 실기체는 prefix 없음 | Gazebo 트리만 확인 | `tf_relay.py`와 `tf2_echo`. 하드웨어 TF는 미수신 |
| 기존 지도 로딩 | 카탈로그와 `require_navigation_map` | 단위 테스트. Gazebo 맵 파일 존재와 필드 확인 | `gazebo_test_room` 0.05, origin `[-4, -4, 0]`. `floor_1`과 `floor_7`은 missing으로 거절 |
| Localization | Gazebo AMCL 파라미터 유지. 하드웨어는 Pinky nav launch를 가리킴 | Gazebo goal 성공으로 간접 확인. 하드웨어 미실행 | 도착 오차 약 0.19 m, tolerance 0.2 m. 하드웨어 AMCL은 시작하지 않음 |
| Nav2 | Gazebo lifecycle와 액션 이름. 하드웨어 파라미터 파일은 가리키기만 함 | Gazebo에서 6개 lifecycle active, 액션 2회 성공. 하드웨어 미실행 | `docs/GAZEBO_NAV2_VALIDATION.md` |
| SLAM | mapping 모드가 slam launch를 가리키고 staging 밖 저장을 거절 | 단위 테스트. slam_toolbox 패키지는 설치됨. 실기체 매핑 미실행 | `master_replace_allowed`는 false. 로봇을 움직이지 않음 |
| Fleet Adapter | 기존 Nav2Link와 safety. 하드웨어 goal은 추가 차단 | 단위 테스트. RMF task POST는 하지 않음 | placeholder graph와 `allow_motion` false가 먼저 막힘. 하드웨어는 fleet 조건을 열어도 막힘 |
| RMF graph | 좌표를 만들지 않는 goal 명령 | 설정은 여전히 placeholder. 실측 정합 없음 | `fleet_goal_command`가 placeholder에서 예외. 8 m 테스트 방을 병원 그래프에 맞추지 않음 |
| 실기체 이동 안전성 | 승인값을 파일에 저장하면 로더가 실패. goal 함수는 하드웨어를 항상 거절 | 코드 경로만 확인. 물리 정지와 저속 주행은 미실행 | preflight `goal_blocked`는 그래프 이유. 모터 명령 없음 |

## 테스트

`env -u PYTHONPATH -u AMENT_PREFIX_PATH PYTHONNOUSERSITE=1 .venv/bin/python -m pytest -q`

결과: **68 passed**, 1 warning. 이전 58개에 프로필 테스트 10개가 더해졌다. ROS `PYTHONPATH` 없이 실행했다.

단위 테스트가 커버하는 것:

1. gazebo와 hardware 프로필이 도메인, sim time, 맵을 따로 가진다.
2. pinky1·pinky2·pinky3의 namespace와 토픽이 `/{id}/...`이다. pinky2·pinky3는 disabled다.
3. cmd_vel 타입은 양쪽 모두 Twist이고 프레임은 백엔드마다 다르다.
4. `gazebo_test_room`은 로드된다.
5. 필드가 빠진 YAML, 미검증 Pinky 맵, 없는 층 맵은 거절된다.
6. `floor_1`과 `floor_7`은 missing이다.
7. navigation, mapping, map_update는 마스터를 바꾸지 못한다.
8. 후보 맵은 staging 이름만 허용되고, 승격 함수는 복사하지 않는다.
9. 액션 경로는 `/pinky1/navigate_to_pose`다.
10. placeholder 그래프는 좌표 변환을 거절한다.
11. 하드웨어 goal과 hardware launch는 막힌다.
12. `mark_disconnected`는 goal을 failed로 두고, 연결이 없으면 상태는 `disconnected`다.

Gazebo 통합 시험은 이 세션에서 다시 돌리지 않았다. 표의 Gazebo 행은 이전 세션의 측정이다. 그 프로세스에 `cmd_vel`을 다시 보내지 않았다.

실기체 읽기 전용 시험은 장치가 없어 수행하지 않았다. 미실행을 통과로 적지 않았다.

## 세 가지 결론

구현이 끝난 부분:

- pinky1·pinky2·pinky3 프로필과 `ROBOT_BACKEND=gazebo|hardware` 분리
- 공통 토픽·액션 이름과 백엔드별 프레임
- 있는 맵만 읽고, 없는 1층·7층은 오류로 두는 카탈로그
- navigation / mapping / map_update 분리와 마스터 덮어쓰기 거절
- 하드웨어 goal과 driver launch의 기동 전 차단
- Fleet goal을 좌표 없이 액션 경로로만 넘기는 함수
- 위 항목의 단위 테스트 10개와 기존 58개 회귀

Gazebo에서 이미 확인된 부분:

- pinky1의 clock, odom, TF, `/pinky1/cmd_vel`, `/pinky1/scan`
- 테스트 방 맵과 AMCL, Nav2 lifecycle
- `/pinky1/navigate_to_pose` 두 번의 성공. 도착 오차는 tolerance 0.2 m 안이고 약 0.19 m다
- RMF 도메인 0과 fleet 잠금은 그 주행 동안 유지되었다

기체가 연결되어야 확인할 부분:

- `/dev/ttyAMA0` LiDAR와 `/dev/ttyAMA4` 모터가 실제로 열리는지
- namespace를 민 뒤의 `/pinky1/scan`, `/pinky1/odom`, 배터리 Float32
- prefix 없는 TF를 로봇 3대에서 어떻게 나눌지
- joint 이름과 바퀴 반지름 차이가 오돔에 주는 영향
- 하드웨어 AMCL과 Nav2 액션 서버
- 병원 1층·7층 맵 제작과 RMF graph 실측
- 물리 비상정지, 통신 단절, 저속 이동. 이건 별도 승인 뒤의 시험이다
