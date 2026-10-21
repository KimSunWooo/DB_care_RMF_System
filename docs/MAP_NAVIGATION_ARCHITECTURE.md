# 지도 기반 자율주행

기본 주행은 만들어 둔 occupancy map을 읽고 AMCL로 위치를 추정한 뒤 Nav2로 이동한다. SLAM은 기본 모드가 아니다. 병원 1층과 7층 맵 파일은 아직 없다. 빈 맵을 만들어 두지 않았다.

## 로더

`config/maps/catalog.yaml`과 `robot_profile/maps.py`.

| map id | 상태 | 파일 | resolution | origin |
|---|---|---|---|---|
| `gazebo_test_room` | `gazebo_nav2_goal`. 주행에 사용 가능 | `sim/nav2/map/pinky1_room.yaml` | 0.05 | `[-4, -4, 0]` |
| `pinky_my_map` | 파일은 있음. 미검증 | Pinky `map/my_map.yaml` | 0.05 | `[-2.05, -1.62, 0]` |
| `pinky_lab` | 파일은 있음. 미검증 | Pinky `map/pinklab.yaml` | 0.05 | `[-0.65, -10.5, 0]` |
| `floor_1` | `missing` | 없음 | | |
| `floor_7` | `missing` | 없음 | | |

`require_navigation_map`은 `gazebo_nav2_goal` 또는 `surveyed`만 통과시킨다. `floor_1`, `floor_7`, `pinky_lab`, `pinky_my_map`은 오류다. YAML에 `image`, `resolution`, `origin`, `occupied_thresh`, `free_thresh`, `negate`가 없거나 이미지가 없으면 오류다.

Gazebo 테스트 방은 8 m, 160×160, origin `[-4, -4]`이다. 월드 `sim/gazebo/worlds/pinky1_empty.sdf`와 같은 기하이고, 병원 도면이 아니다. Pinky의 `my_map`과 `pinklab`은 다른 origin이라 이 방이나 병원 층과 같은 좌표로 쓰지 않는다.

## 층

1층과 7층은 카탈로그 항목만 있다. 맵이 생기기 전에는 층 선택 함수가 실패한다. 맵 전환은 로봇이 그 층으로 이동했다는 뜻이 아니다. 승강기 탑승과 문 상태, 도착 층 확인은 기존 elevator gateway 계약 밖에 있는 별도 확인이다. 이번 작업에서 승강기 명령은 보내지 않았다.

Navigation graph와의 정합은 `graph_alignment: placeholder` 그대로다. origin, scale, rotation을 추정해 넣지 않았다.

## NAVIGATION

`map_server`가 카탈로그 맵을 읽고 AMCL이 scan과 odom으로 `map` → odom을 낸다. Nav2는 그 위에서 경로를 만든다. 동적 장애물은 costmap에만 남고 마스터 맵 파일로 쓰지 않는다.

Gazebo에서 이미 확인한 흐름은 `sim/gazebo/nav2.launch.py`와 `sim/nav2/pinky1_gazebo.yaml`이다. 노드는 map_server, amcl, controller_server, planner_server, behavior_server, bt_navigator다. 액션은 `/pinky1/navigate_to_pose`다. `xy_goal_tolerance`는 0.2 m이고, 목표 `(1.2, 0)`과 `(0, 0)`은 status 4로 끝났으며 도착 오차는 약 0.19 m다.

하드웨어 Nav2는 Pinky `pinky_navigation/launch/bringup_launch.xml`을 가리킨다. 파라미터의 프레임은 prefix가 없고 `use_sim_time` 기본값은 false다. 이 launch는 이번 작업에서 실행하지 않았다.

## MAPPING

모드는 `navigation`, `mapping`, `map_update`뿐이다. mapping은 하드웨어 프로필의 `pinky_navigation/launch/map_building.launch.xml`을 가리킨다. 그 launch는 slam_toolbox `online_sync`이고, `mapper_params.yaml`의 mode는 `mapping`, scan은 `/scan`, base는 `base_footprint`, odom은 `odom`이다.

저장 위치는 `maps/staging/<파일 이름>`만 허용한다. 경로를 넣거나 카탈로그 맵과 같은 파일 이름이면 `ModeError`다. `master_replace_allowed`는 세 모드 모두 false다. Gazebo 프로필에는 mapping launch가 없다.

## MAP_UPDATE

`update_block_reason`은 두 맵의 resolution과 origin을 비교한다. 하나라도 `missing`이거나 좌표가 다르거나 후보가 미검증이면 이유를 반환한다. 둘이 맞아도 파일을 복사하지 않고, 승격은 수동이라고 반환한다. 롤백은 카탈로그가 이전 yaml 경로를 유지하는 방식으로 하고, 마스터를 지우는 함수는 없다.

버전은 카탈로그의 `version`, `floor_id`, `method`, `verification`, `active_master`다. DB 테이블은 추가하지 않았다. 기존 FastAPI 엔드포인트는 바꾸지 않았다.
