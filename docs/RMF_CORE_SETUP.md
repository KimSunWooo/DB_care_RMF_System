# RMF Core

2026-10-21 호스트 조회 기준이다. 기존 `care-rmf-api`, `care-rmf-dashboard` 컨테이너는 종료하지 않았다.

| 구성요소 | 상태 |
|---|---|
| `rmf_api_server` | 실행 중. `use_sim_time=true`. `GET /time`은 `0` |
| `/clock` | 토픽은 있고 발행자는 0 |
| `rmf_traffic_schedule` | 없음이 확인되어 `use_sim_time:=false`로 시작. 노드 이름은 `rmf_traffic_schedule_node_<uuid>` |
| `rmf_task_dispatcher` | 실행하지 않음. 작업 접수는 API 서버의 `POST /tasks/*` |
| `rmf_demos` | `ros2 pkg list`에 없음 |
| `building_map_generator` | 시스템 NumPy 2와 shapely의 NumPy 1 확장 충돌로 실행 실패 |
| Nav2 / Isaac Sim / Gazebo | 프로세스 없음. 액션 서버 없음 |

스케줄 노드를 다시 띄울 때:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run rmf_traffic_ros2 rmf_traffic_schedule --ros-args -p use_sim_time:=false
```

API 서버와 스케줄의 시간 기준이 다르다. `/clock`이 진행되기 전에는 이 조합으로 주행 완료를 판단하지 않는다.

설치된 어댑터 패키지는 `rmf_fleet_adapter_python` 2.7.2이고 Python 클래스는 `rmf_adapter.EasyFullControl`이다. `rmf_core`라는 실행 파일은 없다.
