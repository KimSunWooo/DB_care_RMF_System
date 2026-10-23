# Gazebo RMF E2E 실행 절차

swPC, Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic. 아래 순서는 2026-10-22에 실제로 완료까지 간 구성이다. 운영 도메인 0, 실제 Pinky 도메인 53, mock odom 도메인 90에는 명령을 보내지 않는다. `rviz2`가 도메인 53에서 떠 있으면 종료하지 않는다.

작업 디렉터리는 `/home/sw/DevSpace/care-move-rmf-starter`이다. Fleet adapter와 gateway는 시스템 `python3`이다. Care-Move API와 pytest만 `.venv`를 쓴다. `colcon`이나 패키지 설치는 하지 않는다.

## 환경

```bash
cd /home/sw/DevSpace/care-move-rmf-starter
source sim/gazebo/env.sh
```

`sim/gazebo/env.sh`가 정하는 값은 `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `PYTHONNOUSERSITE=1`이다. Gazebo와 gateway만 이 셸을 쓴다.

RMF 쪽 셸은 따로 만든다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONNOUSERSITE=1
```

인증은 저장소 `.env`의 기존 local JWT다. 토큰과 비밀은 출력하거나 문서에 복사하지 않는다. 인증을 끄지 않는다.

## 1. Gazebo와 Nav2 (도메인 42)

```bash
source sim/gazebo/env.sh
ros2 launch sim/gazebo/pinky1.launch.py
```

스폰이 끝난 뒤 다른 터미널에서:

```bash
source sim/gazebo/env.sh
ros2 launch sim/gazebo/nav2.launch.py
```

확인:

```bash
source sim/gazebo/env.sh
ros2 action list -t | grep navigate_to_pose
```

`/pinky1/navigate_to_pose`가 `nav2_msgs/action/NavigateToPose`로 보여야 한다.

## 2. 시뮬레이션 RMF (도메인 77)

스케줄:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run rmf_traffic_ros2 rmf_traffic_schedule --ros-args -p use_sim_time:=false
```

건물 지도:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run rmf_building_map_tools building_map_server \
  /home/sw/DevSpace/care-move-rmf-starter/config/rmf/gazebo_test_room/gazebo_test_room.building.yaml
```

GeoJSON 생성 경고는 서버가 waypoint를 publish하는 것을 막지 않는다.

API 설정 파일. 비밀은 넣지 않는다. 이미지 기본 설정 위에 ROS 인자만 덮는다.

```bash
mkdir -p /tmp/care-move-e2e
cat > /tmp/care-move-e2e/gazebo_api_config.py <<'EOF'
from api_server.default_config import config

config["ros_args"] = ["--ros-args", "-p", "use_sim_time:=false"]
config["db_url"] = "sqlite:///run/rmf_api_server.db"
config["log_level"] = "INFO"
EOF

docker run -d --name care-move-gazebo-rmf-api --network host \
  -e ROS_DOMAIN_ID=77 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e RMF_API_SERVER_CONFIG=/ws/gazebo_api_config.py \
  -v /tmp/care-move-e2e/gazebo_api_config.py:/ws/gazebo_api_config.py:ro \
  ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly
```

`AppConfig`에 `use_sim_time` 키를 넣으면 컨테이너가 바로 죽는다. 포트 8000에 다른 RMF API가 이미 있으면 이 컨테이너를 띄우지 않는다.

```bash
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8000/openapi.json
```

200이어야 한다.

## 3. Gateway와 Fleet Adapter

Nav2 action이 준비된 뒤에 gateway를 띄운다.

```bash
source sim/gazebo/env.sh
python3 scripts/gazebo_nav_gateway.py
```

`ready` JSON이 나온 뒤, RMF 셸에서:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONNOUSERSITE=1
export SIM_RMF_SERVER_URI=ws://127.0.0.1:8000/_internal
python3 scripts/gazebo_fleet_adapter.py
```

정상 기동 한 줄:

```json
{"started": true, "robots_added": 1, "fleet": "gazebo_pinky1", "robot": "pinky1", "server_uri_sent": true, "rmf_domain": 77, "nav_domain": 42}
```

`robots_added`만으로 끝내지 않는다. API에서 fleet와 로봇을 확인한다. Care-Move를 띄운 뒤의 `/api/rmf/status`로 봐도 된다.

## 4. Care-Move API

이미 설정된 `.env`를 셸에 다시 넣지 않아도 된다. `load_runtime_env`는 이미 있는 환경 변수를 덮지 않는다. 이번 실행은 아래 세 값을 프로세스 환경으로 주었다.

```bash
cd /home/sw/DevSpace/care-move-rmf-starter
CARE_MOVE_MODE=rmf \
RMF_API_URL=http://127.0.0.1:8000 \
RMF_FLEET_NAME=gazebo_pinky1 \
.venv/bin/python -m uvicorn app.main:app --host 127.0.0.1 --port 8011
```

```bash
curl -s http://127.0.0.1:8011/api/rmf/status
```

`status`가 ready이고 `fleet_names`에 `gazebo_pinky1`, `graph_waypoints`에 `GZ_EAST`와 `GZ_SPAWN`이 있어야 한다.

## 5. Task 제출과 완료 확인

로봇이 멈춰 있고 gateway goal phase가 `idle`, `succeeded`, `canceled` 중 하나일 때만 새 task를 보낸다.

`GZ_EAST`로 보내는 예:

```bash
curl -s -X POST http://127.0.0.1:8011/api/tasks \
  -H 'Content-Type: application/json' \
  -d '{"robot_id":"pinky1","places":["GZ_EAST"],"rounds":1}'
```

응답의 `rmf_task_id`를 저장한다. 그 다음 `/api/state`에서 같은 id의 `rmf_status`를 본다. `queued`와 `underway`는 완료가 아니다.

gateway 상태:

```bash
python3 - <<'PY'
import json, socket
s = socket.create_connection(("127.0.0.1", 17442), 3)
s.sendall(b'{"op":"poll"}\n')
data = b""
while b"\n" not in data:
    data += s.recv(65536)
print(data.decode())
PY
```

성공은 다음 세 가지가 같이 있을 때다.

- `goal.phase`가 `succeeded`이고 `result_confirmed`가 true
- pose가 목적지 0.2 m 안에 들어옴
- RMF `rmf_status`가 `completed`

대시보드 `http://127.0.0.1:8011/`의 임무 이력에 `completed`가 보여야 Care-Move 표시까지 확인된 것이다. 개념 지도는 Nav2 맵이 아니다.

취소는 이동이 시작된 뒤 `POST /api/tasks/{local_id}/cancel`이다. Nav2 phase가 `canceled`이고 pose가 더 이상 변하지 않으며 RMF status가 `canceled`일 때 취소 성공이다.

그래프에 없는 이름:

```bash
curl -s -o /dev/null -w '%{http_code}\n' -X POST http://127.0.0.1:8011/api/tasks \
  -H 'Content-Type: application/json' \
  -d '{"robot_id":"pinky1","places":["NOT_EXIST"],"rounds":1}'
```

400이어야 한다.

## 6. Task POST Timeout

상태 조회 timeout `RMF_API_TIMEOUT` 기본값은 5초다. Task POST read timeout `RMF_TASK_SUBMIT_TIMEOUT` 기본값은 12초다. 2026-10-23에 adapter가 응답하는 상태에서 `POST /tasks/robot_task`는 147ms, HTTP 200, `queued`였다. 5초 timeout은 그때의 정상 접수 시간이 아니라, 응답이 비었을 때의 클라이언트 한계다.

요청에는 `requester=care-move:<correlation>`과 `labels=["care-move-request=<correlation>"]`를 넣는다. RMF-Web schema에는 Idempotency-Key가 없다. POST가 timeout이면 같은 payload를 다시 POST하지 않고 `GET /tasks?requester=care-move:<correlation>`을 한 번 한다.

- label이 하나면 booking을 붙여 `SUBMISSION_ACCEPTED`로 기록한다.
- 없거나 둘 이상이면 `submission_unknown`으로 기록한다. `accepted`는 false다.
- 이후 상태 폴링이 같은 correlation을 조회해 booking이 생기면 붙인다. 그 조회는 POST가 아니다.

## 7. 종료

도메인 53의 `rviz2`와 SSH 세션은 건드리지 않는다. 이번 절차로 띄운 것만 아래로 끈다.

```bash
# adapter, gateway, uvicorn은 각 터미널에서 Ctrl-C
docker stop care-move-gazebo-rmf-api
# building_map_server, rmf_traffic_schedule, nav2, gazebo 터미널에서 Ctrl-C
```

`docker rm`은 컨테이너가 이미 `--rm` 없이 만들어져 있을 때만 필요하다. 검증 중인 goal이 있으면 gateway보다 adapter를 먼저 멈추지 말고, 먼저 task를 취소해 Nav2 status 5를 확인한다.

## 8. 실물 Pinky 단거리 task

이 절차는 2026-10-23 12:12에 한 번 실행했다. 주소는 Pinky AP `192.168.4.1`, ROS domain 24, RMW `rmw_fastrtps_cpp`다. RMF는 domain 77이다. PC Wi-Fi가 `192.168.4.4`로 바뀐 뒤에는 domain 77 Cyclone을 loopback으로 고정해야 새 adapter가 schedule을 본다.

```bash
export CYCLONEDDS_URI=file:///tmp/care-move-e2e/cyclonedds-lo.xml
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run rmf_traffic_ros2 rmf_traffic_schedule --ros-args -p use_sim_time:=false
```

API 컨테이너에도 같은 `CYCLONEDDS_URI`가 필요하다. 설정 파일은 `/tmp/care-move-e2e/cyclonedds-lo.xml`이고, peer는 `127.0.0.1`, multicast는 꺼 둔다.

실물 gateway:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=24 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONNOUSERSITE=1
export PHYSICAL_PINKY_MODE=PHYSICAL_TEST
python3 scripts/physical_pinky_gateway.py
```

poll에서 `register_with_rmf`가 true이고 `motion_commands`가 0일 때만 adapter를 띄운다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp PYTHONNOUSERSITE=1
export CYCLONEDDS_URI=file:///tmp/care-move-e2e/cyclonedds-lo.xml
export SIM_RMF_SERVER_URI=ws://127.0.0.1:8000/_internal
export PHYSICAL_FLEET_REGISTER=1
python3 scripts/physical_fleet_adapter.py
```

`GET /fleets`에 `physical_pinky`와 `real_pinky1`이 보여야 한다. 그 전에 `motion_commands`는 0이어야 한다.

```bash
curl -s -X POST http://127.0.0.1:8011/api/tasks \
  -H 'Content-Type: application/json' \
  -d '{"robot_id":"real_pinky1","places":["P_TEST"],"rounds":1}'
```

끝난 뒤에는 gateway를 `PHYSICAL_DRY_RUN`으로 다시 띄운다. `PHYSICAL_FLEET_REGISTER` 없이 adapter를 실행하면 `Adapter.make`를 호출하지 않는다.

현재 UI 경로는 위 수동 절차와 다르다. Gateway poll에 `session_protocol_version` 2가 없고, 그 로봇의 RMF booking이 `queued`/`standby`/`underway`/`blocked`이면 ARM하지 않는다. 조회 실패도 새 task를 허용하지 않는다. `POST /tasks/cancel_task`가 `rmf service timed out`이면 booking을 completed로 바꾸지 않는다.

