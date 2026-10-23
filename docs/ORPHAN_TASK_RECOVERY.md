# Orphaned RMF task

설치된 `ros-jazzy-rmf-fleet-adapter` 2.7.2에서 `TaskManager::cancel_task`는 그 프로세스의 활성 작업이나 큐에 id가 있을 때만 참이다. 참일 때만 `_handle_cancel_request`가 `task_api_responses`를 낸다. 없으면 응답이 없고, RMF-Web은 5초 뒤 `rmf service timed out`을 반환한다. 그 HTTP 응답은 task 상태를 저장하지 않는다. 저장은 fleet websocket의 `task_state_update`만 한다.

`rmf_task_dispatcher`는 `task_api_requests`에서 `dispatch_task_request`만 처리한다. 자기가 배차하지 않은 id의 ROS `cancel_task`는 `success=false`이고 API 기록을 바꾸지 않는다.

Adapter를 다시 시작해도 TaskManager 큐는 비어 있다. 복원 API는 없다. Care-Move는 그 불일치를 읽고 새 주행을 막는다. booking을 completed나 canceled로 바꾸지 않는다.

## 판정

관측은 Reconciliation worker만 한다. `GET`은 마지막 snapshot을 반환하고 샘플을 늘리지 않는다. `POST /api/admin/recovery/reconcile`이 이미 진행 중이면 같은 epoch를 다시 세지 않는다. 확인은 단조 시계로 15초 이상 떨어진 두 관측에서 fleet task id가 비어 있고 Gateway goal이 idle일 때 `OWNER_MISSING`이다. RMF timestamp가 오래됐다는 사실만으로는 확정하지 않는다. 그 전의 미종료 booking은 `PENDING_CONFIRMATION`이며 신규 주행은 즉시 막는다. 조회 실패는 `RMF_UNREACHABLE` 또는 `GATEWAY_UNREACHABLE`이다. Worker가 멈추면 snapshot이 오래된 것으로 보고 신규 주행을 막는다. 프로세스 재시작은 이전 샘플로 `OWNER_MISSING`을 복원하지 않고 다시 관측한다. 감사 분류 `ORPHANED`는 reconciliation DB에만 남고 RMF `standby`는 바꾸지 않는다.

조회:

```text
GET  /api/admin/recovery/tasks
GET  /api/admin/recovery/tasks/{booking_id}
POST /api/admin/recovery/reconcile
```

`reconcile`은 RMF와 Gateway poll을 다시 읽는다. 취소, ARM, `/cmd_vel`은 없다.

## 격리 스택

운영 domain 77, Pinky domain 24, Gazebo domain 42, API 8000, Care-Move 8011, Gateway 17453, `/run/rmf_api_server.db`와는 겹치지 않는 값만 쓴다. `app/isolated_env.py`의 `ISOLATED_STACK`은 domain 91, API 8001, Gateway 17491, DB `/tmp/care-move-isolated/rmf.sqlite`이다. 격리 스택은 이후 검증에서 domain 91로 기동했다. 그 기동이 운영 domain 77과 Pinky domain 24의 노드를 바꾸지는 않았다.

## 운영 전환은 실행하지 않음

공식 종료 경로가 없는 booking을 DB에서 지워 정상처럼 만들지 않는다. 나중에 환경을 바꿀 때는 로봇 정지, Gateway 비활성, 이전 dispatcher 정지, 새 RMF의 DB·DDS·포트 분리, fleet 등록 확인, 오래된 task가 자동 배차되지 않음을 확인한 뒤에만 한다. 되돌릴 때는 새 fleet 등록을 멈추고 기존 domain 77 API와 Gateway 17453을 그대로 둔다.
