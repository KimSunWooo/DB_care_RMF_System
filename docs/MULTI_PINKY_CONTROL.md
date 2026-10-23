# 다중 Pinky 관제 기반

이 문서는 2026-10-23 구현 상태다. 실물 추가 주행, 180도 회전, Pinky 2대 이상 주행, Nav2 지도 주행은 실행하지 않았다.

## 모드 전환

Gateway 프로세스는 하나다. `127.0.0.1:17453`가 이미 쓰이고 있으면 새 프로세스는 기동을 거부한다.

버튼은 그 프로세스에 `arm`을 보낸다. 선택한 Pinky만 `PHYSICAL_DRY_RUN`에서 `PHYSICAL_TEST`로 바뀌고, 그 로봇의 `cmd_vel` publisher만 생긴다. 그 세션이 `go`를 수락한 뒤 시험이 멈추고 속도가 0이면 그 Pinky만 다시 `PHYSICAL_DRY_RUN`이 된다. 이전 목표의 `succeeded`만으로는 새 세션을 끝내지 않는다. 멈추지 않았으면 `FAULT`이며, Dry-run 복귀로 표시하지 않는다.

RMF fleet 이름은 `physical_pinky`다. 로봇 ID `real_pinky1`과 다르다. 주행 ROS domain은 24다. 예전에 Dry-run 판정에 남아 있던 domain 53 문구는 domain 24로 고쳐 두었다. `fleet_adapter/real_gate.py`의 Nav2 domain 53 거부는 별도 차단이며, 이번 5cm 경로가 아니다.

## 로봇

| ID | 상태 기준 | Topic |
|---|---|---|
| real_pinky1 | domain 24에서 `/odom` publisher가 있을 때만 available | `/odom`, `/cmd_vel` |
| real_pinky2 | `/pinky2/odom` publisher가 있을 때만 available | `/pinky2/odom`, `/pinky2/cmd_vel` |
| real_pinky3 | `/pinky3/odom` publisher가 있을 때만 available | `/pinky3/odom`, `/pinky3/cmd_vel` |

RMF에 남아 있는 fleet 캐시만으로 Online을 만들지 않는다.

## 작업 대기열

SQLite `data/task_queue.sqlite`. 재시작 시 `dispatch_allowed`인 진행 중 작업은 `INTERRUPTED`가 되고 다시 제출하지 않는다. 외부 운송 요청은 `dispatch_allowed`가 아니므로 RMF에 올라가지 않는다.

## 지도와 스케줄

대시보드 waypoint는 UI 개념도다. Gazebo 방 지도와 `physical_odom_trial`은 시설 지도가 아니다. `physical_navigation_ready`는 false다.

`rmf_visualization_schedule` 패키지는 설치되어 있다. WebSocket 서버가 없으면 궤적을 만들지 않고 빈 목록을 보여 준다. Schedule node를 추가로 띄우지 않는다.

## 외부 요청

`POST /api/v1/transport/requests`는 `Authorization: Bearer`와 `CARE_MOVE_OPERATOR_TOKEN`이 일치할 때만 받는다. `MOVE`, `CALL`, `ONE_WAY`, `ROUND_TRIP`을 검증해 대기열에 넣는다. 로봇이 출발지에 있다고 가정하지 않는다. RMF booking은 만들지 않는다. 로컬 시험 UI를 켜도 이 인증은 빠지지 않는다.

로컬 시험 UI는 `CARE_MOVE_ENV=local`과 `CARE_MOVE_LOCAL_TEST_UI_ENABLED=true`가 둘 다 있을 때만 켜진다. 하나라도 없으면 실물 구동 HTTP는 운영자 토큰이 필요하다. 켜진 경우에도 요청은 127.0.0.1의 Care-Move 화면에서만 받는다. 다른 Origin, 다른 Host, 프록시 헤더는 거절한다. 이 검사는 로그인 대체가 아니다. 로컬 화면에는 토큰 입력창이 없다.

복구 불가능한 `go` 거부는 `execution.finished()`로 끝내지 않는다. Adapter가 RMF `cancel_task`를 한 번 요청한다. 이미 움직이는 목표와 중복 목표는 취소하지 않는다. 그 취소 코드는 다음 Adapter 기동부터 적용된다.

## 미종료 Booking과 Gateway 프로토콜

새 5cm·180도 요청은 ARM 전에 다음을 확인한다. 하나라도 실패하면 ARM하지 않는다.

- 실행 중인 Gateway poll의 `session_protocol_version`이 2이고, `supported_test_kinds`에 `forward_5cm`와 `rotate_180`이 있다. 이 값은 프로세스가 모듈을 읽을 때 고정된다. 디스크의 소스를 다시 읽어 실행 중 프로세스의 버전으로 쓰지 않는다.
- RMF `GET /tasks?status=queued,standby,underway,blocked`에서 그 로봇에 배정될 수 있는 미종료 booking이 없다. `real_pinky1`의 booking은 `real_pinky2`를 막지 않는다. 조회가 실패하면 작업이 없는 것으로 보지 않고 그 요청을 거절한다.
- 완료는 같은 session, `go` 수락, 같은 goal generation, 목표 phase, fresh odom, 정지가 맞을 때만이다. `go` 전에는 disarm하지 않는다. 60초가 지나도 움직이고 있으면 disarm하지 않는다.

`7c5e2da1-e89a-44e3-9d68-591a027c4922`는 `standby`다. `POST /tasks/cancel_task`는 `task_api_requests`에 올리고 `task_api_responses`를 5초 기다리다가 `rmf service timed out`이다. Fleet 상태는 idle이고 task id는 비어 있다. completed로 바꾸지 않았고 DB를 지우지 않았다. 이 booking이 남아 있으면 Pinky #1 시험 버튼은 `RECOVERY_REQUIRED`다. 소유권 비교와 격리 스택은 `docs/ORPHAN_TASK_RECOVERY.md`에 있다.

다음 실물 5cm는 현장 비상정지와 관찰이 있는 뒤에만 한다. 소프트웨어 조건은 그 booking이 종료 상태이고, Gateway 프로토콜이 2이며, 모드가 `PHYSICAL_DRY_RUN`이고, 모터 ready, odom fresh, 정지, 활성 goal이 없는 것이다. 그 한 건이 RMF 종료와 정지와 Dry-run 복귀까지 끝난 뒤에만 180도를 따로 한다.

pytest의 외부 요청은 임시 SQLite에 기록한다. 운영 `data/task_queue.sqlite`에 테스트 행을 넣지 않는다.

## 검증 구분

| 항목 | 결과 |
|---|---|
| 모드 상태 기계, 로봇별 격리, FIFO, 재시작 미재실행 | 단위 테스트 |
| 외부 요청 401, 층 불일치, offline 거부, booking 없음 | 단위 테스트 |
| 지도 미정합, 빈 schedule | 단위 테스트 |
| 두 번째 Gateway의 포트 거부 | 구현 후 프로세스 확인 |
| UI에서 5cm 또는 180도를 눌러 모드가 바뀌는 실주행 | NOT_RUN |
| Pinky 2대 이상 실물 | NOT_RUN |
| Nav2 지도 배포와 자율주행 | NOT_RUN |
