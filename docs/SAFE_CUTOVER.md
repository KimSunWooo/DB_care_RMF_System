# 운영 전환과 Rollback

이 문서는 절차다. 이번 작업에서 실물 Gateway의 제어권은 옮기지 않았다.

## 환경

| | Environment A | Environment B |
|---|---|---|
| ID | `rmf-a-domain77` | `rmf-b-domain91` |
| API | `127.0.0.1:8000` | `127.0.0.1:8001` |
| ROS domain | 77 | 91 |
| DB | `/tmp/care-move-e2e/rmf_api_server.db` | `/tmp/care-move-isolated/rmf.sqlite` |
| Gateway | `17453` | 사용하지 않음 |
| 역할 | 기존 booking 보존 | Mock 검증 |

Booking은 `environment id + booking id`로 구분한다. A의 `7c5e2da1-e89a-44e3-9d68-591a027c4922`는 RMF 상태 `standby`이고 Care-Move 감사 분류만 `ORPHANED`다. B의 작업이 A의 booking을 지우거나 완료로 바꾸지 않는다.

## Fencing

Care-Move API는 실물 포트 17453에 대한 ARM, prepare, 작업 제출을 lease holder `rmf-a-domain77`만 요청할 수 있게 막는다. `rmf-b-domain91`은 그 API 경로가 거부한다. 이 검사는 실행 중인 Gateway 프로세스의 검사가 아니다.

Mock Gateway `127.0.0.1:17491`는 별도다. ARM, prepare, GO는 Gateway 프로세스가 HMAC lease를 확인한 뒤에만 받아들이고, 명령 수를 올리기 직전에 한 번 더 확인한다. 이전 generation, 다른 boot id, 다른 session, 만료, 변조는 거절한다. 재시작은 새 boot id를 만들고 generation 파일을 뒤로 되돌리지 않는다. 만료되면 진행 중 goal을 멈추며 그 정지는 새 이동 명령으로 세지 않는다. 정지는 현재 lease 또는 로컬 emergency credential만 받는다.

지금 떠 있는 운영 Gateway(17453, protocol 2)는 이 lease 코드를 로드하지 않았다. 소스를 고쳤다는 이유만으로 그 프로세스가 보호되는 것은 아니다. 운영 Gateway와 운영 Adapter는 재시작하지 않았다.

Domain 24 `/cmd_vel`의 구독자는 `pinky_bringup`이고, 발행자를 구분하지 않는다. Publisher가 0인 것은 지금 명령이 없다는 관측이지 접근 통제가 아니다. SROS2나 모터 브링업 변경은 하지 않았다. 현장 담당자, 비상정지, 이동 공간을 확인할 수 없으면 실물 Gateway 교체와 실물 주행은 하지 않는다. `PHYSICAL_TEST`로 다음 Gateway를 띄울 때는 `GATEWAY_LEASE_ENFORCED=1`과 로컬 secret 파일이 없으면 기동을 거부한다. 이미 떠 있는 프로세스는 그 검사를 실행하지 않는다.

`apply_cutover()`는 조건이 모두 참이어도 `physical cutover is not executed`로 거부한다. 승인 플래그를 이 프로세스에 넣지 않았다.

## 전환 조건

다음이 모두 확인된 뒤에만 사람이 승인한다.

```text
Old Environment Fenced
No Active Physical Goal
Robot Stopped
Gateway DRY_RUN
Old Task Cannot Replay
New RMF Database Independent
New Task Lifecycle Verified
Rollback Plan Ready
Explicit Operator Approval
```

## Rollback

이미 움직인 로봇은 소프트웨어로 되돌리지 않는다. 전환을 중단할 때는 새 환경의 task 제출을 멈추고, 실물 Gateway는 `PHYSICAL_DRY_RUN`과 모터 명령 0을 확인한다. A의 DB와 booking은 그대로 둔다. B의 컨테이너와 domain 91 프로세스는 B를 띄울 때 기록한 이름과 PID만 종료한다. domain 77의 schedule, API, physical adapter, Gateway는 이 절차로 종료하지 않는다.
