# 엘리베이터 연동 전략 (학습용 설계 계약)

## 결론

- **관리자 브라우저 ↔ Care-Move 서버**: HTTPS REST (명령) + WSS (상태 구독).
- **Care-Move 서버 ↔ Open-RMF**: RMF-Web API (작업 요청·조회). 실제 lift 이동은 RMF의 리프트 연동 계층이 제어.
- **Open-RMF ↔ Lift Adapter**: ROS 2 `rmf_lift_msgs/msg/LiftRequest`와 `LiftState` 메시지.
- **Lift Adapter ↔ Elevator Gateway**: HTTP/HTTPS (제어 명령/확인) + WebSocket (상태 push) 또는 제조사가 제공하는 전용 인터페이스.
- **Elevator Gateway ↔ 실제 엘리베이터 제어기**: 해당 제조사의 공식·승인된 제어 인터페이스에 따름 (Serial/RS-485/Modbus TCP/PLC/API 등). 특정 장비 정보 없이 프로토콜을 단정하지 않는다.
- **Isaac Sim 시뮬레이션**: Gateway 대신 Mock Lift Adapter 또는 ROS 2/시뮬레이터 브리지로 상태를 에뮬레이션. 이 예제의 `/api/elevator/mock-travel`은 **그 어떤 실제 장비와도 연결되지 않는다**.

> 중요: 애플리케이션이 자체적으로 리프트 호출을 보내는 방식과 RMF가 리프트 세션을 관리하는 방식이 동시에 활성화되면 충돌할 수 있다. 실제 RMF 모드에서는 웹의 직접 승강기 버튼을 막고, RMF에서 lift 제어권을 갖도록 한다.

## HTTP 계약 제안 (미구현, 추후 Elevator Gateway 구현 시 적용)

### 명령 요청

`POST /v1/lifts/LIFT_A/trips` (Gateway 주소, FMS/RMF Lift Adapter에서 호출)

```json
{
  "command_id": "1b8c0e3f-d538-43dc-91ed-3c3fbbfa5c12",
  "robot_id": "pinky1",
  "source_floor": "1F",
  "target_floor": "7F",
  "requested_by": "rmf-lift-adapter",
  "session_id": "rmf-session-unique-id"
}
```

**응답 `202 Accepted`는 접수일 뿐 이동이나 탑승 성공을 보장하지 않는다.** `409 Conflict`는 자원 점유, `422`는 규격 오류, `503`은 컨트롤러 연결 불가에 사용한다. `command_id`는 재전송 시 중복 실행하지 않도록 idempotency key로 사용한다.

### 상태 이벤트 (WSS 또는 `GET /v1/lifts/LIFT_A`)

```json
{
  "lift_id": "LIFT_A",
  "event_seq": 42,
  "session_id": "rmf-session-unique-id",
  "current_floor": "1F",
  "target_floor": "7F",
  "moving": false,
  "door_state": "open",
  "occupied": true,
  "fault": null,
  "timestamp": "2026-10-20T08:00:00Z"
}
```

최초 연결과 재연결 때는 스냅샷 GET을 수행하고, 이후 WebSocket의 순번·타임스탬프를 검증한다. WS 끊김/상태 지연 시 **unknown**으로 표시하고 리프트 진입을 허가하지 않는다.

## 1F ↔ 7F 절차

1. Robot `pinky1`이 `N_EV1` 웨이포인트까지 이동. RMF는 nav graph의 리프트 구간과 Lift Adapter를 이용해 리프트 예약/요청.
2. Gateway가 리프트의 현재 위치, 문 열림 상태 및 장애 여부를 보고. 단순 API `202`만으로 승차 불가.
3. 승강기 진입 허가 및 로봇의 실제 탑승 확인은 Lift Adapter/내비게이션 구성 및 센서 피드백으로 수행.
4. 문 닫힘, 층 이동, 도착, 문 열림이 **승강기 상태 피드백**으로 확인됨.
5. `7F` 맵으로 재위치추정(localization)하고 `N_EV7` 하차 후 `N_GYM`까지 Nav2 주행.
6. 비상정지, 화재모드, 문 막힘, 네트워크 단절, 사람 탑승, 수동 개입 등은 실패·대기 상태로 분기.

QR 시연을 사용할 경우 QR 스캔 이벤트는 **로봇/세션 식별**로만 사용한다. 서명/만료시간/재사용 방지 등 검증 전에는 QR을 제어 명령으로 신뢰하면 안 된다. QR 스캔만으로 문 개방·이동 완료를 간주하지 않는다.

## Open-RMF 연동 시 필요한 ROS 2 메시지

```bash
ros2 interface show rmf_lift_msgs/msg/LiftRequest
ros2 interface show rmf_lift_msgs/msg/LiftState
ros2 topic list | grep -E 'lift_(requests|states)'
```

예시로 `/lift_requests`, `/lift_states`가 사용될 수 있으나 **실제 토픽 이름은 배포/리매핑 설정을 확인**한다. RMF가 session ID와 층·문 상태를 관리하므로 Gateway는 동일 세션의 피드백을 무결하게 전파해야 한다. 실제 하드웨어 액세스 권한·인터락·법적 검토는 이 교육용 프로젝트 밖의 범위다.

## 이번 Starter 구현 범위

- 구현됨: Mock 리프트 1대 직렬화 상태 시뮬레이션, 대시보드 실시간 표시, Mock 수동 층 이동.
- 계약 스텁: `app/elevator.py`의 `ElevatorPhase` (`IDLE`, `REQUESTED`, `MOVING`, `ARRIVED`, `DOOR_OPEN`, `BOARDING`, `ERROR`)와 `build_trip_preview()`. `transmit`은 항상 false다. `POST /api/elevator/hardware`는 403으로 거절한다.
- 미구현: `rmf_lift_msgs` ROS 2 bridge, 실제 HTTP Gateway, Isaac Sim 엘리베이터 물리 연동, 시리얼 장비 제어, 실제 탑승 확인.
- 확인 플래그는 탑승 허가, 문 닫힘, 목적층 도착, 하차를 분리한다. QR 인식은 이 플래그를 대체하지 않는다.

참조: https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg
