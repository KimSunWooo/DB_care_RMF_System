# Navigation Graph

`config/waypoints.json`의 x, y는 화면 비율이다. 그 값을 미터로 바꾸지 않았다.

`config/rmf/care_move_placeholder.nav.yaml`은 파서 확인용이다.

- 레벨 `L1`
- `N_DOCK1` `(0.0, 0.0)`, charger
- `N_ROOM102` `(5.0, 0.0)`
- 양방향 lane 하나

이 5 m는 시험용 간격이다. Nav2 map, Isaac Sim, 건물 실측과 연결되어 있지 않다. `care_move.graph_alignment`는 `placeholder`이고 `allow_motion`은 false다.

설치된 `rmf_adapter.graph.parse_graph` 결과: waypoint 2, lane 2, 이름 `N_DOCK1`, `N_ROOM102`. 그래프에 lift/door 항목이 없어 어댑터가 경고를 낸다. `building_map_generator`는 NumPy 충돌로 공식 `door_madness.building.yaml`을 변환하지 못했다.

`GET /building_map`은 404다. 경로가 OpenAPI에 있고 지도 데이터는 없다. 이 파일은 `/map`으로 발행하지 않았다.
