"""Room shared by the Gazebo world and the Nav2 map.

Boxes are axis-aligned. The map origin is the lower-left corner of the image.
A cell is occupied when its square overlaps a box footprint.
"""
from __future__ import annotations

from pathlib import Path

RESOLUTION = 0.05
ORIGIN_X = -4.0
ORIGIN_Y = -4.0
WIDTH_M = 8.0
HEIGHT_M = 8.0
# name, center x, center y, size x, size y, size z
BOXES: tuple[tuple[str, float, float, float, float, float], ...] = (
    ("wall_north", 0.0, 3.6, 7.4, 0.2, 0.8),
    ("wall_south", 0.0, -3.6, 7.4, 0.2, 0.8),
    ("wall_east", 3.6, 0.0, 0.2, 7.4, 0.8),
    ("wall_west", -3.6, 0.0, 0.2, 7.4, 0.8),
    ("obstacle", 2.0, 1.2, 0.5, 0.5, 0.6),
)
SPAWN_XY = (0.0, 0.0)
GOAL_XY = (1.2, 0.0)


def grid_size() -> tuple[int, int]:
    width = int(round(WIDTH_M / RESOLUTION))
    height = int(round(HEIGHT_M / RESOLUTION))
    return width, height


def cell_overlaps_box(ix: int, iy: int) -> bool:
    x0 = ORIGIN_X + ix * RESOLUTION
    y0 = ORIGIN_Y + iy * RESOLUTION
    x1 = x0 + RESOLUTION
    y1 = y0 + RESOLUTION
    for _name, cx, cy, sx, sy, _sz in BOXES:
        if x1 > cx - sx / 2 and x0 < cx + sx / 2 and y1 > cy - sy / 2 and y0 < cy + sy / 2:
            return True
    return False


def occupancy_rows() -> list[bytearray]:
    """Image rows, top row first. 0 is occupied, 254 is free."""
    width, height = grid_size()
    rows: list[bytearray] = []
    for image_row in range(height):
        iy = height - 1 - image_row
        row = bytearray(width)
        for ix in range(width):
            row[ix] = 0 if cell_overlaps_box(ix, iy) else 254
        rows.append(row)
    return rows


def point_occupied(x: float, y: float) -> bool:
    ix = int((x - ORIGIN_X) / RESOLUTION)
    iy = int((y - ORIGIN_Y) / RESOLUTION)
    width, height = grid_size()
    if ix < 0 or iy < 0 or ix >= width or iy >= height:
        return True
    return cell_overlaps_box(ix, iy)


def world_sdf() -> str:
    models = []
    for name, cx, cy, sx, sy, sz in BOXES:
        models.append(
            f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {sz / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""
        )
    body = "\n".join(models)
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="pinky1_empty">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{body}

    <physics type="ode">
      <real_time_update_rate>200.0</real_time_update_rate>
      <max_step_size>0.005</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
  </world>
</sdf>
"""


def write_map(directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    width, height = grid_size()
    rows = occupancy_rows()
    payload = b"".join(rows)
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    (directory / "pinky1_room.pgm").write_bytes(header + payload)
    (directory / "pinky1_room.yaml").write_text(
        "\n".join([
            "image: pinky1_room.pgm",
            "mode: trinary",
            "resolution: 0.05",
            "origin: [-4.0, -4.0, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.25",
            "",
        ]),
        encoding="utf-8",
    )


def write_world(path: Path) -> None:
    path.write_text(world_sdf(), encoding="utf-8")
