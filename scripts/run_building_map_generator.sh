#!/usr/bin/env bash
# Run the system RMF map tool without ~/.local NumPy 2 shadowing python3-shapely.
set -euo pipefail
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
exec ros2 run rmf_building_map_tools building_map_generator "$@"
