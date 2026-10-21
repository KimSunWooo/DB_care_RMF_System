#!/usr/bin/env bash
# Shell for the virtual pinky1 only. Does not change the RMF containers on domain 0.
source /opt/ros/jazzy/setup.bash
if [[ -f /home/sw/DevSpace/pinky/install/local_setup.bash ]]; then
  source /home/sw/DevSpace/pinky/install/local_setup.bash
fi
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISTRO=jazzy
export ISAAC_SIM_ROOT=/home/sw/isaacsim
export PYTHONNOUSERSITE=1
echo "virtual pinky1 shell: domain=${ROS_DOMAIN_ID} rmw=${RMW_IMPLEMENTATION} python=$(command -v python3)"
