#!/usr/bin/env bash
# Shell for the virtual pinky1 Gazebo process only.
# Does not change RMF containers or the traffic schedule on domain 0.
source /opt/ros/jazzy/setup.bash
if [[ -f /home/sw/DevSpace/pinky/install/local_setup.bash ]]; then
  source /home/sw/DevSpace/pinky/install/local_setup.bash
fi
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISTRO=jazzy
export PYTHONNOUSERSITE=1

PINKY_GZ_LIB=/home/sw/DevSpace/pinky/install/pinky_gz_sim/lib
if [[ -d "${PINKY_GZ_LIB}" ]]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${PINKY_GZ_LIB}${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
fi

DESC_SHARE="$(ros2 pkg prefix pinky_description)/share"
GZ_MODELS="$(ros2 pkg prefix pinky_gz_sim)/share/pinky_gz_sim/models"
export GZ_SIM_RESOURCE_PATH="${DESC_SHARE}:${GZ_MODELS}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

echo "virtual pinky1 gazebo shell: domain=${ROS_DOMAIN_ID} rmw=${RMW_IMPLEMENTATION} gz=$(gz sim --version | head -n 1)"
