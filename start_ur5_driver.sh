#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/home/z/Apps-my/.ros-log
mkdir -p "${ROS_LOG_DIR}"

exec ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5 \
  robot_ip:=192.168.1.10 \
  kinematics_params_file:=/home/z/Apps-my/ur5_actual_calibration.yaml \
  reverse_ip:=192.168.1.20 \
  launch_rviz:=false \
  controller_spawner_timeout:=30
