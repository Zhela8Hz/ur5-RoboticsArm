#!/usr/bin/env bash
set -eo pipefail

cd /home/z/Apps-my
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
exec ros2 launch orbbec_camera dabai_dcw.launch.py \
  color_width:=640 color_height:=360 color_fps:=10
