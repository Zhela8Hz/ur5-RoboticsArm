#!/usr/bin/env bash
set -eo pipefail

cd /home/z/Apps-my
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
exec ros2 launch orbbec_camera dabai_dcw.launch.py \
  enable_color:=false enable_depth:=false enable_point_cloud:=false \
  enable_ir:=true ir_width:=640 ir_height:=480 ir_fps:=10 \
  enable_ldp:=false
