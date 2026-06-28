#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/z/Apps-my/ros2_ws/install/setup.bash
export ROS_LOG_DIR=/home/z/Apps-my/.ros-log
mkdir -p "${ROS_LOG_DIR}"

exec python3 /home/z/Apps-my/handeye_live_validate.py \
  --image-topic /camera/color/image_raw \
  --intrinsics /home/z/Apps-my/rgb_intrinsics_640x360.yaml \
  --handeye /home/z/Apps-my/handeye_sessions/latest/handeye_result.yaml \
  --base-frame base \
  --tool-frame tool0
