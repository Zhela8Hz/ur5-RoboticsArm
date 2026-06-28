#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/z/Apps-my/ros2_ws/install/setup.bash
export ROS_LOG_DIR=/home/z/Apps-my/.ros-log
mkdir -p "${ROS_LOG_DIR}"

exec python3 /home/z/Apps-my/calibration/extrinsics/handeye/tools/handeye_capture.py \
  --image-topic /camera/color/image_raw \
  --intrinsics /home/z/Apps-my/calibration/rgb_intrinsics/results/rgb_intrinsics_640x360.yaml \
  --output-dir /home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples \
  --base-frame base \
  --tool-frame tool0 \
  --camera-frame camera_color_optical_frame \
  --use-latest-tf
