#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/z/Apps-my/ros2_ws/install/setup.bash
export ROS_LOG_DIR=/home/z/Apps-my/.ros-log
mkdir -p "${ROS_LOG_DIR}"

session_dir="/home/z/Apps-my/calibration/extrinsics/handeye/sessions/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${session_dir}"
ln -sfn "${session_dir}" /home/z/Apps-my/calibration/extrinsics/handeye/sessions/latest

echo "New hand-eye session: ${session_dir}"
echo "Only captures with at least 21 ChArUco corners will be accepted."

exec python3 /home/z/Apps-my/calibration/extrinsics/handeye/tools/handeye_capture.py \
  --image-topic /camera/color/image_raw \
  --intrinsics /home/z/Apps-my/calibration/rgb_intrinsics/results/rgb_intrinsics_640x360.yaml \
  --output-dir "${session_dir}" \
  --samples-file "${session_dir}/samples.jsonl" \
  --base-frame base \
  --tool-frame tool0 \
  --camera-frame camera_color_optical_frame \
  --min-charuco-corners 21 \
  --use-latest-tf
