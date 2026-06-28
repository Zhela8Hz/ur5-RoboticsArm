#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/home/z/Apps-my/.ros-log
mkdir -p "${ROS_LOG_DIR}"
exec python3 /home/z/Apps-my/calibration/extrinsics/handeye/tools/handeye_solve.py \
  --samples /home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples/samples.jsonl \
  --output /home/z/Apps-my/calibration/extrinsics/handeye/sessions/handeye_samples/handeye_result.yaml \
  --method tsai
