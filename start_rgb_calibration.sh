#!/usr/bin/env bash
set -eo pipefail

source /home/z/ros2_ws/setup_charuco_calibration.bash
exec ros2 run charuco_camera_calibration charuco_intrinsics --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p squares_x:=6 -p squares_y:=6 \
  -p square_length_m:=0.025 \
  -p marker_length_m:=0.018 \
  -p dictionary_id:=DICT_6X6_1000 \
  -p start_id:=233 \
  -p output_file:=/home/z/Apps-my/rgb_intrinsics_640x360.yaml
