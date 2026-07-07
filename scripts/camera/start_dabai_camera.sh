#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

cd "${PROJECT_ROOT}"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${PROJECT_ROOT}/.ros-log"
mkdir -p "${ROS_LOG_DIR}"

COLOR_WIDTH="${COLOR_WIDTH:-1920}"
COLOR_HEIGHT="${COLOR_HEIGHT:-1080}"
COLOR_FPS="${COLOR_FPS:-30}"
COLOR_FORMAT="${COLOR_FORMAT:-MJPG}"

exec ros2 run orbbec_camera orbbec_camera_node --ros-args \
  -r __ns:=/camera \
  -p camera_name:=camera \
  -p enable_color:=true \
  -p color_width:="${COLOR_WIDTH}" \
  -p color_height:="${COLOR_HEIGHT}" \
  -p color_fps:="${COLOR_FPS}" \
  -p color_format:="${COLOR_FORMAT}" \
  -p enable_depth:=false \
  -p enable_point_cloud:=false \
  -p enable_colored_point_cloud:=false \
  -p enable_left_ir:=false \
  -p enable_right_ir:=false \
  -p enable_accel:=false \
  -p enable_gyro:=false
