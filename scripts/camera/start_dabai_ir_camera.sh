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

LEFT_IR_WIDTH="${LEFT_IR_WIDTH:-1280}"
LEFT_IR_HEIGHT="${LEFT_IR_HEIGHT:-800}"
LEFT_IR_FPS="${LEFT_IR_FPS:-30}"
LEFT_IR_FORMAT="${LEFT_IR_FORMAT:-Y8}"
RIGHT_IR_WIDTH="${RIGHT_IR_WIDTH:-1280}"
RIGHT_IR_HEIGHT="${RIGHT_IR_HEIGHT:-800}"
RIGHT_IR_FPS="${RIGHT_IR_FPS:-30}"
RIGHT_IR_FORMAT="${RIGHT_IR_FORMAT:-Y8}"
IR_AUTO_EXPOSURE="${IR_AUTO_EXPOSURE:-true}"
IR_EXPOSURE="${IR_EXPOSURE:--1}"
IR_GAIN="${IR_GAIN:--1}"
LDP_POWER_LEVEL="${LDP_POWER_LEVEL:--1}"

exec ros2 run orbbec_camera orbbec_camera_node --ros-args \
  -r __ns:=/camera \
  -p camera_name:=camera \
  -p enable_color:=false \
  -p enable_depth:=false \
  -p enable_point_cloud:=false \
  -p enable_colored_point_cloud:=false \
  -p enable_left_ir:=true \
  -p left_ir_width:="${LEFT_IR_WIDTH}" \
  -p left_ir_height:="${LEFT_IR_HEIGHT}" \
  -p left_ir_fps:="${LEFT_IR_FPS}" \
  -p left_ir_format:="${LEFT_IR_FORMAT}" \
  -p enable_right_ir:=true \
  -p right_ir_width:="${RIGHT_IR_WIDTH}" \
  -p right_ir_height:="${RIGHT_IR_HEIGHT}" \
  -p right_ir_fps:="${RIGHT_IR_FPS}" \
  -p right_ir_format:="${RIGHT_IR_FORMAT}" \
  -p enable_ir_auto_exposure:="${IR_AUTO_EXPOSURE}" \
  -p ir_exposure:="${IR_EXPOSURE}" \
  -p ir_gain:="${IR_GAIN}" \
  -p enable_ldp:=false \
  -p ldp_power_level:="${LDP_POWER_LEVEL}" \
  -p enable_accel:=false \
  -p enable_gyro:=false
