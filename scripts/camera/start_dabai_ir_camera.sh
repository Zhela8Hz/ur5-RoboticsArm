#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

cd "${PROJECT_ROOT}"
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"

IR_WIDTH="${IR_WIDTH:-1024}"
IR_HEIGHT="${IR_HEIGHT:-768}"
IR_FPS="${IR_FPS:-10}"
IR_FORMAT="${IR_FORMAT:-Y10}"
IR_AUTO_EXPOSURE="${IR_AUTO_EXPOSURE:-true}"
IR_EXPOSURE="${IR_EXPOSURE:--1}"
IR_GAIN="${IR_GAIN:--1}"
LASER_ENERGY_LEVEL="${LASER_ENERGY_LEVEL:--1}"

exec ros2 launch orbbec_camera dabai_dcw.launch.py \
  enable_color:=false enable_depth:=false enable_point_cloud:=false \
  enable_ir:=true ir_width:="${IR_WIDTH}" ir_height:="${IR_HEIGHT}" ir_fps:="${IR_FPS}" ir_format:="${IR_FORMAT}" \
  enable_ir_auto_exposure:="${IR_AUTO_EXPOSURE}" ir_exposure:="${IR_EXPOSURE}" ir_gain:="${IR_GAIN}" \
  enable_ldp:=false laser_energy_level:="${LASER_ENERGY_LEVEL}"
