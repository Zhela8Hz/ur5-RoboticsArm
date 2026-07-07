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

if ros2 service list | grep -qx '/camera/set_laser_enable'; then
  ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: true}'
else
  echo "Service /camera/set_laser_enable is not available in this driver session."
fi

if ros2 service list | grep -qx '/camera/set_ldp_enable'; then
  ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: false}'
else
  echo "Service /camera/set_ldp_enable is not available in this driver session."
fi
