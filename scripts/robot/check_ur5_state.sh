#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${PROJECT_ROOT}/.ros-log"
mkdir -p "${ROS_LOG_DIR}"

echo "Checking /joint_states once..."
python3 - <<'PY' || {
import sys

import rclpy
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node('check_ur5_joint_states_once')
received = []

def callback(msg):
    if msg.name and msg.position:
        print('JointState names:', ', '.join(msg.name))
        print('JointState positions:', ', '.join(f'{v:.6f}' for v in msg.position))
        received.append(True)

node.create_subscription(JointState, '/joint_states', callback, 10)
end_time = node.get_clock().now().nanoseconds + 5_000_000_000
while rclpy.ok() and not received and node.get_clock().now().nanoseconds < end_time:
    rclpy.spin_once(node, timeout_sec=0.2)

node.destroy_node()
rclpy.shutdown()
sys.exit(0 if received else 1)
PY
  echo
  echo "ERROR: /joint_states has no data yet."
  echo "Check UR driver terminal, robot power/brake, and teach pendant External Control Play state."
  exit 1
}

echo
echo "Checking TF base -> tool0..."
tmp_output="$(mktemp /tmp/ur5_tf_check.XXXXXX)"
set +e
timeout 5s ros2 run tf2_ros tf2_echo base tool0 >"${tmp_output}" 2>&1
tf_status=$?
set -e
cat "${tmp_output}"
if grep -q "Translation:" "${tmp_output}"; then
  rm -f "${tmp_output}"
  echo
  echo "UR5 state and TF look ready."
  exit 0
fi
rm -f "${tmp_output}"
if [ "${tf_status}" -ne 0 ]; then
  echo
  echo "ERROR: TF base -> tool0 is not available."
  echo "If robot frames use different names, run: ros2 topic echo /tf_static --once"
  exit 1
fi
