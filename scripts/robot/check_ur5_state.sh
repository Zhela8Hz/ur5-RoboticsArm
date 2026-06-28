#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/home/z/Apps-my/.ros-log
mkdir -p "${ROS_LOG_DIR}"

echo "Checking /joint_states once..."
timeout 5s ros2 topic echo /joint_states --once || {
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
