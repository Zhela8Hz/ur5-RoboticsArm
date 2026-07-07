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

ROBOT_IP="192.168.1.10"
REVERSE_IP="192.168.1.20"
MAX_ATTEMPTS="${UR5_DRIVER_MAX_ATTEMPTS:-3}"
CONFIG_TIMEOUT_PATTERN="Could not get configuration package within timeout"
STARTED_PATTERN="System successfully started"

launch_pid=""
tail_pid=""

cleanup() {
  if [ -n "${tail_pid}" ] && kill -0 "${tail_pid}" 2>/dev/null; then
    kill "${tail_pid}" 2>/dev/null || true
  fi
  if [ -n "${launch_pid}" ] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || kill -INT "${launch_pid}" 2>/dev/null || true
    sleep 2
    kill -TERM -- "-${launch_pid}" 2>/dev/null || kill -TERM "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

wait_for_primary_stream() {
  local attempt
  for attempt in 1 2 3 4 5; do
    if timeout 2 bash -c "dd bs=1 count=1 status=none < /dev/tcp/${ROBOT_IP}/30001 >/dev/null" 2>/dev/null; then
      return 0
    fi
    echo "Waiting for UR primary interface ${ROBOT_IP}:30001... (${attempt}/5)"
    sleep 1
  done
  echo "ERROR: UR primary interface ${ROBOT_IP}:30001 is not readable."
  return 1
}

run_launch_once() {
  local attempt_log="$1"

  setsid ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5 \
    robot_ip:="${ROBOT_IP}" \
    kinematics_params_file:="${PROJECT_ROOT}/configs/robot/ur5_actual_calibration.yaml" \
    reverse_ip:="${REVERSE_IP}" \
    launch_rviz:=false \
    controller_spawner_timeout:=30 >"${attempt_log}" 2>&1 &
  launch_pid=$!

  tail -n +1 -f --pid="${launch_pid}" "${attempt_log}" &
  tail_pid=$!

  while kill -0 "${launch_pid}" 2>/dev/null; do
    if grep -q "${STARTED_PATTERN}" "${attempt_log}"; then
      wait "${launch_pid}"
      local status=$?
      wait "${tail_pid}" 2>/dev/null || true
      launch_pid=""
      tail_pid=""
      return "${status}"
    fi

    if grep -q "${CONFIG_TIMEOUT_PATTERN}" "${attempt_log}"; then
      echo
      echo "Detected UR configuration package timeout; stopping this launch attempt."
      kill -INT -- "-${launch_pid}" 2>/dev/null || kill -INT "${launch_pid}" 2>/dev/null || true
      sleep 3
      if kill -0 "${launch_pid}" 2>/dev/null; then
        kill -TERM -- "-${launch_pid}" 2>/dev/null || kill -TERM "${launch_pid}" 2>/dev/null || true
      fi
      sleep 2
      if kill -0 "${launch_pid}" 2>/dev/null; then
        kill -KILL -- "-${launch_pid}" 2>/dev/null || kill -KILL "${launch_pid}" 2>/dev/null || true
      fi
      wait "${launch_pid}" 2>/dev/null || true
      wait "${tail_pid}" 2>/dev/null || true
      launch_pid=""
      tail_pid=""
      return 75
    fi

    sleep 0.5
  done

  wait "${launch_pid}"
  local status=$?
  wait "${tail_pid}" 2>/dev/null || true
  launch_pid=""
  tail_pid=""
  return "${status}"
}

wait_for_primary_stream

for attempt in $(seq 1 "${MAX_ATTEMPTS}"); do
  attempt_log="$(mktemp /tmp/ur5_driver_attempt.XXXXXX.log)"
  echo "Starting UR5 driver attempt ${attempt}/${MAX_ATTEMPTS}..."

  set +e
  run_launch_once "${attempt_log}"
  status=$?
  set -e

  if [ "${status}" -eq 75 ] && [ "${attempt}" -lt "${MAX_ATTEMPTS}" ]; then
    echo "Retrying UR5 driver startup after configuration timeout..."
    sleep 3
    continue
  fi

  if [ "${status}" -eq 75 ]; then
    echo "ERROR: UR5 driver failed after ${MAX_ATTEMPTS} attempts due to configuration package timeout."
    exit 1
  fi

  exit "${status}"
done
