#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
fi

source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${PROJECT_ROOT}/.ros-log"
mkdir -p "${ROS_LOG_DIR}"
exec python3 "${PROJECT_ROOT}/calibration/extrinsics/handeye/tools/handeye_solve.py" \
  --samples "${PROJECT_ROOT}/calibration/extrinsics/handeye/sessions/handeye_samples/samples.jsonl" \
  --output "${PROJECT_ROOT}/calibration/extrinsics/handeye/sessions/handeye_samples/handeye_result.yaml" \
  --method tsai
