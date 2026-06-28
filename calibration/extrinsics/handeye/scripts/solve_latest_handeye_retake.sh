#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
fi

session_dir="${PROJECT_ROOT}/calibration/extrinsics/handeye/sessions/latest"
test -f "${session_dir}/samples.jsonl"

python3 "${PROJECT_ROOT}/calibration/extrinsics/handeye/tools/handeye_solve.py" \
  --samples "${session_dir}/samples.jsonl" \
  --output "${session_dir}/handeye_result.yaml" \
  --method tsai

python3 "${PROJECT_ROOT}/calibration/extrinsics/handeye/tools/handeye_validate_samples.py" \
  --samples "${session_dir}/samples.jsonl" \
  --result "${session_dir}/handeye_result.yaml" \
  --output "${session_dir}/base_target_validation.csv"

echo
echo "Result: ${session_dir}/handeye_result.yaml"
echo "Validation CSV: ${session_dir}/base_target_validation.csv"
