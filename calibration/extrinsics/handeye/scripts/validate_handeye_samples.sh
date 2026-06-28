#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${PROJECT_ROOT}" ]; then
  PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
fi

exec python3 "${PROJECT_ROOT}/calibration/extrinsics/handeye/tools/handeye_validate_samples.py" \
  --samples "${PROJECT_ROOT}/calibration/extrinsics/handeye/sessions/handeye_samples/samples.jsonl" \
  --result "${PROJECT_ROOT}/calibration/extrinsics/handeye/sessions/handeye_samples/handeye_result.yaml" \
  --output "${PROJECT_ROOT}/calibration/extrinsics/handeye/sessions/handeye_samples/base_target_validation.csv"
