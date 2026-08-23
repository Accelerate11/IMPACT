#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
duration_s=65
requested_result_dir=""

while (($#)); do
  case "$1" in
    --duration) duration_s="$2"; shift 2 ;;
    --result-dir) requested_result_dir="$2"; shift 2 ;;
    -h|--help) echo "Usage: $0 [--duration SECONDS] [--result-dir PATH]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "${duration_s}" =~ ^[1-9][0-9]*$ ]] && ((duration_s >= 65)) || {
  echo "P7 capture duration must be at least 65 seconds." >&2; exit 2;
}

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
results_root="${workspace_root}/experiments/results/impact_p7"
if [[ -n "${requested_result_dir}" ]]; then
  result_dir="$(realpath -m -- "${requested_result_dir}")"
else
  result_dir="${results_root}/p7_${timestamp}_$$"
fi
case "${result_dir}" in
  "${results_root}"/*) ;;
  *) echo "P7 result directory must stay below ${results_root}." >&2; exit 2 ;;
esac
mkdir -p -- "${result_dir}"

train_room="${result_dir}/train_structured_room"
train_corridor="${result_dir}/train_long_corridor"
test_room="${result_dir}/test_structured_room"
test_corridor="${result_dir}/test_long_corridor"
calibration="${result_dir}/p7-calibration.json"
summary="${result_dir}/summary.json"

bash "${script_dir}/run_p3_fast_lio.sh" \
  --scenario structured_room --duration "${duration_s}" --result-dir "${train_room}" \
  --p7-split train --trajectory-variant train
bash "${script_dir}/run_p3_fast_lio.sh" \
  --scenario long_corridor --duration "${duration_s}" --result-dir "${train_corridor}" \
  --p7-split train --trajectory-variant train

python3 "${script_dir}/calibrate_p7.py" calibrate \
  --train "${train_room}/p7-capture.json" "${train_corridor}/p7-capture.json" \
  --output "${calibration}" >"${result_dir}/calibration.log"
sha256sum "${calibration}" >"${result_dir}/calibration.before-tests.sha256"

bash "${script_dir}/run_p3_fast_lio.sh" \
  --scenario structured_room --duration "${duration_s}" --result-dir "${test_room}" \
  --p7-split test --trajectory-variant validation --calibration-file "${calibration}"
bash "${script_dir}/run_p3_fast_lio.sh" \
  --scenario long_corridor --duration "${duration_s}" --result-dir "${test_corridor}" \
  --p7-split test --trajectory-variant validation --calibration-file "${calibration}"

sha256sum "${calibration}" >"${result_dir}/calibration.after-tests.sha256"
cmp --silent "${result_dir}/calibration.before-tests.sha256" "${result_dir}/calibration.after-tests.sha256" || {
  echo "Frozen calibration artifact changed during test runs." >&2; exit 15;
}
python3 "${script_dir}/calibrate_p7.py" summarize \
  --calibration "${calibration}" \
  --test "${test_room}/p7-capture.json" "${test_corridor}/p7-capture.json" \
  --output "${summary}" | tee "${result_dir}/summary.log"

echo "PASS: P7 train-only Protection Level calibration and independent validation completed."
echo "Results: ${result_dir}"
