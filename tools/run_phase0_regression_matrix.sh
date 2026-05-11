#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_phase0_regression_matrix.sh --bag BAG --map MAP [--output-dir DIR] [--initial-pose "x y z r p y"]

Runs the Phase 0 regression matrix:
  smoke_10s
  failure_15s_guard_off
	  failure_15s_confirmation_guard
	  experimental_30s_best
	  experimental_30s_raw_topk_recovery

Bag and map paths are explicit inputs or GLIM_LOCALIZATION_REGRESSION_BAG /
GLIM_LOCALIZATION_REGRESSION_MAP environment variables. No default config file is
modified.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bag_path="${GLIM_LOCALIZATION_REGRESSION_BAG:-}"
map_path="${GLIM_LOCALIZATION_REGRESSION_MAP:-}"
output_dir="${GLIM_LOCALIZATION_REGRESSION_OUTPUT_DIR:-/tmp/glim_localization_phase0_regression}"
initial_pose="0 0 0 0 0 0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      bag_path="$2"
      shift 2
      ;;
    --map)
      map_path="$2"
      shift 2
      ;;
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    --initial-pose)
      initial_pose="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ -z "$bag_path" || -z "$map_path" ]]; then
  echo "missing --bag/--map or GLIM_LOCALIZATION_REGRESSION_BAG/GLIM_LOCALIZATION_REGRESSION_MAP" >&2
  usage >&2
  exit 1
fi

standard_script="$script_dir/run_standard_experiment.sh"
if [[ ! -f "$standard_script" ]]; then
  echo "failed to locate run_standard_experiment.sh" >&2
  exit 2
fi

mkdir -p "$output_dir"
summary="$output_dir/phase0_regression_summary.md"

run_case() {
  local name="$1"
  local duration="$2"
  local timeout_sec="$3"
  shift 3

  local case_dir="$output_dir/$name"
  echo "[phase0-regression] running $name -> $case_dir"
  set +e
  bash "$standard_script" \
    --bag "$bag_path" \
    --map "$map_path" \
    --initial-pose "$initial_pose" \
    --matching-method cpu_gicp \
    --start-offset 0 \
    --duration "$duration" \
    --timeout-sec "$timeout_sec" \
    --debug-csv \
    --output-dir "$case_dir" \
    "$@"
  local code=$?
  set -e
  echo "$code" > "$case_dir/regression_case_exit_code.txt"
}

run_case smoke_10s 10 90
run_case failure_15s_guard_off 15 90
run_case failure_15s_confirmation_guard 15 90 \
  --target-rebuild-guard \
  --target-rebuild-guard-confirmation-frames 2
run_case experimental_30s_best 30 120 \
  --target-rebuild-guard \
  --target-rebuild-guard-confirmation-frames 2 \
  --relocalization-debug \
  --smoother-guard \
  --smoother-guard-max-holds 200 \
  --lost-recovery \
  --lost-recovery-period-frames 5
run_case experimental_30s_raw_topk_recovery 30 120 \
  --target-rebuild-guard \
  --target-rebuild-guard-confirmation-frames 2 \
  --relocalization-debug \
  --relocalization-debug-topk 10 \
  --enable-verify-raw-topk \
  --verify-raw-topk-distance 0.55 \
  --verify-raw-topk-k 10 \
  --enable-recovering \
  --recovering-stable-frames 3 \
  --smoother-guard \
  --smoother-guard-max-holds 200 \
  --lost-recovery \
  --lost-recovery-period-frames 5

for name in smoke_10s failure_15s_guard_off failure_15s_confirmation_guard experimental_30s_best experimental_30s_raw_topk_recovery; do
  case_csv="$output_dir/$name/debug/localization_debug.csv"
  if [[ -f "$case_csv" && -f "$script_dir/analyze_debug_csv.py" ]]; then
    python3 "$script_dir/analyze_debug_csv.py" "$case_csv" --out "$output_dir/$name/debug_csv_analysis" >/dev/null || true
  fi
done

{
  echo "# Phase 0 Regression Summary"
  echo
  echo "- bag: \`$bag_path\`"
  echo "- map: \`$map_path\`"
  echo "- initial_pose: \`$initial_pose\`"
  echo "- output_dir: \`$output_dir\`"
  echo
  echo "| case | exit | frames | lost | rejected | max_step | suspicious | debug_csv |"
  echo "| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |"
	  for name in smoke_10s failure_15s_guard_off failure_15s_confirmation_guard experimental_30s_best experimental_30s_raw_topk_recovery; do
    case_dir="$output_dir/$name"
    exit_code="$(cat "$case_dir/regression_case_exit_code.txt" 2>/dev/null || echo "?")"
    stats="$case_dir/trajectory/trajectory_stats.txt"
    frames="$(grep -E '^frames:' "$stats" 2>/dev/null | awk '{print $2}' || true)"
    lost="$(python3 - "$case_dir/debug/localization_debug.csv" <<'PY' 2>/dev/null || true
import csv, sys
path = sys.argv[1]
with open(path, newline='', encoding='utf-8') as f:
    print(sum(1 for r in csv.DictReader(f) if r.get('state_after') == 'LOST'))
PY
)"
    rejected="$(grep -c 'scan-to-map registration rejected' "$case_dir/logs/registration_rejection_summary.txt" 2>/dev/null || true)"
    max_step="$(grep -E '^max_step_m:' "$stats" 2>/dev/null | awk '{print $2}' || true)"
    suspicious="$(grep -E '^suspicious_divergence:' "$stats" 2>/dev/null | awk '{print $2}' || true)"
    echo "| $name | $exit_code | ${frames:-} | ${lost:-0} | ${rejected:-0} | ${max_step:-} | ${suspicious:-} | \`$case_dir/debug/localization_debug.csv\` |"
  done
} > "$summary"

echo "[phase0-regression] wrote $summary"
