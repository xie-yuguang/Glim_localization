#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_smoke_baseline_10s.sh --bag BAG --map MAP [--output-dir DIR] [--initial-pose "x y z roll pitch yaw"]

Description:
  Runs the current 10 second CPU smoke baseline. This is a short regression
  smoke test, not a complete localization benchmark.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

bag_path="${GLIM_LOCALIZATION_SMOKE_BAG:-}"
map_path="${GLIM_LOCALIZATION_SMOKE_MAP:-}"
output_dir="/tmp/glim_localization_smoke_10s"
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
  echo "missing --bag/--map or GLIM_LOCALIZATION_SMOKE_BAG/GLIM_LOCALIZATION_SMOKE_MAP" >&2
  usage >&2
  exit 1
fi

bash "$script_dir/run_standard_experiment.sh" \
  --bag "$bag_path" \
  --map "$map_path" \
  --initial-pose "$initial_pose" \
  --matching-method cpu_gicp \
  --start-offset 0 \
  --duration 10 \
  --timeout-sec 90 \
  --output-dir "$output_dir"
