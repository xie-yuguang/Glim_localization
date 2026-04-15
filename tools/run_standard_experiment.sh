#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_standard_experiment.sh \
    --bag BAG \
    --map MAP \
    --initial-pose "x y z roll pitch yaw" \
    --output-dir DIR \
    [--matching-method cpu_gicp|gpu_vgicp] \
    [--label LABEL] \
    [--interval SEC] \
    [--enhanced] \
    [--gpu-monitor] \
    [--benchmark-queries N] \
    [--benchmark-distance M] \
    [--benchmark-submaps N] \
    [--benchmark-index-resolution M]

Description:
  Standard offline experiment wrapper for glim_localization. It combines:
    1. offline localization run
    2. external resource monitoring
    3. HTML resource report generation
    4. trajectory plotting
    5. localization map query benchmark

  The script does not modify glim_localization runtime code. It only orchestrates
  existing external tools and writes experiment artifacts into OUTPUT_DIR.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

bag_path=""
map_path=""
initial_pose=""
output_dir=""
matching_method="cpu_gicp"
label=""
interval="1"
enable_enhanced=0
enable_gpu_monitor=0
benchmark_queries="1000"
benchmark_distance="40.0"
benchmark_submaps="8"
benchmark_index_resolution="20.0"

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
    --initial-pose)
      initial_pose="$2"
      shift 2
      ;;
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    --matching-method)
      matching_method="$2"
      shift 2
      ;;
    --label)
      label="$2"
      shift 2
      ;;
    --interval)
      interval="$2"
      shift 2
      ;;
    --enhanced)
      enable_enhanced=1
      shift
      ;;
    --gpu-monitor)
      enable_gpu_monitor=1
      shift
      ;;
    --benchmark-queries)
      benchmark_queries="$2"
      shift 2
      ;;
    --benchmark-distance)
      benchmark_distance="$2"
      shift 2
      ;;
    --benchmark-submaps)
      benchmark_submaps="$2"
      shift 2
      ;;
    --benchmark-index-resolution)
      benchmark_index_resolution="$2"
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

if [[ -z "$bag_path" || -z "$map_path" || -z "$initial_pose" || -z "$output_dir" ]]; then
  echo "missing required arguments" >&2
  usage >&2
  exit 1
fi

read -r init_x init_y init_z init_roll init_pitch init_yaw <<<"$initial_pose"
if [[ -z "${init_yaw:-}" ]]; then
  echo "initial pose must contain 6 values: x y z roll pitch yaw" >&2
  exit 1
fi

mkdir -p "$output_dir"
monitor_dir="$output_dir/monitor"
resource_report_dir="$output_dir/resource_report"
trajectory_dir="$output_dir/trajectory"
benchmark_dir="$output_dir/benchmark"
manifest_file="$output_dir/experiment_manifest.txt"
record_file="$output_dir/experiment_record.md"
trajectory_path="$trajectory_dir/glim_localization_traj.txt"

mkdir -p "$trajectory_dir" "$benchmark_dir"

if [[ -z "$label" ]]; then
  label="$(basename "$output_dir")"
fi

{
  echo "label=${label}"
  echo "bag_path=${bag_path}"
  echo "map_path=${map_path}"
  echo "matching_method=${matching_method}"
  echo "initial_pose=${initial_pose}"
  echo "monitor_interval_sec=${interval}"
  echo "enhanced_monitoring=${enable_enhanced}"
  echo "gpu_monitoring=${enable_gpu_monitor}"
  echo "benchmark_queries=${benchmark_queries}"
  echo "benchmark_distance=${benchmark_distance}"
  echo "benchmark_submaps=${benchmark_submaps}"
  echo "benchmark_index_resolution=${benchmark_index_resolution}"
  echo "trajectory_path=${trajectory_path}"
} > "$manifest_file"

echo "[glim_localization] running standard experiment: $label"
echo "[glim_localization] output: $output_dir"

monitor_args=(
  -o "$monitor_dir"
  -i "$interval"
  --label "$label"
)
if [[ "$enable_enhanced" -eq 1 ]]; then
  monitor_args+=(--enhanced)
fi
if [[ "$enable_gpu_monitor" -eq 1 ]]; then
  monitor_args+=(--gpu)
fi

GLIM_LOCALIZATION_MATCHING_METHOD="$matching_method" \
  "$script_dir/monitor/run_with_time.sh" \
  "${monitor_args[@]}" \
  -- \
  bash "$script_dir/run_offline_localization.sh" \
    "$bag_path" \
    "$map_path" \
    "$init_x" "$init_y" "$init_z" "$init_roll" "$init_pitch" "$init_yaw" \
    "$trajectory_path"

python3 "$script_dir/monitor/generate_usage_report.py" \
  --input-dir "$monitor_dir" \
  --output-dir "$resource_report_dir" \
  --title "$label"

python3 "$script_dir/plot_trajectory.py" \
  "$trajectory_path" \
  --plot-2d \
  --plot-3d \
  --show-arrows \
  --time-color \
  -o "$trajectory_dir/trajectory.png" \
  > "$trajectory_dir/trajectory_stats.txt"

ros2 run glim_localization benchmark_localization \
  "$map_path" \
  "$benchmark_queries" \
  "$benchmark_distance" \
  "$benchmark_submaps" \
  "$benchmark_index_resolution" \
  > "$benchmark_dir/benchmark.txt"

cat > "$record_file" <<EOF
# Experiment Record: ${label}

## Context
- Date: $(date -Iseconds)
- Matching method: ${matching_method}
- Bag: ${bag_path}
- Map: ${map_path}
- Initial pose: ${initial_pose}

## Artifacts
- Manifest: [experiment_manifest.txt]($(realpath "$manifest_file"))
- Monitor summary: [summary.txt]($(realpath "$monitor_dir/summary.txt"))
- Resource report: [resource_report.html]($(realpath "$resource_report_dir/resource_report.html"))
- Trajectory stats: [trajectory_stats.txt]($(realpath "$trajectory_dir/trajectory_stats.txt"))
- Trajectory plots: [trajectory_2d.png]($(realpath "$trajectory_dir/trajectory_2d.png")), [trajectory_3d.png]($(realpath "$trajectory_dir/trajectory_3d.png"))
- Benchmark: [benchmark.txt]($(realpath "$benchmark_dir/benchmark.txt"))

## Review Checklist
- [ ] status / diagnostics topics behaved as expected
- [ ] trajectory has no obvious discontinuity or collapse
- [ ] resource report shows acceptable CPU / memory / GPU usage
- [ ] benchmark output is within expected range
- [ ] final conclusion recorded below

## Notes
- Fill in findings, anomalies, and follow-up actions here.
EOF

echo "[glim_localization] experiment finished"
echo "[glim_localization] manifest: $manifest_file"
echo "[glim_localization] record template: $record_file"
echo "[glim_localization] resource report: $resource_report_dir/resource_report.html"
