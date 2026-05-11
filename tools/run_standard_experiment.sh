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
    [--start-offset SEC] \
    [--duration SEC] \
    [--max-frames N] \
    [--timeout-sec SEC] \
    [--debug-csv] \
    [--relocalization-debug] \
    [--relocalization-debug-topk N] \
	    [--verify-rejected-topk] \
	    [--verify-rejected-topk-k N] \
	    [--enable-verify-raw-topk] \
	    [--verify-raw-topk-distance D] \
	    [--verify-raw-topk-k N] \
	    [--enable-recovering] \
	    [--recovering-stable-frames N] \
	    [--smoother-guard] \
    [--smoother-guard-max-holds N] \
    [--target-rebuild-guard] \
    [--target-rebuild-guard-confirmation-frames N] \
    [--lost-recovery] \
    [--lost-recovery-period-frames N] \
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
start_offset=""
duration=""
max_frames=""
timeout_sec=""
debug_csv_enable=0
relocalization_debug_enable=0
relocalization_debug_topk="10"
relocalization_debug_verify_rejected_topk=0
relocalization_debug_verify_rejected_topk_k="5"
verify_raw_topk_enable=0
verify_raw_topk_distance="0.55"
verify_raw_topk_k="10"
recovering_enable=0
recovering_stable_frames="3"
smoother_guard_enable=0
smoother_guard_max_holds="30"
target_rebuild_guard_enable=0
target_rebuild_guard_ratio="0.8"
target_rebuild_guard_confirmation_frames="2"
target_rebuild_guard_consistency_translation="1.0"
target_rebuild_guard_consistency_angle="0.3"
target_rebuild_guard_cooldown_frames="3"
lost_recovery_enable=0
lost_recovery_period_frames="5"
lost_recovery_stable_frames="3"
lost_recovery_max_lost_before_pause="3"
lost_recovery_write_trajectory_while_lost=0

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
    --start-offset)
      start_offset="$2"
      shift 2
      ;;
    --duration|--playback-duration)
      duration="$2"
      shift 2
      ;;
    --max-frames)
      max_frames="$2"
      shift 2
      ;;
    --timeout-sec)
      timeout_sec="$2"
      shift 2
      ;;
    --debug-csv)
      debug_csv_enable=1
      shift
      ;;
    --relocalization-debug)
      relocalization_debug_enable=1
      shift
      ;;
    --relocalization-debug-topk)
      relocalization_debug_topk="$2"
      shift 2
      ;;
    --verify-rejected-topk)
      relocalization_debug_verify_rejected_topk=1
      relocalization_debug_enable=1
      debug_csv_enable=1
      shift
      ;;
	    --verify-rejected-topk-k)
	      relocalization_debug_verify_rejected_topk_k="$2"
	      shift 2
	      ;;
	    --enable-verify-raw-topk)
	      verify_raw_topk_enable=1
	      relocalization_debug_enable=1
	      debug_csv_enable=1
	      shift
	      ;;
	    --verify-raw-topk-distance)
	      verify_raw_topk_distance="$2"
	      shift 2
	      ;;
	    --verify-raw-topk-k)
	      verify_raw_topk_k="$2"
	      relocalization_debug_topk="$2"
	      shift 2
	      ;;
	    --enable-recovering)
	      recovering_enable=1
	      shift
	      ;;
	    --recovering-stable-frames)
	      recovering_stable_frames="$2"
	      shift 2
	      ;;
	    --smoother-guard)
      smoother_guard_enable=1
      shift
      ;;
    --smoother-guard-max-holds)
      smoother_guard_max_holds="$2"
      shift 2
      ;;
    --target-rebuild-guard)
      target_rebuild_guard_enable=1
      shift
      ;;
    --target-rebuild-guard-ratio)
      target_rebuild_guard_ratio="$2"
      shift 2
      ;;
    --target-rebuild-guard-confirmation-frames)
      target_rebuild_guard_confirmation_frames="$2"
      shift 2
      ;;
    --target-rebuild-guard-consistency-translation)
      target_rebuild_guard_consistency_translation="$2"
      shift 2
      ;;
    --target-rebuild-guard-consistency-angle)
      target_rebuild_guard_consistency_angle="$2"
      shift 2
      ;;
    --target-rebuild-guard-cooldown-frames)
      target_rebuild_guard_cooldown_frames="$2"
      shift 2
      ;;
    --lost-recovery)
      lost_recovery_enable=1
      shift
      ;;
    --lost-recovery-period-frames)
      lost_recovery_period_frames="$2"
      shift 2
      ;;
    --lost-recovery-stable-frames)
      lost_recovery_stable_frames="$2"
      shift 2
      ;;
    --lost-recovery-max-lost-before-pause)
      lost_recovery_max_lost_before_pause="$2"
      shift 2
      ;;
    --lost-recovery-write-trajectory-while-lost)
      lost_recovery_write_trajectory_while_lost=1
      shift
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
logs_dir="$output_dir/logs"
debug_dir="$output_dir/debug"
manifest_file="$output_dir/experiment_manifest.txt"
record_file="$output_dir/experiment_record.md"
failure_summary_file="$output_dir/failure_summary.md"
trajectory_path="$trajectory_dir/glim_localization_traj.txt"

mkdir -p "$trajectory_dir" "$benchmark_dir" "$logs_dir" "$debug_dir"
debug_csv_path="$debug_dir/localization_debug.csv"

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
  echo "start_offset=${start_offset}"
  echo "duration=${duration}"
  echo "max_frames=${max_frames}"
  echo "timeout_sec=${timeout_sec}"
  echo "debug_csv_enable=${debug_csv_enable}"
  echo "debug_csv_path=${debug_csv_path}"
	  echo "relocalization_debug_enable=${relocalization_debug_enable}"
	  echo "relocalization_debug_topk=${relocalization_debug_topk}"
	  echo "relocalization_debug_verify_rejected_topk=${relocalization_debug_verify_rejected_topk}"
	  echo "relocalization_debug_verify_rejected_topk_k=${relocalization_debug_verify_rejected_topk_k}"
	  echo "verify_raw_topk_enable=${verify_raw_topk_enable}"
	  echo "verify_raw_topk_distance=${verify_raw_topk_distance}"
	  echo "verify_raw_topk_k=${verify_raw_topk_k}"
	  echo "recovering_enable=${recovering_enable}"
	  echo "recovering_stable_frames=${recovering_stable_frames}"
	  echo "smoother_guard_enable=${smoother_guard_enable}"
  echo "smoother_guard_max_holds=${smoother_guard_max_holds}"
  echo "target_rebuild_guard_enable=${target_rebuild_guard_enable}"
  echo "target_rebuild_guard_ratio=${target_rebuild_guard_ratio}"
  echo "target_rebuild_guard_confirmation_frames=${target_rebuild_guard_confirmation_frames}"
  echo "target_rebuild_guard_consistency_translation=${target_rebuild_guard_consistency_translation}"
  echo "target_rebuild_guard_consistency_angle=${target_rebuild_guard_consistency_angle}"
  echo "target_rebuild_guard_cooldown_frames=${target_rebuild_guard_cooldown_frames}"
  echo "lost_recovery_enable=${lost_recovery_enable}"
  echo "lost_recovery_period_frames=${lost_recovery_period_frames}"
  echo "lost_recovery_stable_frames=${lost_recovery_stable_frames}"
  echo "lost_recovery_max_lost_before_pause=${lost_recovery_max_lost_before_pause}"
  echo "lost_recovery_write_trajectory_while_lost=${lost_recovery_write_trajectory_while_lost}"
  echo "trajectory_path=${trajectory_path}"
} > "$manifest_file"

echo "[glim_localization] running standard experiment: $label"
echo "[glim_localization] output: $output_dir"

monitor_script="$script_dir/monitor/run_with_time.sh"
if [[ ! -f "$monitor_script" ]]; then
  monitor_script="$script_dir/run_with_time.sh"
fi
if [[ ! -f "$monitor_script" ]]; then
  echo "failed to locate run_with_time.sh next to source or installed script layout" >&2
  exit 2
fi
monitor_tool_dir="$(dirname "$monitor_script")"
generate_report_script="$monitor_tool_dir/generate_usage_report.py"
if [[ ! -f "$generate_report_script" && -f "$script_dir/monitor/generate_usage_report.py" ]]; then
  generate_report_script="$script_dir/monitor/generate_usage_report.py"
fi
if [[ ! -f "$generate_report_script" ]]; then
  echo "failed to locate generate_usage_report.py" >&2
  exit 2
fi

plot_trajectory_script="$script_dir/plot_trajectory.py"
if [[ ! -f "$plot_trajectory_script" ]]; then
  echo "failed to locate plot_trajectory.py" >&2
  exit 2
fi

offline_script="$script_dir/run_offline_localization.sh"
if [[ ! -f "$offline_script" ]]; then
  echo "failed to locate run_offline_localization.sh" >&2
  exit 2
fi

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

offline_args=(
  "$bag_path"
  "$map_path"
  "$init_x" "$init_y" "$init_z" "$init_roll" "$init_pitch" "$init_yaw"
  "$trajectory_path"
)
if [[ -n "$start_offset" ]]; then
  offline_args+=(--start-offset "$start_offset")
fi
if [[ -n "$duration" ]]; then
  offline_args+=(--duration "$duration")
fi
if [[ -n "$max_frames" ]]; then
  offline_args+=(--max-frames "$max_frames")
fi
if [[ -n "$timeout_sec" ]]; then
  offline_args+=(--timeout-sec "$timeout_sec")
fi

run_exit_code=0
set +e
GLIM_LOCALIZATION_MATCHING_METHOD="$matching_method" \
  GLIM_LOCALIZATION_DEBUG_CSV_ENABLE="$debug_csv_enable" \
  GLIM_LOCALIZATION_DEBUG_CSV_PATH="$debug_csv_path" \
  GLIM_LOCALIZATION_RELOCALIZATION_DEBUG_ENABLE="$relocalization_debug_enable" \
	  GLIM_LOCALIZATION_RELOCALIZATION_DEBUG_TOPK="$relocalization_debug_topk" \
	  GLIM_LOCALIZATION_RELOCALIZATION_DEBUG_VERIFY_REJECTED_TOPK="$relocalization_debug_verify_rejected_topk" \
	  GLIM_LOCALIZATION_RELOCALIZATION_DEBUG_VERIFY_REJECTED_TOPK_K="$relocalization_debug_verify_rejected_topk_k" \
	  GLIM_LOCALIZATION_VERIFY_RAW_TOPK_ENABLE="$verify_raw_topk_enable" \
	  GLIM_LOCALIZATION_VERIFY_RAW_TOPK_DISTANCE="$verify_raw_topk_distance" \
	  GLIM_LOCALIZATION_VERIFY_RAW_TOPK_K="$verify_raw_topk_k" \
	  GLIM_LOCALIZATION_VERIFY_RAW_TOPK_REQUIRE_GEOMETRIC_VERIFICATION="1" \
	  GLIM_LOCALIZATION_VERIFY_RAW_TOPK_ALLOW_DESCRIPTOR_REJECTED="1" \
	  GLIM_LOCALIZATION_RECOVERING_ENABLE="$recovering_enable" \
	  GLIM_LOCALIZATION_RECOVERING_STABLE_FRAMES="$recovering_stable_frames" \
	  GLIM_LOCALIZATION_SMOOTHER_GUARD_ENABLE="$smoother_guard_enable" \
  GLIM_LOCALIZATION_SMOOTHER_GUARD_MODE="hold_last_pose_prior" \
  GLIM_LOCALIZATION_SMOOTHER_GUARD_MAX_LOST="0" \
  GLIM_LOCALIZATION_SMOOTHER_GUARD_MAX_HOLDS="$smoother_guard_max_holds" \
  GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_ENABLE="$target_rebuild_guard_enable" \
  GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_RATIO="$target_rebuild_guard_ratio" \
  GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_CONFIRMATION_FRAMES="$target_rebuild_guard_confirmation_frames" \
  GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_CONSISTENCY_TRANSLATION="$target_rebuild_guard_consistency_translation" \
  GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_CONSISTENCY_ANGLE="$target_rebuild_guard_consistency_angle" \
  GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_COOLDOWN_FRAMES="$target_rebuild_guard_cooldown_frames" \
  GLIM_LOCALIZATION_LOST_RECOVERY_ENABLE="$lost_recovery_enable" \
  GLIM_LOCALIZATION_LOST_RECOVERY_PERIOD_FRAMES="$lost_recovery_period_frames" \
  GLIM_LOCALIZATION_LOST_RECOVERY_STABLE_FRAMES="$lost_recovery_stable_frames" \
  GLIM_LOCALIZATION_LOST_RECOVERY_MAX_LOST_BEFORE_PAUSE="$lost_recovery_max_lost_before_pause" \
  GLIM_LOCALIZATION_LOST_RECOVERY_WRITE_TRAJECTORY_WHILE_LOST="$lost_recovery_write_trajectory_while_lost" \
  "$monitor_script" \
  "${monitor_args[@]}" \
  -- \
  bash "$offline_script" "${offline_args[@]}"
run_exit_code=$?
set -e

echo "$run_exit_code" > "$output_dir/localization_exit_code.txt"

stdout_log="$monitor_dir/command.stdout.log"
stderr_log="$monitor_dir/command.stderr.log"
stdout_tail="$logs_dir/stdout_tail.txt"
stderr_tail="$logs_dir/stderr_tail.txt"
state_summary="$logs_dir/state_transition_summary.txt"
rejection_summary="$logs_dir/registration_rejection_summary.txt"

write_tail() {
  local input="$1"
  local output="$2"
  if [[ -f "$input" ]]; then
    tail -n 200 "$input" > "$output" || true
  else
    echo "missing log file: $input" > "$output"
  fi
}

summarize_state_transitions() {
  if [[ -f "$stdout_log" ]]; then
    grep -E "localization status transition" "$stdout_log" > "$state_summary" || true
    if [[ ! -s "$state_summary" ]]; then
      echo "no localization status transitions found" > "$state_summary"
    fi
  else
    echo "missing stdout log: $stdout_log" > "$state_summary"
  fi
}

summarize_registration_rejections() {
  if [[ -f "$stdout_log" ]]; then
    {
      echo "# registration rejection lines"
      grep -E "scan-to-map registration rejected" "$stdout_log" || true
      echo
      echo "# rejection reason counts"
      grep -E "scan-to-map registration rejected" "$stdout_log" \
        | grep -Eo "reason=[^ ]+" \
        | sort \
        | uniq -c \
        | sort -nr || true
    } > "$rejection_summary"
    if [[ ! -s "$rejection_summary" ]]; then
      echo "no scan-to-map registration rejections found" > "$rejection_summary"
    fi
  else
    echo "missing stdout log: $stdout_log" > "$rejection_summary"
  fi
}

interpret_exit_code() {
  local code="$1"
  case "$code" in
    0) echo "success" ;;
    124) echo "timeout command reached its time limit" ;;
    137) echo "process was killed, usually SIGKILL or OOM" ;;
    143) echo "process received SIGTERM, often from timeout or external stop" ;;
    *) echo "non-zero exit code" ;;
  esac
}

resource_report_status="not_attempted"
set +e
python3 "$generate_report_script" \
  --input-dir "$monitor_dir" \
  --output-dir "$resource_report_dir" \
  --title "$label" \
  > "$logs_dir/resource_report_stdout.txt" \
  2> "$logs_dir/resource_report_stderr.txt"
resource_report_exit=$?
set -e
if [[ "$resource_report_exit" -eq 0 ]]; then
  resource_report_status="generated"
else
  resource_report_status="failed_exit_${resource_report_exit}"
fi

trajectory_status="not_attempted"
if [[ -s "$trajectory_path" ]]; then
  set +e
  python3 "$plot_trajectory_script" \
    "$trajectory_path" \
    --plot-2d \
    --plot-3d \
    --show-arrows \
    --time-color \
    -o "$trajectory_dir/trajectory.png" \
    > "$trajectory_dir/trajectory_stats.txt" \
    2> "$logs_dir/trajectory_plot_stderr.txt"
  trajectory_exit=$?
  set -e
  if [[ "$trajectory_exit" -eq 0 ]]; then
    trajectory_status="generated"
  else
    trajectory_status="failed_exit_${trajectory_exit}"
  fi
else
  trajectory_status="missing_or_empty_trajectory"
  echo "trajectory file is missing or empty: $trajectory_path" > "$trajectory_dir/trajectory_stats.txt"
fi

benchmark_status="not_attempted"
set +e
ros2 run glim_localization benchmark_localization \
  "$map_path" \
  "$benchmark_queries" \
  "$benchmark_distance" \
  "$benchmark_submaps" \
  "$benchmark_index_resolution" \
  > "$benchmark_dir/benchmark.txt" \
  2> "$logs_dir/benchmark_stderr.txt"
benchmark_exit=$?
set -e
if [[ "$benchmark_exit" -eq 0 ]]; then
  benchmark_status="generated"
else
  benchmark_status="failed_exit_${benchmark_exit}"
fi

write_tail "$stdout_log" "$stdout_tail"
write_tail "$stderr_log" "$stderr_tail"
summarize_state_transitions
summarize_registration_rejections

lost_count=0
relocalizing_count=0
rejection_count=0
gtsam_warning_count=0
if [[ -f "$stdout_log" ]]; then
  lost_count=$(grep -Ec -- "-> LOST|status=LOST| LOST " "$stdout_log" || true)
  relocalizing_count=$(grep -Ec -- "-> RELOCALIZING|status=RELOCALIZING| RELOCALIZING " "$stdout_log" || true)
  rejection_count=$(grep -Ec "scan-to-map registration rejected" "$stdout_log" || true)
fi
if [[ -f "$stderr_log" ]]; then
  gtsam_warning_count=$(grep -Ec "Indeterminant linear system|Indeterminate linear system|underconstrained" "$stderr_log" || true)
fi

cat > "$failure_summary_file" <<EOF
# Failure Summary: ${label}

## Command Result
- localization_exit_code: ${run_exit_code}
- interpretation: $(interpret_exit_code "$run_exit_code")
- timeout_sec: ${timeout_sec:-not_set}
- start_offset: ${start_offset:-0}
- duration: ${duration:-full_bag}
- max_frames: ${max_frames:-not_set}
- debug_csv_enable: ${debug_csv_enable}
- debug_csv_path: ${debug_csv_path}
- relocalization_debug_enable: ${relocalization_debug_enable}
- relocalization_debug_topk: ${relocalization_debug_topk}
- relocalization_debug_verify_rejected_topk: ${relocalization_debug_verify_rejected_topk}
- relocalization_debug_verify_rejected_topk_k: ${relocalization_debug_verify_rejected_topk_k}
- verify_raw_topk_enable: ${verify_raw_topk_enable}
- verify_raw_topk_distance: ${verify_raw_topk_distance}
- verify_raw_topk_k: ${verify_raw_topk_k}
- recovering_enable: ${recovering_enable}
- recovering_stable_frames: ${recovering_stable_frames}
- smoother_guard_enable: ${smoother_guard_enable}
- smoother_guard_max_holds: ${smoother_guard_max_holds}
- target_rebuild_guard_enable: ${target_rebuild_guard_enable}
- target_rebuild_guard_ratio: ${target_rebuild_guard_ratio}
- target_rebuild_guard_confirmation_frames: ${target_rebuild_guard_confirmation_frames}
- target_rebuild_guard_consistency_translation: ${target_rebuild_guard_consistency_translation}
- target_rebuild_guard_consistency_angle: ${target_rebuild_guard_consistency_angle}
- target_rebuild_guard_cooldown_frames: ${target_rebuild_guard_cooldown_frames}
- lost_recovery_enable: ${lost_recovery_enable}
- lost_recovery_period_frames: ${lost_recovery_period_frames}
- lost_recovery_stable_frames: ${lost_recovery_stable_frames}
- lost_recovery_max_lost_before_pause: ${lost_recovery_max_lost_before_pause}
- lost_recovery_write_trajectory_while_lost: ${lost_recovery_write_trajectory_while_lost}

## Artifact Generation
- resource_report_status: ${resource_report_status}
- trajectory_status: ${trajectory_status}
- benchmark_status: ${benchmark_status}

## Log Summary
- lost_transition_or_status_count: ${lost_count}
- relocalizing_transition_or_status_count: ${relocalizing_count}
- registration_rejection_count: ${rejection_count}
- gtsam_warning_count: ${gtsam_warning_count}

## Key Files
- stdout_tail: ${stdout_tail}
- stderr_tail: ${stderr_tail}
- state_transition_summary: ${state_summary}
- registration_rejection_summary: ${rejection_summary}
- trajectory_stats: ${trajectory_dir}/trajectory_stats.txt
- monitor_summary: ${monitor_dir}/summary.txt

## Exit Code Notes
- 124 means GNU timeout stopped the command after the configured time limit.
- 143 means a process received SIGTERM, commonly because timeout or a parent wrapper terminated it.
- A non-zero localization exit code does not prevent artifact generation in this script.
EOF

cat > "$record_file" <<EOF
# Experiment Record: ${label}

## Context
- Date: $(date -Iseconds)
- Matching method: ${matching_method}
- Bag: ${bag_path}
- Map: ${map_path}
- Initial pose: ${initial_pose}
- Start offset: ${start_offset:-0}
- Duration: ${duration:-full_bag}
- Timeout sec: ${timeout_sec:-not_set}
- Max frames: ${max_frames:-not_set}$(if [[ -n "$max_frames" ]]; then echo " (recorded only; glim_rosbag has no max-frames parameter)"; fi)
- Localization exit code: ${run_exit_code} ($(interpret_exit_code "$run_exit_code"))

## Artifacts
- Manifest: [experiment_manifest.txt]($(realpath -m "$manifest_file"))
- Failure summary: [failure_summary.md]($(realpath -m "$failure_summary_file"))
- Monitor summary: [summary.txt]($(realpath -m "$monitor_dir/summary.txt"))
- Resource report: [resource_report.html]($(realpath -m "$resource_report_dir/resource_report.html")) (${resource_report_status})
- Trajectory stats: [trajectory_stats.txt]($(realpath -m "$trajectory_dir/trajectory_stats.txt")) (${trajectory_status})
- Trajectory plots: [trajectory_2d.png]($(realpath -m "$trajectory_dir/trajectory_2d.png")), [trajectory_3d.png]($(realpath -m "$trajectory_dir/trajectory_3d.png"))
- Benchmark: [benchmark.txt]($(realpath -m "$benchmark_dir/benchmark.txt")) (${benchmark_status})
- Stdout tail: [stdout_tail.txt]($(realpath -m "$stdout_tail"))
- Stderr tail: [stderr_tail.txt]($(realpath -m "$stderr_tail"))
- State transitions: [state_transition_summary.txt]($(realpath -m "$state_summary"))
- Registration rejections: [registration_rejection_summary.txt]($(realpath -m "$rejection_summary"))
- Debug CSV: [localization_debug.csv]($(realpath -m "$debug_csv_path"))

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
echo "[glim_localization] localization exit code: $run_exit_code ($(interpret_exit_code "$run_exit_code"))"
echo "[glim_localization] manifest: $manifest_file"
echo "[glim_localization] record template: $record_file"
echo "[glim_localization] resource report: $resource_report_dir/resource_report.html"
echo "[glim_localization] failure summary: $failure_summary_file"

exit "$run_exit_code"
