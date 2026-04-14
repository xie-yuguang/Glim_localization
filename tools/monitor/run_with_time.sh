#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_with_time.sh [-o OUTPUT_DIR] [-i INTERVAL] [--gpu] [--enhanced] [--label LABEL] -- <command> [args...]

Description:
  Run a command under external monitoring and save logs to OUTPUT_DIR.
  The wrapper does not modify glim_localization itself; it only observes the
  launched process from the outside.

Options:
  -o, --output-dir DIR   Output directory. Default: ./monitor_run_<timestamp>
  -i, --interval SEC     Sampling interval in seconds. Default: 1
      --gpu              Enable GPU sampling if nvidia-smi is available
      --enhanced         Also start pidstat and iostat collectors when available
      --label LABEL      Optional label written into metadata
  -h, --help             Show this help
EOF
}

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

output_dir=""
interval="1"
enable_gpu=0
enable_enhanced=0
label=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -o|--output-dir)
      output_dir="$2"
      shift 2
      ;;
    -i|--interval)
      interval="$2"
      shift 2
      ;;
    --gpu)
      enable_gpu=1
      shift
      ;;
    --enhanced)
      enable_enhanced=1
      shift
      ;;
    --label)
      label="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ $# -eq 0 ]]; then
  echo "missing command" >&2
  usage >&2
  exit 1
fi

if [[ -z "$output_dir" ]]; then
  output_dir="$(pwd)/monitor_run_$(date +%Y%m%d_%H%M%S)"
fi

mkdir -p "$output_dir"

command_file="$output_dir/command.txt"
meta_file="$output_dir/run_meta.txt"
time_log="$output_dir/time.txt"
stdout_log="$output_dir/command.stdout.log"
stderr_log="$output_dir/command.stderr.log"
pid_file="$output_dir/command.pid"
exit_code_file="$output_dir/exit_code.txt"

printf '%q ' "$@" | sed 's/ $/\n/' > "$command_file"

{
  echo "label=${label}"
  echo "start_time_iso=$(date -Iseconds)"
  echo "cwd=$(pwd)"
  echo "host=$(hostname)"
  echo "interval_sec=${interval}"
  echo "enhanced=${enable_enhanced}"
  echo "gpu_sampling=${enable_gpu}"
} > "$meta_file"

collector_pids=()
time_pid=""

cleanup() {
  if [[ -n "$time_pid" ]] && kill -0 "$time_pid" 2>/dev/null; then
    kill "$time_pid" 2>/dev/null || true
  fi

  for pid in "${collector_pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
}

trap cleanup INT TERM

/usr/bin/time -v -o "$time_log" \
  bash -lc 'pid_file="$1"; shift; echo $$ > "$pid_file"; exec "$@"' \
  bash "$pid_file" "$@" \
  >"$stdout_log" 2>"$stderr_log" &
time_pid=$!

main_pid=""
for _ in $(seq 1 100); do
  if [[ -s "$pid_file" ]]; then
    main_pid=$(cat "$pid_file")
    if [[ -n "$main_pid" ]] && kill -0 "$main_pid" 2>/dev/null; then
      break
    fi
  fi
  sleep 0.1
done

if [[ -z "$main_pid" ]] || ! kill -0 "$main_pid" 2>/dev/null; then
  echo "failed to capture target pid" >&2
  wait "$time_pid" || true
  exit 1
fi

echo "main_pid=${main_pid}" >> "$meta_file"

"$script_dir/monitor_ps.sh" "$main_pid" "$output_dir/ps_samples.csv" "$interval" &
collector_pids+=("$!")

if [[ "$enable_enhanced" -eq 1 ]]; then
  if command -v pidstat >/dev/null 2>&1; then
    "$script_dir/monitor_pidstat.sh" "$main_pid" "$output_dir/pidstat" "$interval" &
    collector_pids+=("$!")
  fi

  if command -v iostat >/dev/null 2>&1; then
    "$script_dir/monitor_iostat.sh" "$output_dir/iostat.log" "$interval" "$main_pid" &
    collector_pids+=("$!")
  fi
fi

if [[ "$enable_gpu" -eq 1 ]] && command -v nvidia-smi >/dev/null 2>&1; then
  "$script_dir/monitor_gpu.sh" "$output_dir/gpu" "$interval" "$main_pid" &
  collector_pids+=("$!")
fi

set +e
wait "$time_pid"
exit_code=$?
set -e

echo "$exit_code" > "$exit_code_file"
echo "end_time_iso=$(date -Iseconds)" >> "$meta_file"

for pid in "${collector_pids[@]:-}"; do
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
  fi
done
for pid in "${collector_pids[@]:-}"; do
  wait "$pid" 2>/dev/null || true
done

python3 "$script_dir/summarize_usage.py" --input-dir "$output_dir" > "$output_dir/summary.txt" || true

echo "monitor output: $output_dir"
echo "summary: $output_dir/summary.txt"
echo "exit code: $exit_code"

exit "$exit_code"
