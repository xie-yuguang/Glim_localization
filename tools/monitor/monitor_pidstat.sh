#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "Usage: monitor_pidstat.sh <pid> <output_prefix> [interval_sec]" >&2
}

if [[ $# -lt 2 ]]; then
  usage
  exit 1
fi

if ! command -v pidstat >/dev/null 2>&1; then
  echo "pidstat is not available" >&2
  exit 127
fi

target_pid="$1"
output_prefix="$2"
interval="${3:-1}"

mkdir -p "$(dirname "$output_prefix")"

summary_log="${output_prefix}_cpu_mem_io.log"
thread_log="${output_prefix}_threads.log"

pidstat -h -u -r -d -p "$target_pid" "$interval" > "$summary_log" 2>&1 &
collector_a=$!
pidstat -h -t -u -r -d -p "$target_pid" "$interval" > "$thread_log" 2>&1 &
collector_b=$!

while kill -0 "$target_pid" 2>/dev/null; do
  sleep "$interval"
done

kill "$collector_a" "$collector_b" 2>/dev/null || true
wait "$collector_a" 2>/dev/null || true
wait "$collector_b" 2>/dev/null || true
