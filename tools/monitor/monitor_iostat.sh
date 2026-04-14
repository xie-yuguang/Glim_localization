#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "Usage: monitor_iostat.sh <output_log> [interval_sec] [until_pid]" >&2
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

if ! command -v iostat >/dev/null 2>&1; then
  echo "iostat is not available" >&2
  exit 127
fi

output_log="$1"
interval="${2:-1}"
until_pid="${3:-}"

mkdir -p "$(dirname "$output_log")"

iostat -x -y -t "$interval" > "$output_log" 2>&1 &
collector_pid=$!

if [[ -n "$until_pid" ]]; then
  while kill -0 "$until_pid" 2>/dev/null; do
    sleep "$interval"
  done

  kill "$collector_pid" 2>/dev/null || true
fi

wait "$collector_pid" 2>/dev/null || true
