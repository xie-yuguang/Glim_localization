#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "Usage: monitor_gpu.sh <output_prefix> [interval_sec] [target_pid]" >&2
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

if ! command -v nvidia-smi >/dev/null 2>&1; then
  echo "nvidia-smi is not available" >&2
  exit 127
fi

output_prefix="$1"
interval="${2:-1}"
target_pid="${3:-}"

mkdir -p "$(dirname "$output_prefix")"

gpu_csv="${output_prefix}_gpus.csv"
app_csv="${output_prefix}_apps.csv"

echo "timestamp_iso,gpu_index,name,utilization_gpu_pct,utilization_mem_pct,memory_used_mb,memory_total_mb,power_draw_w" > "$gpu_csv"
echo "timestamp_iso,gpu_uuid,pid,process_name,used_gpu_memory_mb" > "$app_csv"

sample_gpu() {
  local timestamp_iso
  timestamp_iso="$(date -Iseconds)"

  local gpu_rows
  gpu_rows=$(nvidia-smi \
    --query-gpu=index,name,utilization.gpu,utilization.memory,memory.used,memory.total,power.draw \
    --format=csv,noheader,nounits 2>/dev/null || true)

  if [[ -n "$gpu_rows" ]]; then
    while IFS=',' read -r index name util_gpu util_mem mem_used mem_total power_draw; do
      [[ -n "${index// /}" ]] || continue
      printf '%s,%s,%s,%s,%s,%s,%s,%s\n' \
        "$timestamp_iso" \
        "$(echo "$index" | xargs)" \
        "$(echo "$name" | xargs)" \
        "$(echo "$util_gpu" | xargs)" \
        "$(echo "$util_mem" | xargs)" \
        "$(echo "$mem_used" | xargs)" \
        "$(echo "$mem_total" | xargs)" \
        "$(echo "$power_draw" | xargs)" >> "$gpu_csv"
    done <<< "$gpu_rows"
  fi

  local app_rows
  app_rows=$(nvidia-smi \
    --query-compute-apps=gpu_uuid,pid,process_name,used_gpu_memory \
    --format=csv,noheader,nounits 2>/dev/null || true)

  if [[ -n "$app_rows" ]]; then
    while IFS=',' read -r gpu_uuid pid process_name used_gpu_memory; do
      [[ -n "${pid// /}" ]] || continue
      pid="$(echo "$pid" | xargs)"
      if [[ -n "$target_pid" ]] && [[ "$pid" != "$target_pid" ]]; then
        continue
      fi

      printf '%s,%s,%s,%s,%s\n' \
        "$timestamp_iso" \
        "$(echo "$gpu_uuid" | xargs)" \
        "$pid" \
        "$(echo "$process_name" | xargs)" \
        "$(echo "$used_gpu_memory" | xargs)" >> "$app_csv"
    done <<< "$app_rows"
  fi
}

sample_gpu

if [[ -n "$target_pid" ]]; then
  while kill -0 "$target_pid" 2>/dev/null; do
    sleep "$interval"
    sample_gpu
  done
else
  while true; do
    sleep "$interval"
    sample_gpu
  done
fi
