#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "Usage: monitor_ps.sh <pid> <output_csv> [interval_sec]" >&2
}

if [[ $# -lt 2 ]]; then
  usage
  exit 1
fi

target_pid="$1"
output_csv="$2"
interval="${3:-1}"

mkdir -p "$(dirname "$output_csv")"

echo "timestamp_iso,epoch_s,pid,pcpu,pmem,rss_kb,vsz_kb,nlwp,etimes_s,stat,read_bytes,write_bytes,rchar,wchar,pid_count,pids,commands" > "$output_csv"

collect_tree_pids() {
  local root="$1"
  local queue=("$root")
  local result=()
  local current children child

  while [[ "${#queue[@]}" -gt 0 ]]; do
    current="${queue[0]}"
    queue=("${queue[@]:1}")
    if ! kill -0 "$current" 2>/dev/null; then
      continue
    fi
    result+=("$current")
    children="$(pgrep -P "$current" 2>/dev/null || true)"
    for child in $children; do
      queue+=("$child")
    done
  done

  printf '%s\n' "${result[@]}"
}

read_io_field() {
  local pid="$1"
  local field="$2"
  if [[ -r "/proc/$pid/io" ]]; then
    awk -v key="${field}:" '$1 == key {print $2}' "/proc/$pid/io" 2>/dev/null || true
  fi
}

sample_once() {
  local pids
  mapfile -t pids < <(collect_tree_pids "$target_pid")
  if [[ "${#pids[@]}" -eq 0 ]]; then
    return 1
  fi

  local pid_arg
  pid_arg="$(IFS=,; echo "${pids[*]}")"

  local ps_lines
  if ! ps_lines=$(ps -p "$pid_arg" -o pid=,%cpu=,%mem=,rss=,vsz=,nlwp=,etimes=,stat=,comm= -ww 2>/dev/null); then
    return 1
  fi

  local pcpu_sum="0" pmem_sum="0" rss_sum=0 vsz_sum=0 nlwp_sum=0 etimes_max=0
  local read_bytes_sum=0 write_bytes_sum=0 rchar_sum=0 wchar_sum=0
  local stats=() commands=()
  local pid pcpu pmem rss vsz nlwp etimes stat comm value

  while read -r pid pcpu pmem rss vsz nlwp etimes stat comm; do
    [[ -z "${pid:-}" ]] && continue
    pcpu_sum=$(awk -v a="$pcpu_sum" -v b="${pcpu:-0}" 'BEGIN {printf "%.3f", a + b}')
    pmem_sum=$(awk -v a="$pmem_sum" -v b="${pmem:-0}" 'BEGIN {printf "%.3f", a + b}')
    rss_sum=$((rss_sum + ${rss:-0}))
    vsz_sum=$((vsz_sum + ${vsz:-0}))
    nlwp_sum=$((nlwp_sum + ${nlwp:-0}))
    if [[ "${etimes:-0}" -gt "$etimes_max" ]]; then
      etimes_max="$etimes"
    fi
    stats+=("${pid}:${stat}")
    commands+=("${pid}:${comm}")

    value="$(read_io_field "$pid" read_bytes)"
    read_bytes_sum=$((read_bytes_sum + ${value:-0}))
    value="$(read_io_field "$pid" write_bytes)"
    write_bytes_sum=$((write_bytes_sum + ${value:-0}))
    value="$(read_io_field "$pid" rchar)"
    rchar_sum=$((rchar_sum + ${value:-0}))
    value="$(read_io_field "$pid" wchar)"
    wchar_sum=$((wchar_sum + ${value:-0}))
  done <<<"$ps_lines"

  local joined_pids joined_stats joined_commands
  joined_pids="$(IFS=';'; echo "${pids[*]}")"
  joined_stats="$(IFS=';'; echo "${stats[*]}")"
  joined_commands="$(IFS=';'; echo "${commands[*]}")"

  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$(date -Iseconds)" "$(date +%s)" "$target_pid" "$pcpu_sum" "$pmem_sum" "$rss_sum" "$vsz_sum" "$nlwp_sum" "$etimes_max" "$joined_stats" \
    "$read_bytes_sum" "$write_bytes_sum" "$rchar_sum" "$wchar_sum" "${#pids[@]}" "$joined_pids" "$joined_commands" >> "$output_csv"
}

sample_once || exit 0

while kill -0 "$target_pid" 2>/dev/null; do
  sleep "$interval"
  sample_once || true
done

sample_once || true
