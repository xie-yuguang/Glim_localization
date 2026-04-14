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

echo "timestamp_iso,epoch_s,pid,pcpu,pmem,rss_kb,vsz_kb,nlwp,etimes_s,stat,read_bytes,write_bytes,rchar,wchar" > "$output_csv"

sample_once() {
  local ps_line
  if ! ps_line=$(ps -p "$target_pid" -o pid=,%cpu=,%mem=,rss=,vsz=,nlwp=,etimes=,stat= -ww 2>/dev/null); then
    return 1
  fi
  if [[ -z "${ps_line//[[:space:]]/}" ]]; then
    return 1
  fi

  local pid pcpu pmem rss vsz nlwp etimes stat
  read -r pid pcpu pmem rss vsz nlwp etimes stat <<<"$ps_line"

  local read_bytes="" write_bytes="" rchar="" wchar=""
  if [[ -r "/proc/$target_pid/io" ]]; then
    read_bytes=$(awk '/^read_bytes:/ {print $2}' "/proc/$target_pid/io" 2>/dev/null || true)
    write_bytes=$(awk '/^write_bytes:/ {print $2}' "/proc/$target_pid/io" 2>/dev/null || true)
    rchar=$(awk '/^rchar:/ {print $2}' "/proc/$target_pid/io" 2>/dev/null || true)
    wchar=$(awk '/^wchar:/ {print $2}' "/proc/$target_pid/io" 2>/dev/null || true)
  fi

  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$(date -Iseconds)" "$(date +%s)" "$pid" "$pcpu" "$pmem" "$rss" "$vsz" "$nlwp" "$etimes" "$stat" \
    "${read_bytes}" "${write_bytes}" "${rchar}" "${wchar}" >> "$output_csv"
}

sample_once || exit 0

while kill -0 "$target_pid" 2>/dev/null; do
  sleep "$interval"
  sample_once || true
done

sample_once || true
