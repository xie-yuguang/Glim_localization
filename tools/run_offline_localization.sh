#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 8 ]]; then
  echo "usage: $0 <rosbag_path> <map_path> <x> <y> <z> <roll> <pitch> <yaw> [trajectory_path]" >&2
  exit 1
fi

BAG_PATH="$1"
MAP_PATH="$2"
INIT_X="$3"
INIT_Y="$4"
INIT_Z="$5"
INIT_ROLL="$6"
INIT_PITCH="$7"
INIT_YAW="$8"
TRAJ_PATH="${9:-/tmp/glim_localization_traj.txt}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
find_config_template_dir() {
  if [[ -n "${GLIM_LOCALIZATION_CONFIG_DIR:-}" && -d "${GLIM_LOCALIZATION_CONFIG_DIR}" ]]; then
    printf '%s\n' "${GLIM_LOCALIZATION_CONFIG_DIR}"
    return 0
  fi

  local candidates=(
    "${SCRIPT_DIR}/../config"
    "${SCRIPT_DIR}/../share/glim_localization/config"
    "${SCRIPT_DIR}/../../share/glim_localization/config"
  )

  for candidate in "${candidates[@]}"; do
    if [[ -d "${candidate}" && -f "${candidate}/config.json" && -f "${candidate}/localization.json" ]]; then
      (cd "${candidate}" && pwd)
      return 0
    fi
  done

  if command -v ros2 >/dev/null 2>&1; then
    local prefix
    prefix="$(ros2 pkg prefix glim_localization 2>/dev/null || true)"
    if [[ -n "${prefix}" && -d "${prefix}/share/glim_localization/config" ]]; then
      printf '%s\n' "${prefix}/share/glim_localization/config"
      return 0
    fi
  fi

  return 1
}

CONFIG_TEMPLATE_DIR="$(find_config_template_dir)" || {
  echo "[glim_localization] failed to locate config template directory" >&2
  echo "[glim_localization] tried source-tree and installed layouts; you can override with GLIM_LOCALIZATION_CONFIG_DIR" >&2
  exit 2
}
WORK_DIR="$(mktemp -d /tmp/glim_localization_config.XXXXXX)"
trap 'rm -rf "${WORK_DIR}"' EXIT

cp -r "${CONFIG_TEMPLATE_DIR}/." "${WORK_DIR}/"

python3 - "$WORK_DIR/localization.json" "$MAP_PATH" "$TRAJ_PATH" "$INIT_X" "$INIT_Y" "$INIT_Z" "$INIT_ROLL" "$INIT_PITCH" "$INIT_YAW" <<'PY'
import json
import os
import sys

config_path, map_path, traj_path = sys.argv[1], sys.argv[2], sys.argv[3]
xyz = [float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])]
rpy = [float(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9])]

with open(config_path, "r", encoding="utf-8") as f:
    config = json.load(f)

config.setdefault("glim_ros", {})["enable_local_mapping"] = False
config.setdefault("glim_ros", {})["enable_global_mapping"] = False
config.setdefault("odometry_estimation", {})["so_name"] = "libodometry_estimation_localization_cpu.so"

localization = config.setdefault("localization", {})
localization["map_path"] = map_path
localization["trajectory_path"] = traj_path

initial_pose = localization.setdefault("initial_pose", {})
initial_pose["source"] = "config"
initial_pose["xyz"] = xyz
initial_pose["rpy"] = rpy

matching_method = os.environ.get("GLIM_LOCALIZATION_MATCHING_METHOD", "").strip()
if matching_method:
    localization.setdefault("matching", {})["method"] = matching_method

with open(config_path, "w", encoding="utf-8") as f:
    json.dump(config, f, indent=2)
    f.write("\n")
PY

echo "[glim_localization] temporary config: ${WORK_DIR}"
echo "[glim_localization] bag: ${BAG_PATH}"
echo "[glim_localization] map: ${MAP_PATH}"
echo "[glim_localization] trajectory: ${TRAJ_PATH}"

ros2 run glim_ros glim_rosbag "${BAG_PATH}" --ros-args -p "config_path:=${WORK_DIR}"
