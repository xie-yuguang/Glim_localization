#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 8 ]]; then
  echo "usage: $0 <rosbag_path> <map_path> <x> <y> <z> <roll> <pitch> <yaw> [trajectory_path] [--start-offset SEC] [--duration SEC] [--timeout-sec SEC] [--max-frames N]" >&2
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
TRAJ_PATH="/tmp/glim_localization_traj.txt"
if [[ $# -ge 9 && "${9}" != --* ]]; then
  TRAJ_PATH="$9"
  shift 9
else
  shift 8
fi

START_OFFSET=""
PLAYBACK_DURATION=""
TIMEOUT_SEC=""
MAX_FRAMES=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --start-offset)
      START_OFFSET="$2"
      shift 2
      ;;
    --duration|--playback-duration)
      PLAYBACK_DURATION="$2"
      shift 2
      ;;
    --timeout-sec)
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --max-frames)
      MAX_FRAMES="$2"
      shift 2
      ;;
    -h|--help)
      echo "usage: $0 <rosbag_path> <map_path> <x> <y> <z> <roll> <pitch> <yaw> [trajectory_path] [--start-offset SEC] [--duration SEC] [--timeout-sec SEC] [--max-frames N]" >&2
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
as_ros_double() {
  local value="$1"
  if [[ "$value" =~ ^-?[0-9]+$ ]]; then
    printf '%s.0\n' "$value"
  else
    printf '%s\n' "$value"
  fi
}

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

debug_csv_enable = os.environ.get("GLIM_LOCALIZATION_DEBUG_CSV_ENABLE", "").strip().lower()
debug_csv_path = os.environ.get("GLIM_LOCALIZATION_DEBUG_CSV_PATH", "").strip()
if debug_csv_enable or debug_csv_path:
    debug = localization.setdefault("debug", {})
    if debug_csv_enable:
        debug["csv_enable"] = debug_csv_enable in ("1", "true", "yes", "on")
    if debug_csv_path:
        debug["csv_path"] = debug_csv_path

relocalization_debug_enable = os.environ.get("GLIM_LOCALIZATION_RELOCALIZATION_DEBUG_ENABLE", "").strip().lower()
relocalization_debug_topk = os.environ.get("GLIM_LOCALIZATION_RELOCALIZATION_DEBUG_TOPK", "").strip()
relocalization_debug_dump_rejected = os.environ.get("GLIM_LOCALIZATION_RELOCALIZATION_DEBUG_DUMP_REJECTED", "").strip().lower()
relocalization_debug_verify_rejected = os.environ.get("GLIM_LOCALIZATION_RELOCALIZATION_DEBUG_VERIFY_REJECTED_TOPK", "").strip().lower()
relocalization_debug_verify_rejected_k = os.environ.get("GLIM_LOCALIZATION_RELOCALIZATION_DEBUG_VERIFY_REJECTED_TOPK_K", "").strip()
if (
    relocalization_debug_enable
    or relocalization_debug_topk
    or relocalization_debug_dump_rejected
    or relocalization_debug_verify_rejected
    or relocalization_debug_verify_rejected_k
):
    relocalization_debug = localization.setdefault("relocalization_debug", {})
    if relocalization_debug_enable:
        relocalization_debug["enable"] = relocalization_debug_enable in ("1", "true", "yes", "on")
    if relocalization_debug_topk:
        relocalization_debug["dump_topk"] = int(relocalization_debug_topk)
    if relocalization_debug_dump_rejected:
        relocalization_debug["dump_rejected_candidates"] = relocalization_debug_dump_rejected in ("1", "true", "yes", "on")
    if relocalization_debug_verify_rejected:
        relocalization_debug["verify_rejected_topk"] = relocalization_debug_verify_rejected in ("1", "true", "yes", "on")
        if relocalization_debug_verify_rejected_k:
            relocalization_debug["verify_rejected_topk_k"] = int(relocalization_debug_verify_rejected_k)

verify_raw_topk_enable = os.environ.get("GLIM_LOCALIZATION_VERIFY_RAW_TOPK_ENABLE", "").strip().lower()
verify_raw_topk_distance = os.environ.get("GLIM_LOCALIZATION_VERIFY_RAW_TOPK_DISTANCE", "").strip()
verify_raw_topk_k = os.environ.get("GLIM_LOCALIZATION_VERIFY_RAW_TOPK_K", "").strip()
verify_raw_topk_require_geom = os.environ.get("GLIM_LOCALIZATION_VERIFY_RAW_TOPK_REQUIRE_GEOMETRIC_VERIFICATION", "").strip().lower()
verify_raw_topk_allow_rejected = os.environ.get("GLIM_LOCALIZATION_VERIFY_RAW_TOPK_ALLOW_DESCRIPTOR_REJECTED", "").strip().lower()
recovering_enable = os.environ.get("GLIM_LOCALIZATION_RECOVERING_ENABLE", "").strip().lower()
recovering_stable_frames = os.environ.get("GLIM_LOCALIZATION_RECOVERING_STABLE_FRAMES", "").strip()
recovering_max_translation = os.environ.get("GLIM_LOCALIZATION_RECOVERING_MAX_TRANSLATION", "").strip()
recovering_max_angle = os.environ.get("GLIM_LOCALIZATION_RECOVERING_MAX_ANGLE", "").strip()
recovering_write_traj = os.environ.get("GLIM_LOCALIZATION_RECOVERING_WRITE_TRAJECTORY", "").strip().lower()
if (
    verify_raw_topk_enable
    or verify_raw_topk_distance
    or verify_raw_topk_k
    or verify_raw_topk_require_geom
    or verify_raw_topk_allow_rejected
    or recovering_enable
    or recovering_stable_frames
    or recovering_max_translation
    or recovering_max_angle
    or recovering_write_traj
):
    relocalization = localization.setdefault("relocalization", {})
    verify_raw_topk = relocalization.setdefault("verify_raw_topk", {})
    if verify_raw_topk_enable:
        verify_raw_topk["enable"] = verify_raw_topk_enable in ("1", "true", "yes", "on")
    if verify_raw_topk_k:
        verify_raw_topk["topk"] = int(verify_raw_topk_k)
    if verify_raw_topk_distance:
        verify_raw_topk["max_descriptor_distance"] = float(verify_raw_topk_distance)
    if verify_raw_topk_require_geom:
        verify_raw_topk["require_geometric_verification"] = verify_raw_topk_require_geom in ("1", "true", "yes", "on")
    if verify_raw_topk_allow_rejected:
        verify_raw_topk["allow_descriptor_rejected_candidates"] = verify_raw_topk_allow_rejected in ("1", "true", "yes", "on")
    recovering = relocalization.setdefault("recovering", {})
    if recovering_enable:
        recovering["enable"] = recovering_enable in ("1", "true", "yes", "on")
    if recovering_stable_frames:
        recovering["stable_frames"] = int(recovering_stable_frames)
    if recovering_max_translation:
        recovering["max_recovery_correction_translation"] = float(recovering_max_translation)
    if recovering_max_angle:
        recovering["max_recovery_correction_angle"] = float(recovering_max_angle)
    if recovering_write_traj:
        recovering["write_trajectory_during_recovering"] = recovering_write_traj in ("1", "true", "yes", "on")

smoother_guard_enable = os.environ.get("GLIM_LOCALIZATION_SMOOTHER_GUARD_ENABLE", "").strip().lower()
smoother_guard_mode = os.environ.get("GLIM_LOCALIZATION_SMOOTHER_GUARD_MODE", "").strip()
smoother_guard_max = os.environ.get("GLIM_LOCALIZATION_SMOOTHER_GUARD_MAX_LOST", "").strip()
smoother_guard_max_holds = os.environ.get("GLIM_LOCALIZATION_SMOOTHER_GUARD_MAX_HOLDS", "").strip()
smoother_guard_write_traj = os.environ.get("GLIM_LOCALIZATION_SMOOTHER_GUARD_WRITE_TRAJECTORY_DURING_HOLD", "").strip().lower()
smoother_guard_publish_pose = os.environ.get("GLIM_LOCALIZATION_SMOOTHER_GUARD_PUBLISH_POSE_DURING_HOLD", "").strip().lower()
if smoother_guard_enable or smoother_guard_mode or smoother_guard_max or smoother_guard_max_holds or smoother_guard_write_traj or smoother_guard_publish_pose:
    smoother_guard = localization.setdefault("smoother_guard", {})
    if smoother_guard_enable:
        smoother_guard["enable"] = smoother_guard_enable in ("1", "true", "yes", "on")
    if smoother_guard_mode:
        smoother_guard["mode"] = smoother_guard_mode
    if smoother_guard_max:
        smoother_guard["max_lost_frames_without_prior"] = int(smoother_guard_max)
    if smoother_guard_max_holds:
        smoother_guard["max_consecutive_hold_priors"] = int(smoother_guard_max_holds)
    if smoother_guard_write_traj:
        smoother_guard["write_trajectory_during_hold"] = smoother_guard_write_traj in ("1", "true", "yes", "on")
    if smoother_guard_publish_pose:
        smoother_guard["publish_pose_during_hold"] = smoother_guard_publish_pose in ("1", "true", "yes", "on")

target_guard_enable = os.environ.get("GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_ENABLE", "").strip().lower()
target_guard_ratio = os.environ.get("GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_RATIO", "").strip()
target_guard_confirm = os.environ.get("GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_CONFIRMATION_FRAMES", "").strip()
target_guard_consistency_translation = os.environ.get("GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_CONSISTENCY_TRANSLATION", "").strip()
target_guard_consistency_angle = os.environ.get("GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_CONSISTENCY_ANGLE", "").strip()
target_guard_cooldown = os.environ.get("GLIM_LOCALIZATION_TARGET_REBUILD_GUARD_COOLDOWN_FRAMES", "").strip()
if target_guard_enable or target_guard_ratio or target_guard_confirm or target_guard_consistency_translation or target_guard_consistency_angle or target_guard_cooldown:
    target_guard = localization.setdefault("target_rebuild_guard", {})
    if target_guard_enable:
        target_guard["enable"] = target_guard_enable in ("1", "true", "yes", "on")
    if target_guard_ratio:
        target_guard["near_threshold_ratio"] = float(target_guard_ratio)
    if target_guard_confirm:
        target_guard["confirmation_frames"] = int(target_guard_confirm)
    if target_guard_consistency_translation:
        target_guard["consistency_translation"] = float(target_guard_consistency_translation)
    if target_guard_consistency_angle:
        target_guard["consistency_angle"] = float(target_guard_consistency_angle)
    if target_guard_cooldown:
        target_guard["cooldown_frames"] = int(target_guard_cooldown)

lost_recovery_enable = os.environ.get("GLIM_LOCALIZATION_LOST_RECOVERY_ENABLE", "").strip().lower()
lost_recovery_period = os.environ.get("GLIM_LOCALIZATION_LOST_RECOVERY_PERIOD_FRAMES", "").strip()
lost_recovery_stable = os.environ.get("GLIM_LOCALIZATION_LOST_RECOVERY_STABLE_FRAMES", "").strip()
lost_recovery_max_pause = os.environ.get("GLIM_LOCALIZATION_LOST_RECOVERY_MAX_LOST_BEFORE_PAUSE", "").strip()
lost_recovery_write_traj = os.environ.get("GLIM_LOCALIZATION_LOST_RECOVERY_WRITE_TRAJECTORY_WHILE_LOST", "").strip().lower()
if lost_recovery_enable or lost_recovery_period or lost_recovery_stable or lost_recovery_max_pause or lost_recovery_write_traj:
    lost_recovery = localization.setdefault("lost_recovery", {})
    if lost_recovery_enable:
        lost_recovery["enable"] = lost_recovery_enable in ("1", "true", "yes", "on")
    if lost_recovery_period:
        lost_recovery["relocalization_period_frames"] = int(lost_recovery_period)
    if lost_recovery_stable:
        lost_recovery["recovery_stable_frames"] = int(lost_recovery_stable)
    if lost_recovery_max_pause:
        lost_recovery["max_lost_frames_before_pause"] = int(lost_recovery_max_pause)
    if lost_recovery_write_traj:
        lost_recovery["write_trajectory_while_lost"] = lost_recovery_write_traj in ("1", "true", "yes", "on")

with open(config_path, "w", encoding="utf-8") as f:
    json.dump(config, f, indent=2)
    f.write("\n")
PY

echo "[glim_localization] temporary config: ${WORK_DIR}"
echo "[glim_localization] bag: ${BAG_PATH}"
echo "[glim_localization] map: ${MAP_PATH}"
echo "[glim_localization] trajectory: ${TRAJ_PATH}"
if [[ -n "$START_OFFSET" ]]; then
  echo "[glim_localization] start_offset: ${START_OFFSET}"
fi
if [[ -n "$PLAYBACK_DURATION" ]]; then
  echo "[glim_localization] playback_duration: ${PLAYBACK_DURATION}"
fi
if [[ -n "$TIMEOUT_SEC" ]]; then
  echo "[glim_localization] timeout_sec: ${TIMEOUT_SEC}"
fi
if [[ -n "$MAX_FRAMES" ]]; then
  echo "[glim_localization] max_frames requested but not supported by glim_rosbag; request recorded only" >&2
fi

ros_args=(--ros-args -p "config_path:=${WORK_DIR}")
if [[ -n "$START_OFFSET" ]]; then
  ros_args+=(-p "start_offset:=$(as_ros_double "$START_OFFSET")")
fi
if [[ -n "$PLAYBACK_DURATION" ]]; then
  ros_args+=(-p "playback_duration:=$(as_ros_double "$PLAYBACK_DURATION")")
fi

if [[ -n "$TIMEOUT_SEC" ]]; then
  timeout "$TIMEOUT_SEC" ros2 run glim_ros glim_rosbag "${BAG_PATH}" "${ros_args[@]}"
else
  ros2 run glim_ros glim_rosbag "${BAG_PATH}" "${ros_args[@]}"
fi
