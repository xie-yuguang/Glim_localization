#pragma once

namespace glim_localization {

enum class LocalizationStatus {
  WAIT_MAP,
  WAIT_INITIAL_POSE,
  INITIALIZING,
  DEGRADED,
  TRACKING,
  LOST,
  RELOCALIZING,
  RECOVERING,

  WAITING_FOR_MAP = WAIT_MAP,
  WAITING_FOR_INITIAL_POSE = WAIT_INITIAL_POSE
};

const char* to_string(LocalizationStatus status);

}  // namespace glim_localization
