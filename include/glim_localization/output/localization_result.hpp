#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim_localization/core/localization_status.hpp>

namespace glim_localization {

inline constexpr const char* kLocalizationResultCustomDataKey = "glim_localization_result";
inline constexpr const char* kLocalizationTargetMapCustomDataKey = "glim_localization_target_map";

struct LocalizationResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double stamp = 0.0;
  Eigen::Isometry3d T_map_imu = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_map_odom = Eigen::Isometry3d::Identity();
  LocalizationStatus status = LocalizationStatus::WAIT_MAP;
  std::string status_reason;
  double matching_score = 0.0;
  std::string backend_name;
  std::string score_type;
  std::string reject_reason;
  std::string relocalization_message;
  int consecutive_rejections = 0;
  int relocalization_attempts = 0;
  int relocalization_candidate_count = 0;
  int relocalization_verified_rank = -1;
  int recovery_frames_remaining = 0;
  int stable_tracking_successes = 0;
  double descriptor_distance = 0.0;
  double pose_delta_translation = 0.0;
  double pose_delta_angle = 0.0;
  double continuity_translation = 0.0;
  double continuity_angle = 0.0;
  bool continuity_adjusted = false;
  std::vector<int> target_submap_ids;
};

}  // namespace glim_localization
