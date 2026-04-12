#pragma once

#include <vector>
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
  double matching_score = 0.0;
  std::vector<int> target_submap_ids;
};

}  // namespace glim_localization
