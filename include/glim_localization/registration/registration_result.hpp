#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim_localization {

struct RegistrationResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool converged = false;
  bool accepted = false;
  double score = 0.0;
  double residual = 0.0;
  double pose_delta_translation = 0.0;
  double pose_delta_angle = 0.0;
  int num_inliers = 0;
  int num_source_points = 0;
  int num_target_points = 0;
  std::string reject_reason;
  Eigen::Isometry3d T_map_imu = Eigen::Isometry3d::Identity();
};

}  // namespace glim_localization
