#pragma once

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim_localization {

struct InitialPoseOptions {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::string source = "config";
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();

  Eigen::Isometry3d T_map_imu() const;
};

}  // namespace glim_localization
