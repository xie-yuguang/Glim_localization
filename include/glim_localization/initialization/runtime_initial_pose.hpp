#pragma once

#include <optional>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim_localization {

struct RuntimeInitialPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double stamp = 0.0;
  Eigen::Isometry3d T_map_imu = Eigen::Isometry3d::Identity();
};

void set_runtime_initial_pose(const RuntimeInitialPose& pose);
std::optional<RuntimeInitialPose> get_runtime_initial_pose();
void clear_runtime_initial_pose();

}  // namespace glim_localization
