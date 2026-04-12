#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim_localization/initialization/initial_pose_options.hpp>

namespace glim_localization {

class ConfigInitialPoseProvider {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConfigInitialPoseProvider();
  explicit ConfigInitialPoseProvider(const InitialPoseOptions& options);

  bool ready() const;
  const InitialPoseOptions& options() const;
  Eigen::Isometry3d T_map_imu() const;

private:
  InitialPoseOptions options_;
};

}  // namespace glim_localization
