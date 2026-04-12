#include <glim_localization/initialization/config_initial_pose_provider.hpp>

#include <Eigen/Geometry>

namespace glim_localization {

ConfigInitialPoseProvider::ConfigInitialPoseProvider() {}

ConfigInitialPoseProvider::ConfigInitialPoseProvider(const InitialPoseOptions& options) : options_(options) {}

bool ConfigInitialPoseProvider::ready() const {
  return options_.source == "config";
}

const InitialPoseOptions& ConfigInitialPoseProvider::options() const {
  return options_;
}

Eigen::Isometry3d ConfigInitialPoseProvider::T_map_imu() const {
  return options_.T_map_imu();
}

}  // namespace glim_localization
