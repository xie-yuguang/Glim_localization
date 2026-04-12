#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/estimation_frame.hpp>
#include <glim_localization/map/local_target_map.hpp>
#include <glim_localization/registration/registration_result.hpp>

namespace glim_localization {

class MapRegistrationBase {
public:
  using Ptr = std::shared_ptr<MapRegistrationBase>;
  using ConstPtr = std::shared_ptr<const MapRegistrationBase>;

  virtual ~MapRegistrationBase() {}

  virtual RegistrationResult align(
    const glim::EstimationFrame::ConstPtr& frame,
    const LocalTargetMap::ConstPtr& target,
    const Eigen::Isometry3d& initial_T_map_imu) = 0;
};

}  // namespace glim_localization
