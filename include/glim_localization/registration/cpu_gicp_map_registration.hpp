#pragma once

#include <memory>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim_localization/core/localization_options.hpp>
#include <glim_localization/registration/map_registration_base.hpp>

namespace glim_localization {

class CpuGicpMapRegistration : public MapRegistrationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit CpuGicpMapRegistration(const MatchingOptions& options = MatchingOptions());
  ~CpuGicpMapRegistration() override;

  RegistrationResult align(
    const glim::EstimationFrame::ConstPtr& frame,
    const LocalTargetMap::ConstPtr& target,
    const Eigen::Isometry3d& initial_T_map_imu) override;

private:
  gtsam_points::PointCloudCPU::Ptr build_target_cloud(const LocalTargetMap::ConstPtr& target) const;
};

}  // namespace glim_localization
