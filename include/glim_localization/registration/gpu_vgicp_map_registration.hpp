#pragma once

#include <memory>

#include <glim_localization/core/localization_options.hpp>
#include <glim_localization/registration/map_registration_base.hpp>

namespace gtsam_points {
class CUDAStream;
class StreamTempBufferRoundRobin;
class PointCloudCPU;
}  // namespace gtsam_points

namespace glim_localization {

class GpuVgicpMapRegistration : public MapRegistrationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit GpuVgicpMapRegistration(const MatchingOptions& options = MatchingOptions());
  ~GpuVgicpMapRegistration() override;

  RegistrationResult align(
    const glim::EstimationFrame::ConstPtr& frame,
    const LocalTargetMap::ConstPtr& target,
    const Eigen::Isometry3d& initial_T_map_imu) override;

private:
  std::shared_ptr<gtsam_points::PointCloudCPU> build_target_cloud_cpu(const LocalTargetMap::ConstPtr& target) const;
  void evaluate_result(RegistrationResult& result, const Eigen::Isometry3d& initial_T_map_imu) const;

private:
  MatchingOptions options_;
  std::unique_ptr<gtsam_points::CUDAStream> stream_;
  std::unique_ptr<gtsam_points::StreamTempBufferRoundRobin> stream_buffer_roundrobin_;
};

}  // namespace glim_localization
