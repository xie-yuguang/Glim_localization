#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/estimation_frame.hpp>
#include <glim_localization/core/localization_options.hpp>
#include <glim_localization/map/local_target_map.hpp>
#include <glim_localization/registration/registration_result.hpp>

namespace glim_localization {

class MapRegistrationBase {
public:
  using Ptr = std::shared_ptr<MapRegistrationBase>;
  using ConstPtr = std::shared_ptr<const MapRegistrationBase>;

  explicit MapRegistrationBase(const MatchingOptions& options = MatchingOptions());
  virtual ~MapRegistrationBase() {}

  virtual RegistrationResult align(
    const glim::EstimationFrame::ConstPtr& frame,
    const LocalTargetMap::ConstPtr& target,
    const Eigen::Isometry3d& initial_T_map_imu) = 0;

protected:
  const MatchingOptions& options() const;
  void finalize_result(RegistrationResult& result, const Eigen::Isometry3d& initial_T_map_imu) const;
  static double clamp_score(double score);
  static double residual_confidence_score(double residual, int num_points);
  static int estimate_inliers_from_score(double score, int num_source_points);

private:
  MatchingOptions options_;
};

}  // namespace glim_localization
