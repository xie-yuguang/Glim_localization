#include <glim_localization/registration/map_registration_base.hpp>

#include <algorithm>
#include <cmath>

namespace glim_localization {

MapRegistrationBase::MapRegistrationBase(const MatchingOptions& options) : options_(options) {}

const MatchingOptions& MapRegistrationBase::options() const {
  return options_;
}

double MapRegistrationBase::clamp_score(double score) {
  if (!std::isfinite(score)) {
    return 0.0;
  }
  return std::max(0.0, std::min(1.0, score));
}

double MapRegistrationBase::residual_confidence_score(double residual, int num_points) {
  if (!std::isfinite(residual)) {
    return 0.0;
  }

  return clamp_score(std::exp(-std::min(100.0, residual / std::max(1, num_points))));
}

int MapRegistrationBase::estimate_inliers_from_score(double score, int num_source_points) {
  return static_cast<int>(std::lround(clamp_score(score) * std::max(0, num_source_points)));
}

void MapRegistrationBase::finalize_result(RegistrationResult& result, const Eigen::Isometry3d& initial_T_map_imu) const {
  const Eigen::Isometry3d delta = initial_T_map_imu.inverse() * result.T_map_imu;
  result.pose_delta_translation = delta.translation().norm();
  result.pose_delta_angle = Eigen::AngleAxisd(Eigen::Quaterniond(delta.linear()).normalized()).angle();
  result.score = clamp_score(result.score);
  result.accepted = false;
  result.reject_reason.clear();

  if (!result.converged) {
    result.reject_reason = "not_converged";
    return;
  }

  if (!std::isfinite(result.residual) || (options_.max_residual > 0.0 && result.residual > options_.max_residual)) {
    result.reject_reason = "high_residual";
    return;
  }

  if (!std::isfinite(result.score) || result.score < options_.min_score) {
    result.reject_reason = "low_score";
    return;
  }

  if (options_.min_inliers > 0 && result.num_inliers < options_.min_inliers) {
    result.reject_reason = "few_inliers";
    return;
  }

  if (options_.max_pose_correction_translation > 0.0 && result.pose_delta_translation > options_.max_pose_correction_translation) {
    result.reject_reason = "large_translation_correction";
    return;
  }

  if (options_.max_pose_correction_angle > 0.0 && result.pose_delta_angle > options_.max_pose_correction_angle) {
    result.reject_reason = "large_rotation_correction";
    return;
  }

  result.accepted = true;
}

}  // namespace glim_localization
