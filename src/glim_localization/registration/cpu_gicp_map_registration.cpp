#include <glim_localization/registration/cpu_gicp_map_registration.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <vector>

#include <spdlog/spdlog.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

namespace glim_localization {

CpuGicpMapRegistration::CpuGicpMapRegistration(const MatchingOptions& options) : options_(options) {}

CpuGicpMapRegistration::~CpuGicpMapRegistration() {}

RegistrationResult CpuGicpMapRegistration::align(
  const glim::EstimationFrame::ConstPtr& frame,
  const LocalTargetMap::ConstPtr& target,
  const Eigen::Isometry3d& initial_T_map_imu) {
  RegistrationResult result;
  result.T_map_imu = initial_T_map_imu;

  if (!frame || !frame->frame) {
    spdlog::warn("CPU GICP registration skipped: source frame is empty");
    result.reject_reason = "empty_source";
    return result;
  }

  const auto target_cloud = build_target_cloud(target);
  if (!target_cloud || target_cloud->size() == 0) {
    spdlog::warn("CPU GICP registration skipped: target cloud is empty");
    result.reject_reason = "empty_target";
    return result;
  }

  result.num_source_points = frame->frame->size();
  result.num_target_points = target_cloud->size();

  gtsam::Values values;
  values.insert(0, gtsam::Pose3(initial_T_map_imu.matrix()));

  gtsam::NonlinearFactorGraph graph;
  auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(gtsam::Pose3(), 0, target_cloud, frame->frame);
  factor->set_max_correspondence_distance(options_.max_correspondence_distance);
  factor->set_num_threads(options_.num_threads);
  graph.add(factor);

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(options_.max_iterations);
  lm_params.setAbsoluteErrorTol(0.1);

  try {
    values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();
    result.T_map_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(0).matrix());
    result.residual = factor->error(values);
    result.score = factor->inlier_fraction();
    result.num_inliers = std::isfinite(result.score) ? static_cast<int>(std::lround(std::max(0.0, result.score) * result.num_source_points)) : 0;
    result.converged = true;
    evaluate_result(result, initial_T_map_imu);

    spdlog::debug(
      "CPU GICP registration residual={:.6f} score={:.3f} inliers={}/{} delta_t={:.3f} delta_r={:.3f} accepted={} reason={}",
      result.residual,
      result.score,
      result.num_inliers,
      result.num_source_points,
      result.pose_delta_translation,
      result.pose_delta_angle,
      result.accepted,
      result.reject_reason.empty() ? "none" : result.reject_reason);
  } catch (const std::exception& e) {
    spdlog::warn("CPU GICP registration failed: {}", e.what());
    result.reject_reason = "optimizer_exception";
  }

  return result;
}

void CpuGicpMapRegistration::evaluate_result(RegistrationResult& result, const Eigen::Isometry3d& initial_T_map_imu) const {
  const Eigen::Isometry3d delta = initial_T_map_imu.inverse() * result.T_map_imu;
  const Eigen::Quaterniond q_delta(delta.linear());
  result.pose_delta_translation = delta.translation().norm();
  result.pose_delta_angle = Eigen::AngleAxisd(q_delta.normalized()).angle();

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

gtsam_points::PointCloudCPU::Ptr CpuGicpMapRegistration::build_target_cloud(const LocalTargetMap::ConstPtr& target) const {
  if (!target || target->empty()) {
    return nullptr;
  }

  return std::const_pointer_cast<gtsam_points::PointCloudCPU>(target->merged_target_cloud());
}

}  // namespace glim_localization
