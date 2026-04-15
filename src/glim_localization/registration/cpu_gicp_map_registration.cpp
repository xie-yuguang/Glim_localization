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

CpuGicpMapRegistration::CpuGicpMapRegistration(const MatchingOptions& options) : MapRegistrationBase(options) {}

CpuGicpMapRegistration::~CpuGicpMapRegistration() {}

RegistrationResult CpuGicpMapRegistration::align(
  const glim::EstimationFrame::ConstPtr& frame,
  const LocalTargetMap::ConstPtr& target,
  const Eigen::Isometry3d& initial_T_map_imu) {
  RegistrationResult result;
  result.backend_name = "cpu_gicp";
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
  factor->set_max_correspondence_distance(options().max_correspondence_distance);
  factor->set_num_threads(options().num_threads);
  graph.add(factor);

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(options().max_iterations);
  lm_params.setAbsoluteErrorTol(0.1);

  try {
    values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();
    result.T_map_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(0).matrix());
    result.residual = factor->error(values);
    result.score = clamp_score(factor->inlier_fraction());
    result.num_inliers = estimate_inliers_from_score(result.score, result.num_source_points);
    result.converged = true;
    finalize_result(result, initial_T_map_imu);

    spdlog::debug(
      "CPU GICP registration score_type={} residual={:.6f} score={:.3f} inliers={}/{} delta_t={:.3f} delta_r={:.3f} accepted={} reason={}",
      result.score_type,
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

gtsam_points::PointCloudCPU::Ptr CpuGicpMapRegistration::build_target_cloud(const LocalTargetMap::ConstPtr& target) const {
  if (!target || target->empty()) {
    return nullptr;
  }

  return std::const_pointer_cast<gtsam_points::PointCloudCPU>(target->merged_target_cloud());
}

}  // namespace glim_localization
