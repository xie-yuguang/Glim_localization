#include <glim_localization/registration/gpu_vgicp_map_registration.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <vector>

#include <spdlog/spdlog.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/cuda/cuda_stream.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

namespace glim_localization {

GpuVgicpMapRegistration::GpuVgicpMapRegistration(const MatchingOptions& options) : options_(options) {
  stream_ = std::make_unique<gtsam_points::CUDAStream>();
  stream_buffer_roundrobin_ = std::make_unique<gtsam_points::StreamTempBufferRoundRobin>();
}

GpuVgicpMapRegistration::~GpuVgicpMapRegistration() {}

RegistrationResult GpuVgicpMapRegistration::align(
  const glim::EstimationFrame::ConstPtr& frame,
  const LocalTargetMap::ConstPtr& target,
  const Eigen::Isometry3d& initial_T_map_imu) {
  RegistrationResult result;
  result.T_map_imu = initial_T_map_imu;

  if (!frame || !frame->frame) {
    result.reject_reason = "empty_source";
    return result;
  }

  const auto target_cloud_cpu = build_target_cloud_cpu(target);
  if (!target_cloud_cpu || target_cloud_cpu->size() == 0) {
    result.reject_reason = "empty_target";
    return result;
  }

  result.num_source_points = frame->frame->size();
  result.num_target_points = target_cloud_cpu->size();

  auto source_gpu = std::dynamic_pointer_cast<const gtsam_points::PointCloudGPU>(frame->frame);
  if (!source_gpu) {
    source_gpu = gtsam_points::PointCloudGPU::clone(*frame->frame);
  }
  auto target_gpu = gtsam_points::PointCloudGPU::clone(*target_cloud_cpu);

  gtsam::Values values;
  values.insert(0, gtsam::Pose3(initial_T_map_imu.matrix()));

  gtsam::NonlinearFactorGraph graph;
  const int levels = std::max(1, options_.vgicp_voxelmap_levels);
  for (int i = 0; i < levels; i++) {
    const double resolution = options_.vgicp_resolution * std::pow(options_.vgicp_voxelmap_scaling_factor, i);
    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution, 8192 * 2, 10, 1e-3, *stream_);
    voxelmap->insert(*target_gpu);

    auto stream_buffer = stream_buffer_roundrobin_->get_stream_buffer();
    auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(
      gtsam::Pose3(),
      0,
      voxelmap,
      source_gpu,
      stream_buffer.first,
      stream_buffer.second);
    factor->set_enable_surface_validation(true);
    graph.add(factor);
  }

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(options_.max_iterations);
  lm_params.setAbsoluteErrorTol(0.1);

  try {
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    values = optimizer.optimize();
    result.T_map_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(0).matrix());
    // Reuse the optimizer's final nonlinear error instead of issuing an
    // extra synchronous graph.error(values) on GPU factors.
    result.residual = optimizer.error();
    result.score = std::exp(-std::min(100.0, result.residual / std::max(1, result.num_source_points)));
    result.num_inliers = static_cast<int>(std::lround(result.score * result.num_source_points));
    result.converged = true;
    evaluate_result(result, initial_T_map_imu);
  } catch (const std::exception& e) {
    spdlog::warn("GPU VGICP registration failed: {}", e.what());
    result.reject_reason = "optimizer_exception";
  }

  return result;
}

std::shared_ptr<gtsam_points::PointCloudCPU> GpuVgicpMapRegistration::build_target_cloud_cpu(const LocalTargetMap::ConstPtr& target) const {
  if (!target || target->empty()) {
    return nullptr;
  }

  return std::const_pointer_cast<gtsam_points::PointCloudCPU>(target->merged_target_cloud());
}

void GpuVgicpMapRegistration::evaluate_result(RegistrationResult& result, const Eigen::Isometry3d& initial_T_map_imu) const {
  const Eigen::Isometry3d delta = initial_T_map_imu.inverse() * result.T_map_imu;
  result.pose_delta_translation = delta.translation().norm();
  result.pose_delta_angle = Eigen::AngleAxisd(Eigen::Quaterniond(delta.linear()).normalized()).angle();
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
