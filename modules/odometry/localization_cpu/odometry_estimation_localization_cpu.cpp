#include "odometry_estimation_localization_cpu.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <spdlog/spdlog.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <glim_localization/initialization/runtime_initial_pose.hpp>
#include <glim_localization/map_loader/glim_map_loader.hpp>
#include <glim_localization/registration/cpu_gicp_map_registration.hpp>
#include <glim_localization/relocalization/runtime_relocalization_request.hpp>
#include <glim_localization/relocalization/scan_context_relocalizer.hpp>
#ifdef GLIM_LOCALIZATION_USE_CUDA
#include <glim_localization/registration/gpu_vgicp_map_registration.hpp>
#endif

namespace glim_localization {
namespace {

std::string join_ids(const std::vector<int>& ids) {
  std::ostringstream oss;
  for (int i = 0; i < static_cast<int>(ids.size()); i++) {
    if (i) {
      oss << ",";
    }
    oss << ids[i];
  }
  return oss.str();
}

}  // namespace

using gtsam::symbol_shorthand::X;

OdometryEstimationLocalizationCPUParams::OdometryEstimationLocalizationCPUParams() : glim::OdometryEstimationIMUParams() {
  localization = LocalizationOptions::load();

  ConfigInitialPoseProvider initial_pose_provider(localization.initial_pose);
  if (initial_pose_provider.ready()) {
    // This module uses GLIM's internal world frame as the fixed map frame.
    // Setting init_T_world_imu here initializes the base IMU odometry at
    // T_map_imu from the localization config.
    initialization_mode = "NAIVE";
    estimate_init_state = false;
    init_T_world_imu = initial_pose_provider.T_map_imu();
    init_v_world_imu.setZero();
  } else if (localization.initial_pose.source == "topic") {
    initialization_mode = "NAIVE";
    estimate_init_state = false;
    init_T_world_imu.setIdentity();
    init_v_world_imu.setZero();
  }
}

OdometryEstimationLocalizationCPUParams::~OdometryEstimationLocalizationCPUParams() {}

OdometryEstimationLocalizationCPU::OdometryEstimationLocalizationCPU(const OdometryEstimationLocalizationCPUParams& params)
: glim::OdometryEstimationIMU(std::make_unique<OdometryEstimationLocalizationCPUParams>(params)),
  options_(params.localization),
  status_(LocalizationStatus::WAIT_MAP),
  map_loaded_(false),
  initial_pose_ready_(false),
  target_map_ready_(false),
  consecutive_rejections_(0) {
  map_ = std::make_shared<LocalizationMap>();
  initial_pose_provider_ = std::make_unique<ConfigInitialPoseProvider>(options_.initial_pose);
#ifdef GLIM_LOCALIZATION_USE_CUDA
  if (options_.matching.method == "gpu_vgicp") {
    registration_ = std::make_shared<GpuVgicpMapRegistration>(options_.matching);
    logger->info("using GPU VGICP map registration");
  } else
#endif
  {
    if (options_.matching.method == "gpu_vgicp") {
      logger->warn("GPU VGICP requested but glim_localization was built without CUDA; falling back to CPU GICP");
    }
    registration_ = std::make_shared<CpuGicpMapRegistration>(options_.matching);
  }
  geometric_verifier_ = std::make_unique<GeometricVerifier>(options_.matching, registration_);
  trajectory_writer_ = std::make_unique<TrajectoryWriter>();
  initial_T_map_imu_.setIdentity();
  T_map_odom_.setIdentity();

  if (!options_.trajectory_path.empty()) {
    if (!trajectory_writer_->open(options_.trajectory_path)) {
      logger->warn("failed to open localization trajectory output: {}", options_.trajectory_path);
    } else {
      logger->info("localization trajectory output: {}", trajectory_writer_->path());
    }
  }

  load_map();
  build_relocalizer();
  apply_initial_pose();
}

OdometryEstimationLocalizationCPU::~OdometryEstimationLocalizationCPU() {
  if (trajectory_writer_ && trajectory_writer_->is_open()) {
    logger->info("localization trajectory saved: {} poses -> {}", trajectory_writer_->num_written(), trajectory_writer_->path());
  }
}

glim::EstimationFrame::ConstPtr OdometryEstimationLocalizationCPU::insert_frame(
  const glim::PreprocessedFrame::Ptr& frame,
  std::vector<glim::EstimationFrame::ConstPtr>& marginalized_frames) {
  if (options_.initial_pose.source == "topic" && !initial_pose_ready_) {
    const auto runtime_pose = get_runtime_initial_pose();
    if (!runtime_pose) {
      logger->warn("waiting for {} before starting localization", options_.ros.initial_pose_topic);
      status_ = LocalizationStatus::WAIT_INITIAL_POSE;
      return nullptr;
    }

    initial_T_map_imu_ = runtime_pose->T_map_imu;
    initial_pose_ready_ = true;
    status_ = map_loaded_ ? LocalizationStatus::INITIALIZING : LocalizationStatus::WAIT_MAP;
    logger->info("received runtime initial pose; starting localization");
  }

  return glim::OdometryEstimationIMU::insert_frame(frame, marginalized_frames);
}

void OdometryEstimationLocalizationCPU::create_frame(glim::EstimationFrame::Ptr& frame) {
  // OdometryEstimationIMU has already performed the reusable GLIM frontend work:
  // raw PreprocessedFrame -> deskewed IMU-frame PointCloudCPU with normals/covs.
  // In this localization module, world == map, so these fields are interpreted as:
  //   frame->T_world_imu   == T_map_imu
  //   frame->T_world_lidar == T_map_lidar
  //
  if (options_.initial_pose.source == "topic" && initial_pose_ready_ && frame->id == 0) {
    frame->T_world_imu = initial_T_map_imu_;
    frame->T_world_lidar = frame->T_world_imu * frame->T_lidar_imu.inverse();
    logger->info("applied runtime initial pose at first localization frame stamp={:.6f}", frame->stamp);
  }
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationCPU::create_factors(
  const int current,
  const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor,
  gtsam::Values& new_values) {
  (void)imu_factor;

  if (current < 0 || current >= frames.size() || !frames[current]) {
    return gtsam::NonlinearFactorGraph();
  }

  if (!map_loaded_ || !initial_pose_ready_) {
    logger->warn(
      "localization frontend is not ready (map_loaded={} initial_pose_ready={})",
      map_loaded_,
      initial_pose_ready_);
    return gtsam::NonlinearFactorGraph();
  }

  // The base class predicts frame pose with IMU propagation before calling this
  // hook.  Because world == map in this module, the predicted T_world_imu is the
  // predicted T_map_imu used to query fixed map submaps.
  if (consume_runtime_relocalization_request() || status_ == LocalizationStatus::LOST) {
    return attempt_relocalization(current, new_values);
  }

  const Eigen::Isometry3d predicted_T_map_imu = frames[current]->T_world_imu;
  target_map_ = build_or_update_target_map(predicted_T_map_imu);
  target_map_ready_ = target_map_ && !target_map_->empty();

  if (!target_map_ready_) {
    logger->warn("failed to build target map for frame={} stamp={:.6f}", current, frames[current]->stamp);
    status_ = LocalizationStatus::LOST;
    return gtsam::NonlinearFactorGraph();
  }

  logger->debug(
    "localization target ready frame={} active_submaps={}",
    current,
    target_map_->active_submap_ids().size());

  return create_scan_to_map_factor_or_prior(current, target_map_, new_values);
}

void OdometryEstimationLocalizationCPU::update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) {
  glim::OdometryEstimationIMU::update_frames(current, new_factors);

  if (current >= 0 && current < frames.size() && frames[current]) {
    write_trajectory(frames[current]);
  }
}

void OdometryEstimationLocalizationCPU::load_map() {
  if (options_.map.map_path.empty()) {
    logger->warn("localization.map_path is empty; localization map is not loaded");
    status_ = LocalizationStatus::WAIT_MAP;
    return;
  }

  GlimMapLoader loader;
  map_ = loader.load(options_.map);
  map_loaded_ = map_ && !map_->empty();
  if (!map_loaded_) {
    logger->error("failed to load localization map from {}", options_.map.map_path);
    status_ = LocalizationStatus::WAIT_MAP;
    return;
  }

  logger->info("localization map loaded: {} submaps", map_->size());
  if (options_.target_map.use_submap_index) {
    map_->build_index(options_.target_map.index_resolution);
    const auto stats = map_->index_stats();
    logger->info(
      "localization submap index built: resolution={:.3f} cells={} max_cell_size={}",
      stats.resolution,
      stats.num_cells,
      stats.max_cell_size);
  }
  status_ = LocalizationStatus::WAIT_INITIAL_POSE;
}

void OdometryEstimationLocalizationCPU::build_relocalizer() {
  if (!options_.relocalization.enable) {
    logger->info("relocalization is disabled");
    return;
  }

  if (!map_ || map_->empty()) {
    logger->warn("relocalization build skipped: map is not loaded");
    return;
  }

  relocalizer_ = std::make_shared<ScanContextRelocalizer>(options_.relocalization);
  if (!relocalizer_->build(map_)) {
    relocalizer_.reset();
    logger->warn("failed to build relocalization index");
    return;
  }

  logger->info(
    "relocalization index ready: max_candidates={} max_descriptor_distance={:.3f}",
    options_.relocalization.max_candidates,
    options_.relocalization.max_descriptor_distance);
}

void OdometryEstimationLocalizationCPU::apply_initial_pose() {
  if (initial_pose_provider_ && initial_pose_provider_->ready()) {
    initial_T_map_imu_ = initial_pose_provider_->T_map_imu();
    initial_pose_ready_ = true;
    status_ = map_loaded_ ? LocalizationStatus::INITIALIZING : LocalizationStatus::WAIT_MAP;
    logger->info("using config initial pose for localization (world == map)");
  } else {
    initial_pose_ready_ = false;
    status_ = LocalizationStatus::WAIT_INITIAL_POSE;
    logger->warn("waiting for initial pose source '{}'", options_.initial_pose.source);
  }
}

LocalTargetMap::Ptr OdometryEstimationLocalizationCPU::build_or_update_target_map(const Eigen::Isometry3d& T_map_imu) {
  if (!map_ || map_->empty()) {
    target_map_ready_ = false;
    return std::make_shared<LocalTargetMap>();
  }

  if (!should_update_target_map(T_map_imu)) {
    target_map_->mark_reused();
    logger->debug(
      "reusing localization target map: reuse_count={} delta_t={:.3f}/{:.3f} delta_r={:.3f}/{:.3f} active_submaps=[{}]",
      target_map_->reuse_count(),
      target_map_->distance_from_center(T_map_imu),
      options_.target_map.update_distance,
      target_map_->angle_from_center(T_map_imu),
      options_.target_map.update_angle,
      join_ids(target_map_->active_submap_ids()));
    return target_map_;
  }

  auto target = map_->query_target(T_map_imu, options_.target_map.max_num_submaps, options_.target_map.max_distance);
  target_map_ready_ = target && !target->empty();

  if (target_map_ready_) {
    target->mark_rebuilt();
    logger->info(
      "rebuilt localization target map: center=({:.3f}, {:.3f}, {:.3f}) active_submaps=[{}]",
      target->target_center().x(),
      target->target_center().y(),
      target->target_center().z(),
      join_ids(target->active_submap_ids()));
  }

  return target;
}

bool OdometryEstimationLocalizationCPU::should_update_target_map(const Eigen::Isometry3d& T_map_imu) const {
  if (!target_map_ || target_map_->empty()) {
    return true;
  }

  return target_map_->needs_update(T_map_imu, options_.target_map.update_distance, options_.target_map.update_angle);
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationCPU::attempt_relocalization(const int current, gtsam::Values& new_values) {
  gtsam::NonlinearFactorGraph factors;
  if (!options_.relocalization.enable || !relocalizer_ || !relocalizer_->ready() || !geometric_verifier_) {
    logger->warn("relocalization requested but relocalizer is not ready");
    status_ = LocalizationStatus::LOST;
    return factors;
  }

  if (current < 0 || current >= frames.size() || !frames[current]) {
    status_ = LocalizationStatus::LOST;
    return factors;
  }

  status_ = LocalizationStatus::RELOCALIZING;
  const auto candidates = relocalizer_->query(frames[current], options_.relocalization.max_candidates);
  logger->info("relocalization query frame={} candidates={}", current, candidates.size());
  for (const auto& candidate : candidates) {
    logger->debug(
      "relocalization candidate submap={} descriptor_distance={:.3f} yaw={:.3f}",
      candidate.submap_id,
      candidate.descriptor_distance,
      candidate.yaw);
  }

  if (candidates.empty()) {
    status_ = LocalizationStatus::LOST;
    return factors;
  }

  const auto result = geometric_verifier_->verify(frames[current], candidates);
  if (!result.success) {
    logger->warn("relocalization failed: {}", result.message);
    status_ = LocalizationStatus::LOST;
    return factors;
  }

  Eigen::Isometry3d corrected_T_map_imu = result.registration.T_map_imu;
  corrected_T_map_imu.linear() = Eigen::Quaterniond(corrected_T_map_imu.linear()).normalized().toRotationMatrix();
  const gtsam::Pose3 corrected_pose(corrected_T_map_imu.matrix());

  frames[current]->T_world_imu = corrected_T_map_imu;
  frames[current]->T_world_lidar = corrected_T_map_imu * frames[current]->T_lidar_imu.inverse();
  new_values.insert_or_assign(X(current), corrected_pose);

  const auto noise = gtsam::noiseModel::Isotropic::Precision(6, options_.matching.pose_prior_precision);
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), corrected_pose, noise);

  target_map_ = result.target_map;
  if (target_map_) {
    target_map_->mark_rebuilt();
  }
  target_map_ready_ = target_map_ && !target_map_->empty();
  last_registration_result_ = result.registration;
  last_result_.matching_score = result.registration.score;
  consecutive_rejections_ = 0;
  status_ = LocalizationStatus::TRACKING;

  logger->info(
    "relocalization succeeded frame={} submap={} score={:.3f} residual={:.6f} active_submaps=[{}]",
    current,
    result.candidate.submap_id,
    result.registration.score,
    result.registration.residual,
    target_map_ ? join_ids(target_map_->active_submap_ids()) : "");

  return factors;
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationCPU::create_scan_to_map_factor_or_prior(
  const int current,
  const LocalTargetMap::ConstPtr& target,
  gtsam::Values& new_values) {
  gtsam::NonlinearFactorGraph factors;
  if (!target || target->empty()) {
    status_ = LocalizationStatus::LOST;
    return factors;
  }

  // MVP scan-to-map path:
  //   1. Use frames[current] as the current scan. Its pose fields are T_map_*.
  //   2. Use target->submaps()/target->target_frames() as fixed map input.
  //   3. Run direct CPU GICP alignment.
  //   4. Convert the accepted corrected pose to GTSAM prior/between factors.
  if (!registration_) {
    status_ = LocalizationStatus::LOST;
    return factors;
  }

  const Eigen::Isometry3d predicted_T_map_imu = frames[current]->T_world_imu;
  last_registration_result_ = registration_->align(frames[current], target, predicted_T_map_imu);
  last_result_.matching_score = last_registration_result_.score;

  if (!last_registration_result_.accepted) {
    consecutive_rejections_++;
    const int max_consecutive_rejections = std::max(1, options_.matching.max_consecutive_rejections);
    logger->warn(
      "scan-to-map registration rejected frame={} reason={} score={:.3f}/{:.3f} residual={:.6f}/{:.6f} inliers={}/{} delta_t={:.3f}/{:.3f} delta_r={:.3f}/{:.3f} consecutive_rejections={}/{}",
      current,
      last_registration_result_.reject_reason.empty() ? "unknown" : last_registration_result_.reject_reason,
      last_registration_result_.score,
      options_.matching.min_score,
      last_registration_result_.residual,
      options_.matching.max_residual,
      last_registration_result_.num_inliers,
      options_.matching.min_inliers,
      last_registration_result_.pose_delta_translation,
      options_.matching.max_pose_correction_translation,
      last_registration_result_.pose_delta_angle,
      options_.matching.max_pose_correction_angle,
      consecutive_rejections_,
      max_consecutive_rejections);
    if (consecutive_rejections_ >= max_consecutive_rejections) {
      status_ = LocalizationStatus::LOST;
      return attempt_relocalization(current, new_values);
    }

    status_ = LocalizationStatus::TRACKING;
    return factors;
  }

  consecutive_rejections_ = 0;
  Eigen::Isometry3d corrected_T_map_imu = last_registration_result_.T_map_imu;
  corrected_T_map_imu.linear() = Eigen::Quaterniond(corrected_T_map_imu.linear()).normalized().toRotationMatrix();
  const gtsam::Pose3 corrected_pose(corrected_T_map_imu.matrix());

  frames[current]->T_world_imu = corrected_T_map_imu;
  frames[current]->T_world_lidar = corrected_T_map_imu * frames[current]->T_lidar_imu.inverse();
  new_values.insert_or_assign(X(current), corrected_pose);

  const auto noise = gtsam::noiseModel::Isotropic::Precision(6, options_.matching.pose_prior_precision);
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), corrected_pose, noise);

  if (current > 0 && frames[current - 1]) {
    Eigen::Isometry3d T_last_current = frames[current - 1]->T_world_imu.inverse() * corrected_T_map_imu;
    T_last_current.linear() = Eigen::Quaterniond(T_last_current.linear()).normalized().toRotationMatrix();
    const gtsam::Pose3 relative_pose(T_last_current.matrix());
    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current - 1), X(current), relative_pose, noise);
  }

  logger->debug(
    "scan-to-map registration accepted frame={} score={:.3f} residual={:.6f} inliers={} delta_t={:.3f} delta_r={:.3f} active_submaps=[{}]",
    current,
    last_registration_result_.score,
    last_registration_result_.residual,
    last_registration_result_.num_inliers,
    last_registration_result_.pose_delta_translation,
    last_registration_result_.pose_delta_angle,
    join_ids(target->active_submap_ids()));

  status_ = LocalizationStatus::TRACKING;
  return factors;
}

void OdometryEstimationLocalizationCPU::write_trajectory(const glim::EstimationFrame::ConstPtr& frame) {
  if (!frame) {
    return;
  }

  last_result_.stamp = frame->stamp;
  last_result_.T_map_imu = frame->T_world_imu;
  last_result_.T_map_lidar = frame->T_world_lidar;
  last_result_.T_map_odom = T_map_odom_;
  last_result_.status = status_;
  last_result_.matching_score = last_registration_result_.score;
  last_result_.target_submap_ids = target_map_ ? target_map_->submap_ids() : std::vector<int>();

  auto mutable_frame = std::const_pointer_cast<glim::EstimationFrame>(frame);
  if (mutable_frame) {
    mutable_frame->custom_data[kLocalizationResultCustomDataKey] = std::make_shared<LocalizationResult>(last_result_);
    if (target_map_) {
      mutable_frame->custom_data[kLocalizationTargetMapCustomDataKey] = target_map_;
    }
  }

  if (trajectory_writer_ && trajectory_writer_->is_open()) {
    trajectory_writer_->write(last_result_);
  }
}

}  // namespace glim_localization
