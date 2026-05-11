#include "odometry_estimation_localization_cpu.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
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

std::string join_ids_csv(const std::vector<int>& ids) {
  std::ostringstream oss;
  for (int i = 0; i < static_cast<int>(ids.size()); i++) {
    if (i) {
      oss << ";";
    }
    oss << ids[i];
  }
  return oss.str();
}

std::string join_ints_csv(const std::vector<int>& values) {
  std::ostringstream oss;
  for (int i = 0; i < static_cast<int>(values.size()); i++) {
    if (i) {
      oss << ";";
    }
    oss << values[i];
  }
  return oss.str();
}

std::string join_doubles_csv(const std::vector<double>& values) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  for (int i = 0; i < static_cast<int>(values.size()); i++) {
    if (i) {
      oss << ";";
    }
    if (std::isfinite(values[i])) {
      oss << values[i];
    } else {
      oss << "nan";
    }
  }
  return oss.str();
}

std::string csv_escape(const std::string& value) {
  if (value.find_first_of(",\"\n\r") == std::string::npos) {
    return value;
  }

  std::string escaped = "\"";
  for (const char ch : value) {
    if (ch == '"') {
      escaped += "\"\"";
    } else {
      escaped += ch;
    }
  }
  escaped += "\"";
  return escaped;
}

double elapsed_ms(const std::chrono::steady_clock::time_point& start, const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

bool same_active_submap_set(const std::vector<int>& lhs, const std::vector<int>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }

  auto sorted_lhs = lhs;
  auto sorted_rhs = rhs;
  std::sort(sorted_lhs.begin(), sorted_lhs.end());
  std::sort(sorted_rhs.begin(), sorted_rhs.end());
  return sorted_lhs == sorted_rhs;
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
  status_reason_("startup"),
  map_loaded_(false),
  initial_pose_ready_(false),
  target_map_ready_(false),
  tracking_started_(false),
  consecutive_rejections_(0),
  stable_tracking_successes_(0),
  recovery_frames_remaining_(0),
  relocalization_attempts_(0),
  last_relocalization_candidate_count_(0),
  last_relocalization_verified_rank_(-1),
  target_map_rebuild_count_(0),
  target_map_recenter_count_(0),
  target_map_reuse_count_(0),
  last_relocalization_message_("idle"),
	  last_relocalization_descriptor_distance_(0.0),
	  last_continuity_translation_(0.0),
	  last_continuity_angle_(0.0),
	  last_continuity_adjusted_(false),
	  last_accepted_pose_ready_(false),
	  lost_frames_without_prior_(0),
	  consecutive_hold_priors_(0),
	  recovering_stable_count_(0),
	  last_relocalization_query_frame_(-1000000),
  target_rebuild_guard_rejections_(0),
  target_rebuild_guard_confirmation_remaining_(0),
  target_rebuild_guard_good_confirmations_(0),
  target_rebuild_guard_pending_valid_(false),
  target_rebuild_guard_pending_delta_t_(std::numeric_limits<double>::quiet_NaN()),
  debug_record_active_(false),
  debug_csv_warned_(false),
  current_frame_hold_prior_(false) {
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
  geometric_verifier_ = std::make_unique<GeometricVerifier>(options_.matching, options_.relocalization, map_, registration_);
  trajectory_writer_ = std::make_unique<TrajectoryWriter>();
  initial_T_map_imu_.setIdentity();
  T_map_odom_.setIdentity();
  last_accepted_T_map_imu_.setIdentity();

  if (!options_.trajectory_path.empty()) {
    if (!trajectory_writer_->open(options_.trajectory_path)) {
      logger->warn("failed to open localization trajectory output: {}", options_.trajectory_path);
    } else {
      logger->info("localization trajectory output: {}", trajectory_writer_->path());
    }
  }

  open_debug_csv();
  load_map();
  build_relocalizer();
  apply_initial_pose();
}

OdometryEstimationLocalizationCPU::~OdometryEstimationLocalizationCPU() {
  if (trajectory_writer_ && trajectory_writer_->is_open()) {
    logger->info("localization trajectory saved: {} poses -> {}", trajectory_writer_->num_written(), trajectory_writer_->path());
  }

  logger->info(
    "localization target map summary: rebuilds={} recenters={} reuses={} target_rebuild_guard_rejections={}",
    target_map_rebuild_count_,
    target_map_recenter_count_,
    target_map_reuse_count_,
    target_rebuild_guard_rejections_);

  if (debug_csv_.is_open()) {
    debug_csv_.flush();
  }
}

glim::EstimationFrame::ConstPtr OdometryEstimationLocalizationCPU::insert_frame(
  const glim::PreprocessedFrame::Ptr& frame,
  std::vector<glim::EstimationFrame::ConstPtr>& marginalized_frames) {
  if (options_.initial_pose.source == "topic" && !initial_pose_ready_) {
    const auto runtime_pose = get_runtime_initial_pose();
    if (!runtime_pose) {
      logger->warn("waiting for {} before starting localization", options_.ros.initial_pose_topic);
      set_status(LocalizationStatus::WAIT_INITIAL_POSE, "waiting_runtime_initial_pose");
      return nullptr;
    }

    initial_T_map_imu_ = runtime_pose->T_map_imu;
    initial_pose_ready_ = true;
    tracking_started_ = false;
    stable_tracking_successes_ = 0;
    recovery_frames_remaining_ = 0;
    recovering_stable_count_ = 0;
    validate_initial_pose(initial_T_map_imu_, "runtime_topic");
    set_status(map_loaded_ ? LocalizationStatus::INITIALIZING : LocalizationStatus::WAIT_MAP, "runtime_initial_pose_ready");
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
  const auto create_factors_started_at = std::chrono::steady_clock::now();

  if (current < 0 || current >= frames.size() || !frames[current]) {
    return gtsam::NonlinearFactorGraph();
  }

  const LocalizationStatus state_before = status_;
  const Eigen::Isometry3d predicted_T_map_imu = frames[current]->T_world_imu;
  current_frame_hold_prior_ = false;
  debug_record_ = LocalizationDebugFrameRecord();
  debug_record_active_ = true;
  debug_record_.frame_id = current;
  debug_record_.timestamp = frames[current]->stamp;
  debug_record_.state_before = to_string(state_before);
  debug_record_.predicted = predicted_T_map_imu.translation();

  auto finish = [&](gtsam::NonlinearFactorGraph factors) {
    finalize_debug_record(current, status_, factors, create_factors_started_at);
    debug_record_active_ = false;
    return factors;
  };

  if (!map_loaded_ || !initial_pose_ready_) {
    logger->warn(
      "localization frontend is not ready (map_loaded={} initial_pose_ready={})",
      map_loaded_,
      initial_pose_ready_);
    return finish(gtsam::NonlinearFactorGraph());
  }

  last_continuity_translation_ = 0.0;
  last_continuity_angle_ = 0.0;
  last_continuity_adjusted_ = false;

  // The base class predicts frame pose with IMU propagation before calling this
  // hook.  Because world == map in this module, the predicted T_world_imu is the
  // predicted T_map_imu used to query fixed map submaps.
  const bool runtime_relocalization_requested = consume_runtime_relocalization_request();
  if (runtime_relocalization_requested || status_ == LocalizationStatus::LOST) {
    debug_record_.relocalization_requested = true;
    debug_record_.relocalization_query_reason = runtime_relocalization_requested ? "runtime_request" : "lost_state";
    if (
      options_.lost_recovery.enable && !runtime_relocalization_requested && status_ == LocalizationStatus::LOST &&
      last_relocalization_query_frame_ >= 0) {
      const int period = std::max(1, options_.lost_recovery.relocalization_period_frames);
      const int max_before_pause = std::max(0, options_.lost_recovery.max_lost_frames_before_pause);
      if (lost_frames_without_prior_ >= max_before_pause && current - last_relocalization_query_frame_ < period) {
        debug_record_.relocalization_requested = false;
        debug_record_.relocalization_query_enable = true;
        debug_record_.relocalization_query_reason = "lost_recovery_period_wait";
        debug_record_.relocalization_failure_reason = "not_requested";
        set_status(LocalizationStatus::LOST, "lost_recovery_wait");
        gtsam::NonlinearFactorGraph factors;
        apply_smoother_hold_guard(current, new_values, factors, "lost_pause_no_candidate");
        return finish(factors);
      }
    }
    return finish(attempt_relocalization(current, new_values));
  }

  const auto target_build_started_at = std::chrono::steady_clock::now();
  target_map_ = build_or_update_target_map(predicted_T_map_imu);
  debug_record_.target_build_ms = elapsed_ms(target_build_started_at, std::chrono::steady_clock::now());
  target_map_ready_ = target_map_ && !target_map_->empty();
  if (target_map_ready_) {
    debug_record_.target_rebuilt = !target_map_->reused_last_time();
    debug_record_.target_reused = target_map_->reused_last_time();
    debug_record_.target_center = target_map_->target_center();
    debug_record_.active_submaps = target_map_->active_submap_ids();
    if (options_.target_rebuild_guard.enable && debug_record_.target_rebuilt) {
      target_rebuild_guard_confirmation_remaining_ = std::max(0, options_.target_rebuild_guard.confirmation_frames) + 1;
      target_rebuild_guard_good_confirmations_ = 0;
      target_rebuild_guard_pending_valid_ = false;
      target_rebuild_guard_pending_delta_t_ = std::numeric_limits<double>::quiet_NaN();
      debug_record_.target_rebuild_guard_action = "confirmation_window_started";
      debug_record_.confirmation_window_remaining = target_rebuild_guard_confirmation_remaining_;
    }
  }

  if (!target_map_ready_) {
    logger->warn("failed to build target map for frame={} stamp={:.6f}", current, frames[current]->stamp);
    set_status(LocalizationStatus::LOST, "target_map_unavailable");
    return finish(gtsam::NonlinearFactorGraph());
  }

  logger->debug(
    "localization target ready frame={} active_submaps={}",
    current,
    target_map_->active_submap_ids().size());

  return finish(create_scan_to_map_factor_or_prior(current, target_map_, new_values));
}

void OdometryEstimationLocalizationCPU::update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) {
  glim::OdometryEstimationIMU::update_frames(current, new_factors);

  if (current >= 0 && current < frames.size() && frames[current]) {
    write_trajectory(frames[current]);
  }
}

void OdometryEstimationLocalizationCPU::set_status(LocalizationStatus status, const std::string& reason) {
  if (status_ != status || status_reason_ != reason) {
    logger->info(
      "localization status transition: {} -> {} reason={}",
      to_string(status_),
      to_string(status),
      reason);
  }
  status_ = status;
  status_reason_ = reason;
}

void OdometryEstimationLocalizationCPU::update_tracking_status_after_accept(const std::string& reason) {
  stable_tracking_successes_++;
  if (recovery_frames_remaining_ > 0) {
    if (!options_.relocalization.recovering.enable) {
      recovery_frames_remaining_ = std::max(0, recovery_frames_remaining_ - 1);
      set_status(
        recovery_frames_remaining_ > 0 ? LocalizationStatus::RECOVERING : LocalizationStatus::TRACKING,
        recovery_frames_remaining_ > 0 ? reason : "recovery_stabilized");
      return;
    }

    const bool stable_correction =
      last_registration_result_.accepted &&
      last_registration_result_.pose_delta_translation <= std::max(0.0, options_.relocalization.recovering.max_recovery_correction_translation) &&
      last_registration_result_.pose_delta_angle <= std::max(0.0, options_.relocalization.recovering.max_recovery_correction_angle) &&
      last_registration_result_.reject_reason.empty();
    if (stable_correction) {
      recovering_stable_count_++;
    } else {
      recovering_stable_count_ = 0;
    }

    const int required = required_recovering_stable_frames();
    recovery_frames_remaining_ = std::max(0, required - recovering_stable_count_);
    set_status(
      recovery_frames_remaining_ > 0 ? LocalizationStatus::RECOVERING : LocalizationStatus::TRACKING,
      recovery_frames_remaining_ > 0 ? (stable_correction ? reason : "recovery_unstable_correction") : "recovery_stabilized");
    return;
  }

  recovering_stable_count_ = 0;
  set_status(tracking_started_ ? LocalizationStatus::TRACKING : LocalizationStatus::INITIALIZING, reason);
}

void OdometryEstimationLocalizationCPU::load_map() {
  if (options_.map.map_path.empty()) {
    logger->warn("localization.map_path is empty; localization map is not loaded");
    set_status(LocalizationStatus::WAIT_MAP, "map_path_empty");
    return;
  }

  GlimMapLoader loader;
  map_ = loader.load(options_.map);
  map_loaded_ = map_ && !map_->empty();
  if (!map_loaded_) {
    logger->error("failed to load localization map from {}", options_.map.map_path);
    set_status(LocalizationStatus::WAIT_MAP, "map_load_failed");
    return;
  }

  const auto& metadata = map_->metadata();
  const auto stats = map_->stats();
  logger->info(
    "localization map loaded: format={} compatibility={} submaps={}/{} skipped={} points={}",
    metadata.detected_format,
    metadata.compatibility,
    metadata.loaded_submaps,
    metadata.requested_submaps,
    metadata.skipped_submaps,
    stats.num_points);
  if (stats.has_bounds) {
    logger->info(
      "localization map bounds: origin_min=({:.3f}, {:.3f}, {:.3f}) origin_max=({:.3f}, {:.3f}, {:.3f})",
      stats.origin_min.x(),
      stats.origin_min.y(),
      stats.origin_min.z(),
      stats.origin_max.x(),
      stats.origin_max.y(),
      stats.origin_max.z());
  }
  if (options_.target_map.use_submap_index) {
    map_->build_index(options_.target_map.index_resolution);
    const auto stats = map_->index_stats();
    logger->info(
      "localization submap index built: resolution={:.3f} cells={} max_cell_size={}",
      stats.resolution,
      stats.num_cells,
      stats.max_cell_size);
  }
  set_status(LocalizationStatus::WAIT_INITIAL_POSE, "map_ready");
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
    tracking_started_ = false;
    stable_tracking_successes_ = 0;
    recovery_frames_remaining_ = 0;
    recovering_stable_count_ = 0;
    validate_initial_pose(initial_T_map_imu_, "config");
    set_status(map_loaded_ ? LocalizationStatus::INITIALIZING : LocalizationStatus::WAIT_MAP, "config_initial_pose_ready");
    logger->info("using config initial pose for localization (world == map)");
  } else {
    initial_pose_ready_ = false;
    tracking_started_ = false;
    stable_tracking_successes_ = 0;
    recovery_frames_remaining_ = 0;
    recovering_stable_count_ = 0;
    set_status(map_loaded_ ? LocalizationStatus::WAIT_INITIAL_POSE : LocalizationStatus::WAIT_MAP, "initial_pose_not_ready");
    logger->warn("waiting for initial pose source '{}'", options_.initial_pose.source);
  }
}

void OdometryEstimationLocalizationCPU::validate_initial_pose(const Eigen::Isometry3d& T_map_imu, const char* source) {
  if (!map_loaded_ || !map_ || map_->empty()) {
    logger->warn("initial pose diagnostic skipped for source={} because localization map is not ready", source);
    return;
  }

  const auto candidates = map_->query_nearby(T_map_imu, options_.target_map.max_num_submaps, options_.target_map.max_distance);
  if (candidates.empty()) {
    logger->warn(
      "initial pose diagnostic source={} found no nearby submaps within max_distance={:.3f} around ({:.3f}, {:.3f}, {:.3f})",
      source,
      options_.target_map.max_distance,
      T_map_imu.translation().x(),
      T_map_imu.translation().y(),
      T_map_imu.translation().z());
    return;
  }

  std::vector<int> candidate_ids;
  candidate_ids.reserve(candidates.size());
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (const auto& submap : candidates) {
    if (!submap) {
      continue;
    }
    candidate_ids.push_back(submap->id);
    nearest_distance = std::min(nearest_distance, (submap->T_world_origin.translation() - T_map_imu.translation()).norm());
  }

  logger->info(
    "initial pose diagnostic source={} candidates={} nearest_distance={:.3f} active_submaps=[{}]",
    source,
    candidate_ids.size(),
    std::isfinite(nearest_distance) ? nearest_distance : -1.0,
    join_ids(candidate_ids));
}

LocalTargetMap::Ptr OdometryEstimationLocalizationCPU::build_or_update_target_map(const Eigen::Isometry3d& T_map_imu) {
  if (!map_ || map_->empty()) {
    target_map_ready_ = false;
    return std::make_shared<LocalTargetMap>();
  }

  if (!should_update_target_map(T_map_imu)) {
    target_map_->mark_reused();
    target_map_reuse_count_++;
    logger->debug(
      "reusing localization target map: reuse_count={} total_reuses={} delta_t={:.3f}/{:.3f} delta_r={:.3f}/{:.3f} active_submaps=[{}]",
      target_map_->reuse_count(),
      target_map_reuse_count_,
      target_map_->distance_from_center(T_map_imu),
      options_.target_map.update_distance,
      target_map_->angle_from_center(T_map_imu),
      options_.target_map.update_angle,
      join_ids(target_map_->active_submap_ids()));
    return target_map_;
  }

  auto target = map_->query_target(T_map_imu, options_.target_map.max_num_submaps, options_.target_map.max_distance);
  target_map_ready_ = target && !target->empty();

  if (target_map_ready_ && target_map_ && !target_map_->empty() &&
      same_active_submap_set(target->active_submap_ids(), target_map_->active_submap_ids())) {
    target_map_->set_T_map_target(T_map_imu);
    target_map_->mark_reused();
    target_map_recenter_count_++;
    logger->info(
      "recentered localization target map: center=({:.3f}, {:.3f}, {:.3f}) active_submaps=[{}] stats(rebuild/recenter/reuse)={}/{}/{}",
      target_map_->target_center().x(),
      target_map_->target_center().y(),
      target_map_->target_center().z(),
      join_ids(target_map_->active_submap_ids()),
      target_map_rebuild_count_,
      target_map_recenter_count_,
      target_map_reuse_count_);
    return target_map_;
  }

  if (target_map_ready_) {
    target->mark_rebuilt();
    target_map_rebuild_count_++;
    logger->info(
      "rebuilt localization target map: center=({:.3f}, {:.3f}, {:.3f}) active_submaps=[{}] stats(rebuild/recenter/reuse)={}/{}/{}",
      target->target_center().x(),
      target->target_center().y(),
      target->target_center().z(),
      join_ids(target->active_submap_ids()),
      target_map_rebuild_count_,
      target_map_recenter_count_,
      target_map_reuse_count_);
  }

  return target;
}

bool OdometryEstimationLocalizationCPU::should_update_target_map(const Eigen::Isometry3d& T_map_imu) const {
  if (!target_map_ || target_map_->empty()) {
    return true;
  }

  return target_map_->needs_update(T_map_imu, options_.target_map.update_distance, options_.target_map.update_angle);
}

void OdometryEstimationLocalizationCPU::apply_corrected_pose_to_frame(
  const int current,
  const Eigen::Isometry3d& corrected_T_map_imu,
  gtsam::Values& new_values) {
  Eigen::Isometry3d normalized = corrected_T_map_imu;
  normalized.linear() = Eigen::Quaterniond(normalized.linear()).normalized().toRotationMatrix();

  frames[current]->T_world_imu = normalized;
  frames[current]->T_world_lidar = normalized * frames[current]->T_lidar_imu.inverse();
  new_values.insert_or_assign(X(current), gtsam::Pose3(normalized.matrix()));
}

void OdometryEstimationLocalizationCPU::add_graph_injection_factors(
  const int current,
  const Eigen::Isometry3d& corrected_T_map_imu,
  gtsam::NonlinearFactorGraph& factors,
  bool include_between_factor) const {
  const auto noise = gtsam::noiseModel::Isotropic::Precision(6, options_.matching.pose_prior_precision);
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), gtsam::Pose3(corrected_T_map_imu.matrix()), noise);

  if (include_between_factor && current > 0 && frames[current - 1]) {
    Eigen::Isometry3d T_last_current = frames[current - 1]->T_world_imu.inverse() * corrected_T_map_imu;
    T_last_current.linear() = Eigen::Quaterniond(T_last_current.linear()).normalized().toRotationMatrix();
    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(current - 1), X(current), gtsam::Pose3(T_last_current.matrix()), noise);
  }
}

void OdometryEstimationLocalizationCPU::update_map_odom_for_continuity(
  const Eigen::Isometry3d& corrected_T_map_imu,
  const Eigen::Isometry3d& reference_T_map_imu) {
  const Eigen::Isometry3d previous_T_map_odom = T_map_odom_;
  T_map_odom_ = corrected_T_map_imu * reference_T_map_imu.inverse() * previous_T_map_odom;

  const Eigen::Isometry3d delta = previous_T_map_odom.inverse() * T_map_odom_;
  last_continuity_translation_ = delta.translation().norm();
  last_continuity_angle_ = Eigen::AngleAxisd(Eigen::Quaterniond(delta.linear()).normalized()).angle();
  last_continuity_adjusted_ = last_continuity_translation_ > 1e-6 || last_continuity_angle_ > 1e-6;

  logger->info(
    "updated map->odom continuity anchor: delta_t={:.3f} delta_r={:.3f}",
    last_continuity_translation_,
	    last_continuity_angle_);
}

int OdometryEstimationLocalizationCPU::required_recovering_stable_frames() const {
  if (options_.relocalization.recovering.enable) {
    return std::max(0, options_.relocalization.recovering.stable_frames);
  }
  return std::max(0, options_.relocalization.recovery_stable_frames);
}

std::vector<RelocalizationCandidate> OdometryEstimationLocalizationCPU::collect_raw_topk_recovery_candidates() const {
  std::vector<RelocalizationCandidate> candidates;
  if (!options_.relocalization.verify_raw_topk.enable) {
    return candidates;
  }

  const auto scan_context = std::dynamic_pointer_cast<ScanContextRelocalizer>(relocalizer_);
  if (!scan_context) {
    return candidates;
  }

  const auto diagnostics = scan_context->last_query_diagnostics();
  const int limit = std::max(0, options_.relocalization.verify_raw_topk.topk);
  const double max_descriptor_distance = std::max(0.0, options_.relocalization.verify_raw_topk.max_descriptor_distance);
  candidates.reserve(limit);
  for (const auto& candidate_debug : diagnostics.topk) {
    if (limit > 0 && static_cast<int>(candidates.size()) >= limit) {
      break;
    }
    if (!candidate_debug.candidate.submap) {
      continue;
    }
    if (!candidate_debug.passed_translation) {
      continue;
    }
    if (!options_.relocalization.verify_raw_topk.allow_descriptor_rejected_candidates && !candidate_debug.passed_descriptor) {
      continue;
    }
    if (!std::isfinite(candidate_debug.descriptor_distance) || candidate_debug.descriptor_distance > max_descriptor_distance) {
      continue;
    }
    candidates.push_back(candidate_debug.candidate);
  }

  return candidates;
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationCPU::accept_verified_relocalization(
  const int current,
  const RelocalizationResult& result,
  const Eigen::Isometry3d& reference_T_map_imu,
  double registration_ms,
  gtsam::Values& new_values,
  bool verify_raw_topk_used) {
  gtsam::NonlinearFactorGraph factors;
  Eigen::Isometry3d corrected_T_map_imu = result.registration.T_map_imu;
  corrected_T_map_imu.linear() = Eigen::Quaterniond(corrected_T_map_imu.linear()).normalized().toRotationMatrix();
  update_map_odom_for_continuity(corrected_T_map_imu, reference_T_map_imu);
  apply_corrected_pose_to_frame(current, corrected_T_map_imu, new_values);
  add_graph_injection_factors(current, corrected_T_map_imu, factors, false);

  target_map_ = result.target_map;
  if (target_map_) {
    target_map_->mark_rebuilt();
    target_map_rebuild_count_++;
  }
  target_map_ready_ = target_map_ && !target_map_->empty();
  last_registration_result_ = result.registration;
  if (debug_record_active_) {
    debug_record_.relocalization_success = true;
    debug_record_.relocalization_failure_reason = "success";
    debug_record_.descriptor_passed_main_threshold =
      result.candidate.descriptor_distance <= options_.relocalization.max_descriptor_distance;
    debug_record_.descriptor_passed_raw_topk_experiment =
      result.candidate.descriptor_distance <= options_.relocalization.verify_raw_topk.max_descriptor_distance;
    if (verify_raw_topk_used) {
      debug_record_.verify_raw_topk_success = true;
      debug_record_.verify_raw_topk_best_submap = result.candidate.submap_id;
      debug_record_.verify_raw_topk_best_distance = result.candidate.descriptor_distance;
      debug_record_.verify_raw_topk_best_score = result.registration.score;
      debug_record_.verify_raw_topk_best_inliers = result.registration.num_inliers;
      debug_record_.verify_raw_topk_best_residual = result.registration.residual;
    }
    update_debug_record_from_registration(result.registration, reference_T_map_imu, corrected_T_map_imu, target_map_, registration_ms);
  }
  last_result_.matching_score = result.registration.score;
  consecutive_rejections_ = 0;
  lost_frames_without_prior_ = 0;
  consecutive_hold_priors_ = 0;
  tracking_started_ = true;
  last_accepted_T_map_imu_ = corrected_T_map_imu;
  last_accepted_pose_ready_ = true;
  stable_tracking_successes_ = 0;
  recovering_stable_count_ = 0;
  recovery_frames_remaining_ = required_recovering_stable_frames();
  last_relocalization_message_ = verify_raw_topk_used ? "verify_raw_topk_verified" : result.message;
  last_relocalization_verified_rank_ = result.verified_candidate_rank;
  last_relocalization_descriptor_distance_ = result.candidate.descriptor_distance;
  set_status(
    recovery_frames_remaining_ > 0 ? LocalizationStatus::RECOVERING : LocalizationStatus::TRACKING,
    recovery_frames_remaining_ > 0 ? (verify_raw_topk_used ? "verify_raw_topk_verified" : "relocalization_verified") : "relocalization_recovered");

  logger->info(
    "relocalization verified frame={} source={} backend={} score_type={} submap={} rank={} descriptor_distance={:.3f} score={:.3f} residual={:.6f} active_submaps=[{}]",
    current,
    verify_raw_topk_used ? "raw_topk_experiment" : "descriptor_threshold",
    result.registration.backend_name,
    result.registration.score_type,
    result.candidate.submap_id,
    result.verified_candidate_rank,
    result.candidate.descriptor_distance,
    result.registration.score,
    result.registration.residual,
    target_map_ ? join_ids(target_map_->active_submap_ids()) : "");

  return factors;
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationCPU::attempt_relocalization(const int current, gtsam::Values& new_values) {
  gtsam::NonlinearFactorGraph factors;
  last_relocalization_query_frame_ = current;
  if (debug_record_active_) {
    debug_record_.relocalization_requested = true;
    debug_record_.relocalization_query_enable = options_.relocalization.enable;
    if (debug_record_.relocalization_query_reason.empty()) {
      debug_record_.relocalization_query_reason = "relocalization_attempt";
    }
  }
  if (!options_.relocalization.enable || !relocalizer_ || !relocalizer_->ready() || !geometric_verifier_) {
    logger->warn("relocalization requested but relocalizer is not ready");
    last_relocalization_message_ = "relocalizer_not_ready";
    if (debug_record_active_) {
      debug_record_.relocalization_failure_reason =
        !options_.relocalization.enable ? "disabled" : (!relocalizer_ || !relocalizer_->ready() ? "database_empty" : "unknown");
    }
    set_status(LocalizationStatus::LOST, "relocalizer_not_ready");
    return factors;
  }

  if (current < 0 || current >= frames.size() || !frames[current]) {
    last_relocalization_message_ = "invalid_frame";
    set_status(LocalizationStatus::LOST, "relocalization_invalid_frame");
    return factors;
  }

  relocalization_attempts_++;
  last_relocalization_candidate_count_ = 0;
  last_relocalization_verified_rank_ = -1;
  last_relocalization_descriptor_distance_ = 0.0;
  last_relocalization_message_ = "querying";
  set_status(LocalizationStatus::RELOCALIZING, "relocalization_query");
  const auto candidates = relocalizer_->query(frames[current], options_.relocalization.max_candidates);
  if (options_.relocalization_debug.enable) {
    fill_relocalization_debug_from_scan_context(candidates.empty() ? "unknown" : "success");
  } else if (debug_record_active_) {
    debug_record_.relocalization_failure_reason = candidates.empty() ? "no_candidates" : "success";
  }
	  last_relocalization_candidate_count_ = static_cast<int>(candidates.size());
	  if (debug_record_active_) {
	    debug_record_.relocalization_candidates = last_relocalization_candidate_count_;
	    debug_record_.relocalization_candidates_after_filter = last_relocalization_candidate_count_;
	    debug_record_.verify_raw_topk_enable = options_.relocalization.verify_raw_topk.enable;
	  }
	  logger->info("relocalization query frame={} candidates={}", current, candidates.size());
  for (int i = 0; i < static_cast<int>(candidates.size()); i++) {
    const auto& candidate = candidates[i];
    logger->debug(
      "relocalization candidate rank={} submap={} descriptor_distance={:.3f} translation_distance={:.3f} ranking_score={:.3f} yaw={:.3f}",
      i,
      candidate.submap_id,
      candidate.descriptor_distance,
      candidate.translation_distance,
      candidate.ranking_score,
      candidate.yaw);
  }

	  if (candidates.empty()) {
	    std::vector<RelocalizationCandidate> raw_topk_candidates;
	    if (options_.relocalization.verify_raw_topk.enable) {
	      raw_topk_candidates = collect_raw_topk_recovery_candidates();
	      if (debug_record_active_) {
	        debug_record_.verify_raw_topk_used = !raw_topk_candidates.empty();
	        debug_record_.verify_raw_topk_candidate_count = static_cast<int>(raw_topk_candidates.size());
	      }
	    }

	    if (!raw_topk_candidates.empty()) {
	      if (!options_.relocalization.verify_raw_topk.require_geometric_verification) {
	        logger->warn("verify_raw_topk.require_geometric_verification=false is ignored; geometric verification remains mandatory");
	      }
	      const auto registration_started_at = std::chrono::steady_clock::now();
	      if (debug_record_active_) {
	        debug_record_.relocalization_verification_attempted = true;
	      }
	      const auto result = geometric_verifier_->verify(frames[current], raw_topk_candidates);
	      const double registration_ms = elapsed_ms(registration_started_at, std::chrono::steady_clock::now());
	      if (debug_record_active_) {
	        debug_record_.relocalization_verification_success = result.success;
	        debug_record_.relocalization_verification_best_submap = result.candidate.submap_id;
	        debug_record_.relocalization_verification_best_score = result.registration.score;
	        debug_record_.relocalization_verification_best_inliers = result.registration.num_inliers;
	        debug_record_.relocalization_verification_best_residual = result.registration.residual;
	        debug_record_.verify_raw_topk_best_submap = result.candidate.submap_id;
	        debug_record_.verify_raw_topk_best_distance = result.candidate.descriptor_distance;
	        debug_record_.verify_raw_topk_best_score = result.registration.score;
	        debug_record_.verify_raw_topk_best_inliers = result.registration.num_inliers;
	        debug_record_.verify_raw_topk_best_residual = result.registration.residual;
	        debug_record_.verify_raw_topk_success = result.success;
	        debug_record_.descriptor_passed_main_threshold =
	          result.candidate.descriptor_distance <= options_.relocalization.max_descriptor_distance;
	        debug_record_.descriptor_passed_raw_topk_experiment =
	          result.candidate.descriptor_distance <= options_.relocalization.verify_raw_topk.max_descriptor_distance;
	      }

	      if (result.success) {
	        return accept_verified_relocalization(current, result, frames[current]->T_world_imu, registration_ms, new_values, true);
	      }

	      logger->warn("verify_raw_topk recovery verification failed frame={} message={}", current, result.message);
	      last_relocalization_message_ = result.message;
	      last_registration_result_ = result.registration;
	      if (debug_record_active_) {
	        debug_record_.relocalization_failure_reason = "verify_raw_topk_verification_failed";
	        update_debug_record_from_registration(
	          result.registration,
	          frames[current]->T_world_imu,
	          result.registration.T_map_imu,
	          result.target_map,
	          registration_ms);
	      }
	    } else {
	      verify_rejected_topk_for_debug(current);
	    }

	    last_relocalization_message_ = debug_record_active_ && !debug_record_.relocalization_failure_reason.empty()
	      ? debug_record_.relocalization_failure_reason
	      : "no_candidates";
    set_status(LocalizationStatus::LOST, "relocalization_no_candidates");
    apply_smoother_hold_guard(current, new_values, factors, options_.lost_recovery.enable ? "lost_pause_no_candidate" : "relocalization_no_candidates");
    return factors;
  }

  const auto registration_started_at = std::chrono::steady_clock::now();
  if (debug_record_active_) {
    debug_record_.relocalization_verification_attempted = true;
  }
  const auto result = geometric_verifier_->verify(frames[current], candidates);
  const double registration_ms = elapsed_ms(registration_started_at, std::chrono::steady_clock::now());
  if (debug_record_active_) {
    debug_record_.relocalization_verification_success = result.success;
    debug_record_.relocalization_verification_best_submap = result.candidate.submap_id;
    debug_record_.relocalization_verification_best_score = result.registration.score;
    debug_record_.relocalization_verification_best_inliers = result.registration.num_inliers;
    debug_record_.relocalization_verification_best_residual = result.registration.residual;
  }
  if (!result.success) {
    logger->warn("relocalization failed: {}", result.message);
    last_relocalization_message_ = result.message;
    last_registration_result_ = result.registration;
    if (debug_record_active_) {
      debug_record_.relocalization_failure_reason = "verification_failed";
      update_debug_record_from_registration(
        result.registration,
        frames[current]->T_world_imu,
        result.registration.T_map_imu,
        result.target_map,
        registration_ms);
    }
    set_status(LocalizationStatus::LOST, "relocalization_verification_failed");
    return factors;
  }

	  return accept_verified_relocalization(current, result, frames[current]->T_world_imu, registration_ms, new_values, false);
}

gtsam::NonlinearFactorGraph OdometryEstimationLocalizationCPU::create_scan_to_map_factor_or_prior(
  const int current,
  const LocalTargetMap::ConstPtr& target,
  gtsam::Values& new_values) {
  gtsam::NonlinearFactorGraph factors;
  if (!target || target->empty()) {
    set_status(LocalizationStatus::LOST, "target_map_empty");
    return factors;
  }

  // MVP scan-to-map path:
  //   1. Use frames[current] as the current scan. Its pose fields are T_map_*.
  //   2. Use target->submaps()/target->target_frames() as fixed map input.
  //   3. Run direct CPU GICP alignment.
  //   4. Convert the accepted corrected pose to GTSAM prior/between factors.
  if (!registration_) {
    set_status(LocalizationStatus::LOST, "registration_backend_missing");
    return factors;
  }

  const Eigen::Isometry3d predicted_T_map_imu = frames[current]->T_world_imu;
  const auto registration_started_at = std::chrono::steady_clock::now();
  last_registration_result_ = registration_->align(frames[current], target, predicted_T_map_imu);
  const double registration_ms = elapsed_ms(registration_started_at, std::chrono::steady_clock::now());
  last_result_.matching_score = last_registration_result_.score;

  apply_target_rebuild_confirmation_guard(current, target);

  update_debug_record_from_registration(
    last_registration_result_,
    predicted_T_map_imu,
    last_registration_result_.T_map_imu,
    target,
    registration_ms);

  if (!last_registration_result_.accepted) {
    consecutive_rejections_++;
    const int max_consecutive_rejections = std::max(1, options_.matching.max_consecutive_rejections);
    logger->warn(
      "scan-to-map registration rejected frame={} state={} backend={} score_type={} reason={} score={:.3f}/{:.3f} residual={:.6f}/{:.6f} inliers={}/{} source_points={} target_points={} delta_t={:.3f}/{:.3f} delta_r={:.3f}/{:.3f} active_submaps=[{}] consecutive_rejections={}/{}",
      current,
      to_string(status_),
      last_registration_result_.backend_name,
      last_registration_result_.score_type,
      last_registration_result_.reject_reason.empty() ? "unknown" : last_registration_result_.reject_reason,
      last_registration_result_.score,
      options_.matching.min_score,
      last_registration_result_.residual,
      options_.matching.max_residual,
      last_registration_result_.num_inliers,
      options_.matching.min_inliers,
      last_registration_result_.num_source_points,
      last_registration_result_.num_target_points,
      last_registration_result_.pose_delta_translation,
      options_.matching.max_pose_correction_translation,
      last_registration_result_.pose_delta_angle,
      options_.matching.max_pose_correction_angle,
      join_ids(target->active_submap_ids()),
      consecutive_rejections_,
      max_consecutive_rejections);
	    if (consecutive_rejections_ >= max_consecutive_rejections) {
	      set_status(LocalizationStatus::LOST, "registration_rejections_exceeded");
	      if (debug_record_active_) {
	        debug_record_.relocalization_requested = true;
	        debug_record_.relocalization_query_reason = "registration_rejections_exceeded";
	      }
	      return attempt_relocalization(current, new_values);
	    }

	    if (tracking_started_) {
	      if (recovery_frames_remaining_ > 0 && options_.relocalization.recovering.enable) {
	        recovering_stable_count_ = 0;
	        recovery_frames_remaining_ = required_recovering_stable_frames();
	      }
	      set_status(
	        recovery_frames_remaining_ > 0 ? LocalizationStatus::RECOVERING : LocalizationStatus::DEGRADED,
	        recovery_frames_remaining_ > 0 ? "recovery_registration_rejected" : last_registration_result_.reject_reason);
    } else {
      set_status(LocalizationStatus::INITIALIZING, "initial_registration_rejected");
    }
    return factors;
  }

  consecutive_rejections_ = 0;
  consecutive_hold_priors_ = 0;
  Eigen::Isometry3d corrected_T_map_imu = last_registration_result_.T_map_imu;
  corrected_T_map_imu.linear() = Eigen::Quaterniond(corrected_T_map_imu.linear()).normalized().toRotationMatrix();
  apply_corrected_pose_to_frame(current, corrected_T_map_imu, new_values);
  add_graph_injection_factors(current, corrected_T_map_imu, factors, true);
  tracking_started_ = true;
  last_accepted_T_map_imu_ = corrected_T_map_imu;
  last_accepted_pose_ready_ = true;
  lost_frames_without_prior_ = 0;
  update_tracking_status_after_accept(recovery_frames_remaining_ > 0 ? "recovery_registration_accepted" : "tracking_registration_accepted");

  logger->info(
    "scan-to-map registration accepted frame={} state={} backend={} score_type={} score={:.3f} residual={:.6f} inliers={} source_points={} target_points={} delta_t={:.3f} delta_r={:.3f} active_submaps=[{}]",
    current,
    to_string(status_),
    last_registration_result_.backend_name,
    last_registration_result_.score_type,
    last_registration_result_.score,
    last_registration_result_.residual,
    last_registration_result_.num_inliers,
    last_registration_result_.num_source_points,
    last_registration_result_.num_target_points,
    last_registration_result_.pose_delta_translation,
    last_registration_result_.pose_delta_angle,
    join_ids(target->active_submap_ids()));
  return factors;
}

bool OdometryEstimationLocalizationCPU::apply_target_rebuild_confirmation_guard(
  const int current,
  const LocalTargetMap::ConstPtr& target) {
  if (!options_.target_rebuild_guard.enable || !target || target->empty()) {
    return false;
  }

  const bool in_confirmation_window = !target->reused_last_time() || target_rebuild_guard_confirmation_remaining_ > 0;
  if (!in_confirmation_window) {
    target_rebuild_guard_good_confirmations_ = 0;
    target_rebuild_guard_pending_valid_ = false;
    return false;
  }

  if (debug_record_active_) {
    debug_record_.confirmation_window_remaining = target_rebuild_guard_confirmation_remaining_;
  }

  if (!last_registration_result_.accepted) {
    if (target_rebuild_guard_confirmation_remaining_ > 0) {
      target_rebuild_guard_confirmation_remaining_--;
    }
    return false;
  }

  const double ratio = std::max(0.0, options_.target_rebuild_guard.near_threshold_ratio);
  const double threshold = options_.matching.max_pose_correction_translation * ratio;
  const double delta_t = last_registration_result_.pose_delta_translation;

  if (delta_t <= threshold) {
    if (target_rebuild_guard_confirmation_remaining_ > 0) {
      target_rebuild_guard_good_confirmations_++;
      if (target_rebuild_guard_good_confirmations_ >= std::max(1, options_.target_rebuild_guard.confirmation_frames)) {
        target_rebuild_guard_confirmation_remaining_ = 0;
        target_rebuild_guard_pending_valid_ = false;
        if (debug_record_active_ && debug_record_.target_rebuild_guard_action.empty()) {
          debug_record_.target_rebuild_guard_action = "confirmed_small_corrections";
        }
      } else {
        target_rebuild_guard_confirmation_remaining_--;
        if (debug_record_active_ && debug_record_.target_rebuild_guard_action.empty()) {
          debug_record_.target_rebuild_guard_action = "confirmation_small_correction";
        }
      }
    }
    if (debug_record_active_) {
      debug_record_.confirmation_window_remaining = target_rebuild_guard_confirmation_remaining_;
    }
    return false;
  }

  bool pending_consistent = false;
  if (target_rebuild_guard_pending_valid_) {
    const Eigen::Isometry3d delta = target_rebuild_guard_pending_pose_.inverse() * last_registration_result_.T_map_imu;
    const double translation = delta.translation().norm();
    const double angle = Eigen::AngleAxisd(Eigen::Quaterniond(delta.linear()).normalized()).angle();
    pending_consistent =
      translation <= std::max(0.0, options_.target_rebuild_guard.consistency_translation) &&
      angle <= std::max(0.0, options_.target_rebuild_guard.consistency_angle);
  }

  target_rebuild_guard_pending_pose_ = last_registration_result_.T_map_imu;
  target_rebuild_guard_pending_delta_t_ = delta_t;
  target_rebuild_guard_pending_valid_ = true;
  target_rebuild_guard_good_confirmations_ = 0;
  target_rebuild_guard_confirmation_remaining_ =
    std::max(target_rebuild_guard_confirmation_remaining_ - 1, std::max(0, options_.target_rebuild_guard.cooldown_frames));

  last_registration_result_.accepted = false;
  last_registration_result_.reject_reason =
    pending_consistent ? "target_rebuild_confirmation_pending" : "target_rebuild_large_correction";
  target_rebuild_guard_rejections_++;

  if (debug_record_active_) {
    debug_record_.target_rebuild_guard_action =
      pending_consistent ? "pending_large_correction_consistent" : "pending_large_correction";
    debug_record_.confirmation_window_remaining = target_rebuild_guard_confirmation_remaining_;
    debug_record_.pending_correction_delta_t = target_rebuild_guard_pending_delta_t_;
    debug_record_.pending_correction_consistent = pending_consistent;
  }

  logger->warn(
    "target rebuild confirmation guard rejected frame={} reason={} delta_t={:.3f}/{:.3f} ratio={:.3f} pending_consistent={} window_remaining={} active_submaps=[{}]",
    current,
    last_registration_result_.reject_reason,
    delta_t,
    options_.matching.max_pose_correction_translation,
    ratio,
    pending_consistent,
    target_rebuild_guard_confirmation_remaining_,
    join_ids(target->active_submap_ids()));
  return true;
}

void OdometryEstimationLocalizationCPU::fill_relocalization_debug_from_scan_context(const std::string& fallback_reason) {
  if (!debug_record_active_) {
    return;
  }

  const auto scan_context = std::dynamic_pointer_cast<ScanContextRelocalizer>(relocalizer_);
  if (!scan_context) {
    debug_record_.relocalization_failure_reason = fallback_reason;
    return;
  }

  const auto diagnostics = scan_context->last_query_diagnostics();
  debug_record_.relocalization_query_enable = diagnostics.query_enable;
  debug_record_.relocalization_query_points = diagnostics.query_points;
  debug_record_.relocalization_descriptor_valid = diagnostics.descriptor_valid;
  debug_record_.relocalization_descriptor_nonempty_bins = diagnostics.descriptor_nonempty_bins;
  debug_record_.relocalization_database_size = diagnostics.database_size;
  debug_record_.relocalization_topk_requested = std::max(1, options_.relocalization_debug.dump_topk);
  debug_record_.relocalization_topk_returned = std::min<int>(diagnostics.topk_returned, debug_record_.relocalization_topk_requested);
  debug_record_.relocalization_filtered_by_descriptor = diagnostics.filtered_by_descriptor;
  debug_record_.relocalization_filtered_by_translation = diagnostics.filtered_by_translation;
  debug_record_.relocalization_filtered_by_other = diagnostics.filtered_by_other;
  debug_record_.relocalization_candidates_before_filter = diagnostics.candidates_before_filter;
  debug_record_.relocalization_candidates_after_filter = diagnostics.candidates_after_filter;
  debug_record_.relocalization_failure_reason = diagnostics.failure_reason.empty() ? fallback_reason : diagnostics.failure_reason;

  if (!diagnostics.topk.empty()) {
    debug_record_.relocalization_top1_submap = diagnostics.topk[0].submap_id;
    debug_record_.relocalization_top1_distance = diagnostics.topk[0].descriptor_distance;
    debug_record_.relocalization_top1_yaw = diagnostics.topk[0].yaw;
  }
  if (diagnostics.topk.size() > 1) {
    debug_record_.relocalization_top2_submap = diagnostics.topk[1].submap_id;
    debug_record_.relocalization_top2_distance = diagnostics.topk[1].descriptor_distance;
  }
  if (diagnostics.topk.size() > 2) {
    debug_record_.relocalization_top3_submap = diagnostics.topk[2].submap_id;
    debug_record_.relocalization_top3_distance = diagnostics.topk[2].descriptor_distance;
  }

  std::vector<int> topk_submaps;
  std::vector<double> topk_distances;
  std::vector<double> topk_yaws;
  const int dump_limit = std::max(0, options_.relocalization_debug.dump_topk);
  const int topk_count = std::min<int>(diagnostics.topk.size(), dump_limit);
  topk_submaps.reserve(topk_count);
  topk_distances.reserve(topk_count);
  topk_yaws.reserve(topk_count);
  for (int i = 0; i < topk_count; i++) {
    topk_submaps.push_back(diagnostics.topk[i].submap_id);
    topk_distances.push_back(diagnostics.topk[i].descriptor_distance);
    topk_yaws.push_back(diagnostics.topk[i].yaw);
  }
  debug_record_.relocalization_topk_submaps = join_ints_csv(topk_submaps);
  debug_record_.relocalization_topk_distances = join_doubles_csv(topk_distances);
  debug_record_.relocalization_topk_yaws = join_doubles_csv(topk_yaws);
}

void OdometryEstimationLocalizationCPU::verify_rejected_topk_for_debug(const int current) {
  if (
    !debug_record_active_ || !options_.relocalization_debug.enable || !options_.relocalization_debug.verify_rejected_topk ||
    !geometric_verifier_) {
    return;
  }

  if (current < 0 || current >= frames.size() || !frames[current]) {
    return;
  }

  const auto scan_context = std::dynamic_pointer_cast<ScanContextRelocalizer>(relocalizer_);
  if (!scan_context) {
    return;
  }

  const auto diagnostics = scan_context->last_query_diagnostics();
  std::vector<RelocalizationCandidate> rejected_candidates;
  const int limit = std::max(0, options_.relocalization_debug.verify_rejected_topk_k);
  rejected_candidates.reserve(limit);
  for (const auto& candidate_debug : diagnostics.topk) {
    if (limit > 0 && static_cast<int>(rejected_candidates.size()) >= limit) {
      break;
    }
    if (candidate_debug.passed_descriptor && candidate_debug.passed_translation) {
      continue;
    }
    if (!candidate_debug.candidate.submap) {
      continue;
    }
    rejected_candidates.push_back(candidate_debug.candidate);
  }

  if (rejected_candidates.empty()) {
    return;
  }

  debug_record_.debug_verification_only = true;
  debug_record_.debug_verified_rejected_topk_count = static_cast<int>(rejected_candidates.size());
  const auto result = geometric_verifier_->verify(frames[current], rejected_candidates);
  debug_record_.debug_verified_success = result.success;
  debug_record_.debug_verified_best_submap = result.candidate.submap_id;
  debug_record_.debug_verified_best_descriptor_distance = result.candidate.descriptor_distance;
  debug_record_.debug_verified_best_score = result.registration.score;
  debug_record_.debug_verified_best_inliers = result.registration.num_inliers;
  debug_record_.debug_verified_best_residual = result.registration.residual;

  logger->info(
    "debug-only rejected top-k verification frame={} count={} success={} best_submap={} descriptor_distance={:.3f} score={:.3f} inliers={} residual={:.6f}",
    current,
    rejected_candidates.size(),
    result.success,
    result.candidate.submap_id,
    result.candidate.descriptor_distance,
    result.registration.score,
    result.registration.num_inliers,
    result.registration.residual);
}

void OdometryEstimationLocalizationCPU::open_debug_csv() {
  if (!options_.debug.csv_enable) {
    return;
  }

  if (options_.debug.csv_path.empty()) {
    logger->warn("localization debug CSV requested but localization.debug.csv_path is empty");
    return;
  }

  try {
    const std::filesystem::path csv_path(options_.debug.csv_path);
    if (csv_path.has_parent_path()) {
      std::filesystem::create_directories(csv_path.parent_path());
    }

    debug_csv_.open(options_.debug.csv_path, std::ios::out | std::ios::trunc);
  } catch (const std::exception& e) {
    logger->warn("failed to prepare localization debug CSV path {}: {}", options_.debug.csv_path, e.what());
    return;
  }

  if (!debug_csv_.is_open()) {
    logger->warn("failed to open localization debug CSV: {}", options_.debug.csv_path);
    return;
  }

  debug_csv_
    << "frame_id,timestamp,state_before,state_after,registration_backend,target_rebuilt,target_reused,"
       "target_center_x,target_center_y,target_center_z,active_submaps,source_points,target_points,"
       "predicted_x,predicted_y,predicted_z,corrected_x,corrected_y,corrected_z,delta_t,delta_r,"
       "score,residual,inliers,inlier_fraction,reject_reason,localization_factors,relocalization_requested,"
       "relocalization_candidates,relocalization_success,relocalization_query_enable,relocalization_query_reason,"
       "relocalization_query_points,relocalization_descriptor_valid,relocalization_descriptor_nonempty_bins,"
       "relocalization_database_size,relocalization_topk_requested,relocalization_topk_returned,"
       "relocalization_top1_submap,relocalization_top1_distance,relocalization_top1_yaw,"
       "relocalization_top2_submap,relocalization_top2_distance,relocalization_top3_submap,relocalization_top3_distance,"
       "relocalization_topk_submaps,relocalization_topk_distances,relocalization_topk_yaws,"
       "relocalization_filtered_by_descriptor,relocalization_filtered_by_translation,relocalization_filtered_by_other,"
       "relocalization_candidates_before_filter,relocalization_candidates_after_filter,"
       "relocalization_verification_attempted,relocalization_verification_success,relocalization_verification_best_submap,"
       "relocalization_verification_best_score,relocalization_verification_best_inliers,relocalization_verification_best_residual,"
	       "relocalization_failure_reason,debug_verification_only,debug_verified_rejected_topk_count,"
	       "debug_verified_best_submap,debug_verified_best_descriptor_distance,debug_verified_best_score,"
	       "debug_verified_best_inliers,debug_verified_best_residual,debug_verified_success,"
	       "verify_raw_topk_enable,verify_raw_topk_used,verify_raw_topk_candidate_count,"
	       "verify_raw_topk_best_submap,verify_raw_topk_best_distance,verify_raw_topk_best_score,"
	       "verify_raw_topk_best_inliers,verify_raw_topk_best_residual,verify_raw_topk_success,"
	       "recovery_state,recovering_stable_count,recovering_required_stable_frames,recovering_transition_reason,"
	       "descriptor_passed_main_threshold,descriptor_passed_raw_topk_experiment,"
	       "smoother_guard_action,target_rebuild_guard_action,"
       "confirmation_window_remaining,pending_correction_delta_t,pending_correction_consistent,"
       "registration_ms,target_build_ms,total_create_factors_ms\n";
  debug_csv_.flush();
  logger->info("localization debug CSV output: {}", options_.debug.csv_path);
}

void OdometryEstimationLocalizationCPU::write_debug_csv(const LocalizationDebugFrameRecord& record) {
  if (!options_.debug.csv_enable || !debug_csv_.is_open()) {
    return;
  }

  debug_csv_ << std::fixed << std::setprecision(9)
             << record.frame_id << ','
             << record.timestamp << ','
             << csv_escape(record.state_before) << ','
             << csv_escape(record.state_after) << ','
             << csv_escape(record.registration_backend) << ','
             << (record.target_rebuilt ? 1 : 0) << ','
             << (record.target_reused ? 1 : 0) << ','
             << record.target_center.x() << ','
             << record.target_center.y() << ','
             << record.target_center.z() << ','
             << csv_escape(join_ids_csv(record.active_submaps)) << ','
             << record.source_points << ','
             << record.target_points << ','
             << record.predicted.x() << ','
             << record.predicted.y() << ','
             << record.predicted.z() << ','
             << record.corrected.x() << ','
             << record.corrected.y() << ','
             << record.corrected.z() << ','
             << record.delta_t << ','
             << record.delta_r << ','
             << record.score << ','
             << record.residual << ','
             << record.inliers << ','
             << record.inlier_fraction << ','
             << csv_escape(record.reject_reason) << ','
             << record.localization_factors << ','
             << (record.relocalization_requested ? 1 : 0) << ','
             << record.relocalization_candidates << ','
             << (record.relocalization_success ? 1 : 0) << ','
             << (record.relocalization_query_enable ? 1 : 0) << ','
             << csv_escape(record.relocalization_query_reason) << ','
             << record.relocalization_query_points << ','
             << (record.relocalization_descriptor_valid ? 1 : 0) << ','
             << record.relocalization_descriptor_nonempty_bins << ','
             << record.relocalization_database_size << ','
             << record.relocalization_topk_requested << ','
             << record.relocalization_topk_returned << ','
             << record.relocalization_top1_submap << ','
             << record.relocalization_top1_distance << ','
             << record.relocalization_top1_yaw << ','
             << record.relocalization_top2_submap << ','
             << record.relocalization_top2_distance << ','
             << record.relocalization_top3_submap << ','
             << record.relocalization_top3_distance << ','
             << csv_escape(record.relocalization_topk_submaps) << ','
             << csv_escape(record.relocalization_topk_distances) << ','
             << csv_escape(record.relocalization_topk_yaws) << ','
             << record.relocalization_filtered_by_descriptor << ','
             << record.relocalization_filtered_by_translation << ','
             << record.relocalization_filtered_by_other << ','
             << record.relocalization_candidates_before_filter << ','
             << record.relocalization_candidates_after_filter << ','
             << (record.relocalization_verification_attempted ? 1 : 0) << ','
             << (record.relocalization_verification_success ? 1 : 0) << ','
             << record.relocalization_verification_best_submap << ','
             << record.relocalization_verification_best_score << ','
             << record.relocalization_verification_best_inliers << ','
             << record.relocalization_verification_best_residual << ','
             << csv_escape(record.relocalization_failure_reason) << ','
             << (record.debug_verification_only ? 1 : 0) << ','
             << record.debug_verified_rejected_topk_count << ','
             << record.debug_verified_best_submap << ','
             << record.debug_verified_best_descriptor_distance << ','
             << record.debug_verified_best_score << ','
	             << record.debug_verified_best_inliers << ','
	             << record.debug_verified_best_residual << ','
	             << (record.debug_verified_success ? 1 : 0) << ','
	             << (record.verify_raw_topk_enable ? 1 : 0) << ','
	             << (record.verify_raw_topk_used ? 1 : 0) << ','
	             << record.verify_raw_topk_candidate_count << ','
	             << record.verify_raw_topk_best_submap << ','
	             << record.verify_raw_topk_best_distance << ','
	             << record.verify_raw_topk_best_score << ','
	             << record.verify_raw_topk_best_inliers << ','
	             << record.verify_raw_topk_best_residual << ','
	             << (record.verify_raw_topk_success ? 1 : 0) << ','
	             << csv_escape(record.recovery_state) << ','
	             << record.recovering_stable_count << ','
	             << record.recovering_required_stable_frames << ','
	             << csv_escape(record.recovering_transition_reason) << ','
	             << (record.descriptor_passed_main_threshold ? 1 : 0) << ','
	             << (record.descriptor_passed_raw_topk_experiment ? 1 : 0) << ','
	             << csv_escape(record.smoother_guard_action) << ','
             << csv_escape(record.target_rebuild_guard_action) << ','
             << record.confirmation_window_remaining << ','
             << record.pending_correction_delta_t << ','
             << (record.pending_correction_consistent ? 1 : 0) << ','
             << record.registration_ms << ','
             << record.target_build_ms << ','
             << record.total_create_factors_ms << '\n';

  if (!debug_csv_) {
    if (!debug_csv_warned_) {
      logger->warn("failed while writing localization debug CSV: {}", options_.debug.csv_path);
      debug_csv_warned_ = true;
    }
    debug_csv_.clear();
  }
}

void OdometryEstimationLocalizationCPU::update_debug_record_from_registration(
  const RegistrationResult& result,
  const Eigen::Isometry3d& predicted_T_map_imu,
  const Eigen::Isometry3d& corrected_T_map_imu,
  const LocalTargetMap::ConstPtr& target,
  double registration_ms) {
  if (!debug_record_active_) {
    return;
  }

  debug_record_.registration_backend = result.backend_name;
  debug_record_.source_points = result.num_source_points;
  debug_record_.target_points = result.num_target_points;
  debug_record_.predicted = predicted_T_map_imu.translation();
  debug_record_.corrected = corrected_T_map_imu.translation();
  debug_record_.delta_t = result.pose_delta_translation;
  debug_record_.delta_r = result.pose_delta_angle;
  debug_record_.score = result.score;
  debug_record_.residual = result.residual;
  debug_record_.inliers = result.num_inliers;
  debug_record_.inlier_fraction = result.num_source_points > 0 ? static_cast<double>(result.num_inliers) / static_cast<double>(result.num_source_points) : 0.0;
  debug_record_.reject_reason = result.accepted ? "" : (result.reject_reason.empty() ? "unknown" : result.reject_reason);
  debug_record_.registration_ms = registration_ms;
  if (target && !target->empty()) {
    debug_record_.target_rebuilt = !target->reused_last_time();
    debug_record_.target_reused = target->reused_last_time();
    debug_record_.target_center = target->target_center();
    debug_record_.active_submaps = target->active_submap_ids();
  }
}

void OdometryEstimationLocalizationCPU::finalize_debug_record(
  const int current,
  const LocalizationStatus state_after,
  const gtsam::NonlinearFactorGraph& factors,
  const std::chrono::steady_clock::time_point& started_at) {
  if (!debug_record_active_) {
    return;
  }

	  debug_record_.state_after = to_string(state_after);
	  debug_record_.localization_factors = factors.size();
	  debug_record_.total_create_factors_ms = elapsed_ms(started_at, std::chrono::steady_clock::now());
	  debug_record_.recovery_state = to_string(state_after);
	  debug_record_.recovering_stable_count = recovering_stable_count_;
	  debug_record_.recovering_required_stable_frames = required_recovering_stable_frames();
	  debug_record_.recovering_transition_reason = status_reason_;
	  if (current >= 0 && current < frames.size() && frames[current]) {
    if (!std::isfinite(debug_record_.corrected.x())) {
      debug_record_.corrected = frames[current]->T_world_imu.translation();
    }
  }
  write_debug_csv(debug_record_);
}

bool OdometryEstimationLocalizationCPU::apply_smoother_hold_guard(
  const int current,
  gtsam::Values& new_values,
  gtsam::NonlinearFactorGraph& factors,
  const char* reason) {
  lost_frames_without_prior_++;
  const int max_without_prior = std::max(0, options_.smoother_guard.max_lost_frames_without_prior);

  if (!options_.smoother_guard.enable || lost_frames_without_prior_ <= max_without_prior) {
    consecutive_hold_priors_ = 0;
    return false;
  }

  if (options_.smoother_guard.mode != "hold_last_pose_prior") {
    if (debug_record_active_) {
      debug_record_.smoother_guard_action = "unsupported_mode";
    }
    logger->warn("smoother guard mode '{}' is not supported without upstream insert_frame changes", options_.smoother_guard.mode);
    return false;
  }

  if (!last_accepted_pose_ready_) {
    if (debug_record_active_) {
      debug_record_.smoother_guard_action = "no_last_accepted_pose";
    }
    return false;
  }

  const int max_holds = options_.smoother_guard.max_consecutive_hold_priors;
  if (max_holds >= 0 && consecutive_hold_priors_ >= max_holds) {
    if (debug_record_active_) {
      debug_record_.smoother_guard_action = "hold_last_pose_prior_limit_reached";
    }
    logger->warn(
      "smoother guard hold prior limit reached frame={} max_consecutive_hold_priors={} reason={}",
      current,
      max_holds,
      reason ? reason : "unknown");
    return false;
  }

  apply_corrected_pose_to_frame(current, last_accepted_T_map_imu_, new_values);
  const auto noise = gtsam::noiseModel::Isotropic::Precision(6, options_.matching.pose_prior_precision);
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), gtsam::Pose3(last_accepted_T_map_imu_.matrix()), noise);
  consecutive_hold_priors_++;
  current_frame_hold_prior_ = true;
  if (debug_record_active_) {
    debug_record_.corrected = last_accepted_T_map_imu_.translation();
    debug_record_.smoother_guard_action =
      reason && std::string(reason) == "lost_pause_no_candidate" ? "lost_pause_no_candidate_hold_prior" : "hold_last_pose_prior";
  }

  logger->warn(
    "smoother guard applied frame={} action=hold_last_pose_prior reason={} lost_frames_without_prior={}",
    current,
    reason ? reason : "unknown",
    lost_frames_without_prior_);
  return true;
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
  last_result_.status_reason = status_reason_;
  last_result_.matching_score = last_registration_result_.score;
  last_result_.backend_name = last_registration_result_.backend_name;
  last_result_.score_type = last_registration_result_.score_type;
  last_result_.reject_reason = last_registration_result_.reject_reason;
  last_result_.relocalization_message = last_relocalization_message_;
  last_result_.consecutive_rejections = consecutive_rejections_;
  last_result_.relocalization_attempts = relocalization_attempts_;
  last_result_.relocalization_candidate_count = last_relocalization_candidate_count_;
  last_result_.relocalization_verified_rank = last_relocalization_verified_rank_;
  last_result_.recovery_frames_remaining = recovery_frames_remaining_;
  last_result_.stable_tracking_successes = stable_tracking_successes_;
  last_result_.descriptor_distance = last_relocalization_descriptor_distance_;
  last_result_.pose_delta_translation = last_registration_result_.pose_delta_translation;
  last_result_.pose_delta_angle = last_registration_result_.pose_delta_angle;
  last_result_.continuity_translation = last_continuity_translation_;
  last_result_.continuity_angle = last_continuity_angle_;
  last_result_.continuity_adjusted = last_continuity_adjusted_;
  last_result_.target_submap_ids = target_map_ ? target_map_->submap_ids() : std::vector<int>();

  auto mutable_frame = std::const_pointer_cast<glim::EstimationFrame>(frame);
  if (mutable_frame) {
    mutable_frame->custom_data[kLocalizationResultCustomDataKey] = std::make_shared<LocalizationResult>(last_result_);
    if (target_map_) {
      mutable_frame->custom_data[kLocalizationTargetMapCustomDataKey] = target_map_;
    }
  }

  if (current_frame_hold_prior_ && !options_.smoother_guard.write_trajectory_during_hold) {
    return;
  }

	  if (
	    options_.lost_recovery.enable && !options_.lost_recovery.write_trajectory_while_lost &&
	    (status_ == LocalizationStatus::LOST || status_ == LocalizationStatus::RELOCALIZING)) {
	    return;
	  }

	  if (
	    options_.relocalization.recovering.enable && !options_.relocalization.recovering.write_trajectory_during_recovering &&
	    status_ == LocalizationStatus::RECOVERING) {
	    return;
	  }

	  if (trajectory_writer_ && trajectory_writer_->is_open()) {
    trajectory_writer_->write(last_result_);
  }
}

}  // namespace glim_localization
