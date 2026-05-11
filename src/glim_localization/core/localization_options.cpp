#include <glim_localization/core/localization_options.hpp>

#include <cmath>
#include <vector>

#include <glim/util/config.hpp>
#include <glim_localization/core/localization_status.hpp>

namespace glim_localization {
namespace {

Eigen::Vector3d read_vector3(const glim::Config& config, const std::vector<std::string>& path, const std::string& name, const Eigen::Vector3d& fallback) {
  const auto values = config.param_nested<std::vector<double>>(path, name);
  if (!values || values->size() != 3) {
    return fallback;
  }

  return Eigen::Vector3d((*values)[0], (*values)[1], (*values)[2]);
}

}  // namespace

const char* to_string(LocalizationStatus status) {
  switch (status) {
    case LocalizationStatus::WAIT_MAP:
      return "WAIT_MAP";
    case LocalizationStatus::WAIT_INITIAL_POSE:
      return "WAIT_INITIAL_POSE";
    case LocalizationStatus::INITIALIZING:
      return "INITIALIZING";
    case LocalizationStatus::DEGRADED:
      return "DEGRADED";
    case LocalizationStatus::TRACKING:
      return "TRACKING";
    case LocalizationStatus::LOST:
      return "LOST";
    case LocalizationStatus::RELOCALIZING:
      return "RELOCALIZING";
    case LocalizationStatus::RECOVERING:
      return "RECOVERING";
  }

  return "UNKNOWN";
}

Eigen::Isometry3d InitialPoseOptions::T_map_imu() const {
  const Eigen::AngleAxisd roll(rpy.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpy.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpy.z(), Eigen::Vector3d::UnitZ());

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (yaw * pitch * roll).toRotationMatrix();
  T.translation() = xyz;
  return T;
}

LocalizationOptions LocalizationOptions::load() {
  glim::Config config(glim::GlobalConfig::get_config_path("config_localization"));

  LocalizationOptions options;
  options.map.map_path = config.param<std::string>("localization", "map_path", "");
  options.map.load_voxelmaps = config.param_nested<bool>({"localization", "map"}, "load_voxelmaps", options.map.load_voxelmaps);
  options.map.load_raw_frames = config.param_nested<bool>({"localization", "map"}, "load_raw_frames", options.map.load_raw_frames);
  options.map.strict = config.param_nested<bool>({"localization", "map"}, "strict", options.map.strict);
  options.trajectory_path = config.param<std::string>("localization", "trajectory_path", options.trajectory_path);
  options.map_frame = config.param<std::string>("localization", "map_frame", options.map_frame);
  options.odom_frame = config.param<std::string>("localization", "odom_frame", options.odom_frame);
  options.base_frame = config.param<std::string>("localization", "base_frame", options.base_frame);
  options.sensor_frame = config.param<std::string>("localization", "sensor_frame", options.sensor_frame);

  options.initial_pose.source = config.param_nested<std::string>({"localization", "initial_pose"}, "source", options.initial_pose.source);
  options.initial_pose.xyz = read_vector3(config, {"localization", "initial_pose"}, "xyz", options.initial_pose.xyz);
  options.initial_pose.rpy = read_vector3(config, {"localization", "initial_pose"}, "rpy", options.initial_pose.rpy);

  options.target_map.max_num_submaps = config.param_nested<int>({"localization", "target_map"}, "max_num_submaps", options.target_map.max_num_submaps);
  options.target_map.max_distance = config.param_nested<double>({"localization", "target_map"}, "max_distance", options.target_map.max_distance);
  options.target_map.update_distance = config.param_nested<double>({"localization", "target_map"}, "update_distance", options.target_map.update_distance);
  options.target_map.update_angle = config.param_nested<double>({"localization", "target_map"}, "update_angle", options.target_map.update_angle);
  options.target_map.use_submap_index = config.param_nested<bool>({"localization", "target_map"}, "use_submap_index", options.target_map.use_submap_index);
  options.target_map.index_resolution = config.param_nested<double>({"localization", "target_map"}, "index_resolution", options.target_map.index_resolution);

  options.matching.method = config.param_nested<std::string>({"localization", "matching"}, "method", options.matching.method);
  options.matching.max_iterations = config.param_nested<int>({"localization", "matching"}, "max_iterations", options.matching.max_iterations);
  options.matching.min_score = config.param_nested<double>({"localization", "matching"}, "min_score", options.matching.min_score);
  options.matching.min_inliers = config.param_nested<int>({"localization", "matching"}, "min_inliers", options.matching.min_inliers);
  options.matching.max_residual = config.param_nested<double>({"localization", "matching"}, "max_residual", options.matching.max_residual);
  options.matching.max_pose_correction_translation =
    config.param_nested<double>({"localization", "matching"}, "max_pose_correction_translation", options.matching.max_pose_correction_translation);
  options.matching.max_pose_correction_angle =
    config.param_nested<double>({"localization", "matching"}, "max_pose_correction_angle", options.matching.max_pose_correction_angle);
  options.matching.max_consecutive_rejections =
    config.param_nested<int>({"localization", "matching"}, "max_consecutive_rejections", options.matching.max_consecutive_rejections);
  options.matching.max_correspondence_distance =
    config.param_nested<double>({"localization", "matching"}, "max_correspondence_distance", options.matching.max_correspondence_distance);
  options.matching.pose_prior_precision = config.param_nested<double>({"localization", "matching"}, "pose_prior_precision", options.matching.pose_prior_precision);
  options.matching.num_threads = config.param_nested<int>({"localization", "matching"}, "num_threads", options.matching.num_threads);
  options.matching.vgicp_resolution = config.param_nested<double>({"localization", "matching"}, "vgicp_resolution", options.matching.vgicp_resolution);
  options.matching.vgicp_voxelmap_levels = config.param_nested<int>({"localization", "matching"}, "vgicp_voxelmap_levels", options.matching.vgicp_voxelmap_levels);
  options.matching.vgicp_voxelmap_scaling_factor =
    config.param_nested<double>({"localization", "matching"}, "vgicp_voxelmap_scaling_factor", options.matching.vgicp_voxelmap_scaling_factor);

  options.relocalization.enable = config.param_nested<bool>({"localization", "relocalization"}, "enable", options.relocalization.enable);
  options.relocalization.max_candidates =
    config.param_nested<int>({"localization", "relocalization"}, "max_candidates", options.relocalization.max_candidates);
  options.relocalization.num_rings = config.param_nested<int>({"localization", "relocalization"}, "num_rings", options.relocalization.num_rings);
  options.relocalization.num_sectors = config.param_nested<int>({"localization", "relocalization"}, "num_sectors", options.relocalization.num_sectors);
  options.relocalization.min_radius = config.param_nested<double>({"localization", "relocalization"}, "min_radius", options.relocalization.min_radius);
  options.relocalization.max_radius = config.param_nested<double>({"localization", "relocalization"}, "max_radius", options.relocalization.max_radius);
  options.relocalization.max_descriptor_distance =
    config.param_nested<double>({"localization", "relocalization"}, "max_descriptor_distance", options.relocalization.max_descriptor_distance);
  options.relocalization.candidate_translation_weight =
    config.param_nested<double>({"localization", "relocalization"}, "candidate_translation_weight", options.relocalization.candidate_translation_weight);
  options.relocalization.max_candidate_translation_delta =
    config.param_nested<double>({"localization", "relocalization"}, "max_candidate_translation_delta", options.relocalization.max_candidate_translation_delta);
  options.relocalization.verification_target_max_submaps =
    config.param_nested<int>({"localization", "relocalization"}, "verification_target_max_submaps", options.relocalization.verification_target_max_submaps);
  options.relocalization.verification_target_max_distance =
    config.param_nested<double>({"localization", "relocalization"}, "verification_target_max_distance", options.relocalization.verification_target_max_distance);
  options.relocalization.recovery_stable_frames =
    config.param_nested<int>({"localization", "relocalization"}, "recovery_stable_frames", options.relocalization.recovery_stable_frames);
  options.relocalization.verify_raw_topk.enable =
    config.param_nested<bool>({"localization", "relocalization", "verify_raw_topk"}, "enable", options.relocalization.verify_raw_topk.enable);
  options.relocalization.verify_raw_topk.topk =
    config.param_nested<int>({"localization", "relocalization", "verify_raw_topk"}, "topk", options.relocalization.verify_raw_topk.topk);
  options.relocalization.verify_raw_topk.max_descriptor_distance = config.param_nested<double>(
    {"localization", "relocalization", "verify_raw_topk"},
    "max_descriptor_distance",
    options.relocalization.verify_raw_topk.max_descriptor_distance);
  options.relocalization.verify_raw_topk.require_geometric_verification = config.param_nested<bool>(
    {"localization", "relocalization", "verify_raw_topk"},
    "require_geometric_verification",
    options.relocalization.verify_raw_topk.require_geometric_verification);
  options.relocalization.verify_raw_topk.allow_descriptor_rejected_candidates = config.param_nested<bool>(
    {"localization", "relocalization", "verify_raw_topk"},
    "allow_descriptor_rejected_candidates",
    options.relocalization.verify_raw_topk.allow_descriptor_rejected_candidates);
  options.relocalization.recovering.enable =
    config.param_nested<bool>({"localization", "relocalization", "recovering"}, "enable", options.relocalization.recovering.enable);
  options.relocalization.recovering.stable_frames =
    config.param_nested<int>({"localization", "relocalization", "recovering"}, "stable_frames", options.relocalization.recovering.stable_frames);
  options.relocalization.recovering.max_recovery_correction_translation = config.param_nested<double>(
    {"localization", "relocalization", "recovering"},
    "max_recovery_correction_translation",
    options.relocalization.recovering.max_recovery_correction_translation);
  options.relocalization.recovering.max_recovery_correction_angle = config.param_nested<double>(
    {"localization", "relocalization", "recovering"},
    "max_recovery_correction_angle",
    options.relocalization.recovering.max_recovery_correction_angle);
  options.relocalization.recovering.require_no_rejection = config.param_nested<bool>(
    {"localization", "relocalization", "recovering"},
    "require_no_rejection",
    options.relocalization.recovering.require_no_rejection);
  options.relocalization.recovering.write_trajectory_during_recovering = config.param_nested<bool>(
    {"localization", "relocalization", "recovering"},
    "write_trajectory_during_recovering",
    options.relocalization.recovering.write_trajectory_during_recovering);

  options.relocalization_debug.enable =
    config.param_nested<bool>({"localization", "relocalization_debug"}, "enable", options.relocalization_debug.enable);
  options.relocalization_debug.dump_topk =
    config.param_nested<int>({"localization", "relocalization_debug"}, "dump_topk", options.relocalization_debug.dump_topk);
  options.relocalization_debug.dump_rejected_candidates =
    config.param_nested<bool>({"localization", "relocalization_debug"}, "dump_rejected_candidates", options.relocalization_debug.dump_rejected_candidates);
  options.relocalization_debug.verify_rejected_topk =
    config.param_nested<bool>({"localization", "relocalization_debug"}, "verify_rejected_topk", options.relocalization_debug.verify_rejected_topk);
  options.relocalization_debug.verify_rejected_topk_k =
    config.param_nested<int>({"localization", "relocalization_debug"}, "verify_rejected_topk_k", options.relocalization_debug.verify_rejected_topk_k);

  options.debug.csv_enable = config.param_nested<bool>({"localization", "debug"}, "csv_enable", options.debug.csv_enable);
  options.debug.csv_path = config.param_nested<std::string>({"localization", "debug"}, "csv_path", options.debug.csv_path);

  options.smoother_guard.enable = config.param_nested<bool>({"localization", "smoother_guard"}, "enable", options.smoother_guard.enable);
  options.smoother_guard.mode = config.param_nested<std::string>({"localization", "smoother_guard"}, "mode", options.smoother_guard.mode);
  options.smoother_guard.max_lost_frames_without_prior =
    config.param_nested<int>({"localization", "smoother_guard"}, "max_lost_frames_without_prior", options.smoother_guard.max_lost_frames_without_prior);
  options.smoother_guard.max_consecutive_hold_priors =
    config.param_nested<int>({"localization", "smoother_guard"}, "max_consecutive_hold_priors", options.smoother_guard.max_consecutive_hold_priors);
  options.smoother_guard.write_trajectory_during_hold =
    config.param_nested<bool>({"localization", "smoother_guard"}, "write_trajectory_during_hold", options.smoother_guard.write_trajectory_during_hold);
  options.smoother_guard.publish_pose_during_hold =
    config.param_nested<bool>({"localization", "smoother_guard"}, "publish_pose_during_hold", options.smoother_guard.publish_pose_during_hold);

  options.target_rebuild_guard.enable =
    config.param_nested<bool>({"localization", "target_rebuild_guard"}, "enable", options.target_rebuild_guard.enable);
  options.target_rebuild_guard.confirmation_frames =
    config.param_nested<int>({"localization", "target_rebuild_guard"}, "confirmation_frames", options.target_rebuild_guard.confirmation_frames);
  options.target_rebuild_guard.near_threshold_ratio =
    config.param_nested<double>({"localization", "target_rebuild_guard"}, "near_threshold_ratio", options.target_rebuild_guard.near_threshold_ratio);
  options.target_rebuild_guard.force_degraded_on_large_correction =
    config.param_nested<bool>({"localization", "target_rebuild_guard"}, "force_degraded_on_large_correction", options.target_rebuild_guard.force_degraded_on_large_correction);
  options.target_rebuild_guard.consistency_translation =
    config.param_nested<double>({"localization", "target_rebuild_guard"}, "consistency_translation", options.target_rebuild_guard.consistency_translation);
  options.target_rebuild_guard.consistency_angle =
    config.param_nested<double>({"localization", "target_rebuild_guard"}, "consistency_angle", options.target_rebuild_guard.consistency_angle);
  options.target_rebuild_guard.cooldown_frames =
    config.param_nested<int>({"localization", "target_rebuild_guard"}, "cooldown_frames", options.target_rebuild_guard.cooldown_frames);

  options.lost_recovery.enable = config.param_nested<bool>({"localization", "lost_recovery"}, "enable", options.lost_recovery.enable);
  options.lost_recovery.relocalization_period_frames =
    config.param_nested<int>({"localization", "lost_recovery"}, "relocalization_period_frames", options.lost_recovery.relocalization_period_frames);
  options.lost_recovery.recovery_stable_frames =
    config.param_nested<int>({"localization", "lost_recovery"}, "recovery_stable_frames", options.lost_recovery.recovery_stable_frames);
  options.lost_recovery.max_lost_frames_before_pause =
    config.param_nested<int>({"localization", "lost_recovery"}, "max_lost_frames_before_pause", options.lost_recovery.max_lost_frames_before_pause);
  options.lost_recovery.write_trajectory_while_lost =
    config.param_nested<bool>({"localization", "lost_recovery"}, "write_trajectory_while_lost", options.lost_recovery.write_trajectory_while_lost);

  options.ros.publish_tf = config.param_nested<bool>({"localization", "ros"}, "publish_tf", options.ros.publish_tf);
  options.ros.publish_debug_target_map = config.param_nested<bool>({"localization", "ros"}, "publish_debug_target_map", options.ros.publish_debug_target_map);
  options.ros.publish_diagnostics = config.param_nested<bool>({"localization", "ros"}, "publish_diagnostics", options.ros.publish_diagnostics);
  options.ros.initial_pose_topic = config.param_nested<std::string>({"localization", "ros"}, "initial_pose_topic", options.ros.initial_pose_topic);
  options.ros.relocalization_service =
    config.param_nested<std::string>({"localization", "ros"}, "relocalization_service", options.ros.relocalization_service);
  options.ros.status_topic = config.param_nested<std::string>({"localization", "ros"}, "status_topic", options.ros.status_topic);
  options.ros.diagnostic_topic = config.param_nested<std::string>({"localization", "ros"}, "diagnostic_topic", options.ros.diagnostic_topic);
  options.ros.odom_topic = config.param_nested<std::string>({"localization", "ros"}, "odom_topic", options.ros.odom_topic);
  options.ros.pose_topic = config.param_nested<std::string>({"localization", "ros"}, "pose_topic", options.ros.pose_topic);
  options.ros.trajectory_topic = config.param_nested<std::string>({"localization", "ros"}, "trajectory_topic", options.ros.trajectory_topic);
  options.ros.input_scan_topic = config.param_nested<std::string>({"localization", "ros"}, "input_scan_topic", options.ros.input_scan_topic);
  options.ros.current_scan_topic = config.param_nested<std::string>({"localization", "ros"}, "current_scan_topic", options.ros.current_scan_topic);
  options.ros.target_map_topic = config.param_nested<std::string>({"localization", "ros"}, "target_map_topic", options.ros.target_map_topic);
  options.ros.active_submaps_topic = config.param_nested<std::string>({"localization", "ros"}, "active_submaps_topic", options.ros.active_submaps_topic);

  return options;
}

}  // namespace glim_localization
