#pragma once

#include <string>

#include <glim_localization/initialization/initial_pose_options.hpp>
#include <glim_localization/map_loader/map_load_options.hpp>

namespace glim_localization {

struct TargetMapOptions {
  int max_num_submaps = 8;
  double max_distance = 40.0;
  double update_distance = 2.0;
  double update_angle = 0.2;
  bool use_submap_index = true;
  double index_resolution = 20.0;
};

struct MatchingOptions {
  std::string method = "cpu_gicp";
  int max_iterations = 20;
  double min_score = 0.35;
  int min_inliers = 30;
  double max_residual = 1000000.0;
  double max_pose_correction_translation = 3.0;
  double max_pose_correction_angle = 0.7;
  int max_consecutive_rejections = 3;
  double max_correspondence_distance = 2.0;
  double pose_prior_precision = 1000.0;
  int num_threads = 4;
  double vgicp_resolution = 0.5;
  int vgicp_voxelmap_levels = 2;
  double vgicp_voxelmap_scaling_factor = 2.0;
};

struct VerifyRawTopkOptions {
  bool enable = false;
  int topk = 10;
  double max_descriptor_distance = 0.55;
  bool require_geometric_verification = true;
  bool allow_descriptor_rejected_candidates = true;
};

struct RecoveringOptions {
  bool enable = true;
  int stable_frames = 3;
  double max_recovery_correction_translation = 2.0;
  double max_recovery_correction_angle = 0.5;
  bool require_no_rejection = true;
  bool write_trajectory_during_recovering = true;
};

struct RelocalizationOptions {
  bool enable = true;
  int max_candidates = 5;
  int num_rings = 20;
  int num_sectors = 60;
  double min_radius = 1.0;
  double max_radius = 80.0;
  double max_descriptor_distance = 0.35;
  double candidate_translation_weight = 0.01;
  double max_candidate_translation_delta = 80.0;
  int verification_target_max_submaps = 4;
  double verification_target_max_distance = 20.0;
  int recovery_stable_frames = 2;
  VerifyRawTopkOptions verify_raw_topk;
  RecoveringOptions recovering;
};

struct RelocalizationDebugOptions {
  bool enable = false;
  int dump_topk = 10;
  bool dump_rejected_candidates = true;
  bool verify_rejected_topk = false;
  int verify_rejected_topk_k = 5;
};

struct DebugOptions {
  bool csv_enable = false;
  std::string csv_path = "/tmp/glim_localization_debug.csv";
};

struct SmootherGuardOptions {
  bool enable = false;
  std::string mode = "hold_last_pose_prior";
  int max_lost_frames_without_prior = 0;
  int max_consecutive_hold_priors = 30;
  bool write_trajectory_during_hold = false;
  bool publish_pose_during_hold = false;
};

struct TargetRebuildGuardOptions {
  bool enable = false;
  int confirmation_frames = 2;
  double near_threshold_ratio = 0.8;
  bool force_degraded_on_large_correction = true;
  double consistency_translation = 1.0;
  double consistency_angle = 0.3;
  int cooldown_frames = 3;
};

struct LostRecoveryOptions {
  bool enable = false;
  int relocalization_period_frames = 5;
  int recovery_stable_frames = 3;
  int max_lost_frames_before_pause = 3;
  bool write_trajectory_while_lost = false;
};

struct RosOutputOptions {
  bool publish_tf = true;
  bool publish_debug_target_map = true;
  bool publish_diagnostics = true;
  std::string initial_pose_topic = "/initialpose";
  std::string relocalization_service = "/localization/relocalize";
  std::string status_topic = "/localization/status";
  std::string diagnostic_topic = "/localization/diagnostics";
  std::string odom_topic = "/localization/odom";
  std::string pose_topic = "/localization/pose";
  std::string trajectory_topic = "/localization/trajectory";
  std::string input_scan_topic = "/localization/debug/input_scan";
  std::string current_scan_topic = "/localization/debug/current_scan";
  std::string target_map_topic = "/localization/debug/local_target_map";
  std::string active_submaps_topic = "/localization/debug/active_submaps";
};

struct LocalizationOptions {
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  std::string sensor_frame = "lidar";
  std::string trajectory_path = "/tmp/glim_localization_traj.txt";

  MapLoadOptions map;
  InitialPoseOptions initial_pose;
  TargetMapOptions target_map;
  MatchingOptions matching;
  RelocalizationOptions relocalization;
  RelocalizationDebugOptions relocalization_debug;
  DebugOptions debug;
  SmootherGuardOptions smoother_guard;
  TargetRebuildGuardOptions target_rebuild_guard;
  LostRecoveryOptions lost_recovery;
  RosOutputOptions ros;

  static LocalizationOptions load();
};

}  // namespace glim_localization
