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
  RosOutputOptions ros;

  static LocalizationOptions load();
};

}  // namespace glim_localization
