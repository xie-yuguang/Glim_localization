#pragma once

#include <chrono>
#include <cstddef>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <glim/odometry/odometry_estimation_imu.hpp>
#include <glim_localization/core/localization_options.hpp>
#include <glim_localization/core/localization_status.hpp>
#include <glim_localization/initialization/config_initial_pose_provider.hpp>
#include <glim_localization/map/localization_map.hpp>
#include <glim_localization/map/local_target_map.hpp>
#include <glim_localization/output/localization_result.hpp>
#include <glim_localization/output/trajectory_writer.hpp>
#include <glim_localization/registration/map_registration_base.hpp>
#include <glim_localization/relocalization/geometric_verifier.hpp>
#include <glim_localization/relocalization/relocalizer_base.hpp>

namespace glim_localization {

struct LocalizationDebugFrameRecord {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int frame_id = -1;
  double timestamp = 0.0;
  std::string state_before;
  std::string state_after;
  std::string registration_backend;
  bool target_rebuilt = false;
  bool target_reused = false;
  Eigen::Vector3d target_center = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  std::vector<int> active_submaps;
  int source_points = 0;
  int target_points = 0;
  Eigen::Vector3d predicted = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d corrected = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double delta_t = std::numeric_limits<double>::quiet_NaN();
  double delta_r = std::numeric_limits<double>::quiet_NaN();
  double score = std::numeric_limits<double>::quiet_NaN();
  double residual = std::numeric_limits<double>::quiet_NaN();
  int inliers = 0;
  double inlier_fraction = std::numeric_limits<double>::quiet_NaN();
  std::string reject_reason;
  std::size_t localization_factors = 0;
  bool relocalization_requested = false;
  int relocalization_candidates = 0;
  bool relocalization_success = false;
  bool relocalization_query_enable = false;
  std::string relocalization_query_reason;
  int relocalization_query_points = 0;
  bool relocalization_descriptor_valid = false;
  int relocalization_descriptor_nonempty_bins = 0;
  int relocalization_database_size = 0;
  int relocalization_topk_requested = 0;
  int relocalization_topk_returned = 0;
  int relocalization_top1_submap = -1;
  double relocalization_top1_distance = std::numeric_limits<double>::quiet_NaN();
  double relocalization_top1_yaw = std::numeric_limits<double>::quiet_NaN();
  int relocalization_top2_submap = -1;
  double relocalization_top2_distance = std::numeric_limits<double>::quiet_NaN();
  int relocalization_top3_submap = -1;
  double relocalization_top3_distance = std::numeric_limits<double>::quiet_NaN();
  std::string relocalization_topk_submaps;
  std::string relocalization_topk_distances;
  std::string relocalization_topk_yaws;
  int relocalization_filtered_by_descriptor = 0;
  int relocalization_filtered_by_translation = 0;
  int relocalization_filtered_by_other = 0;
  int relocalization_candidates_before_filter = 0;
  int relocalization_candidates_after_filter = 0;
  bool relocalization_verification_attempted = false;
  bool relocalization_verification_success = false;
  int relocalization_verification_best_submap = -1;
  double relocalization_verification_best_score = std::numeric_limits<double>::quiet_NaN();
  int relocalization_verification_best_inliers = 0;
  double relocalization_verification_best_residual = std::numeric_limits<double>::quiet_NaN();
  std::string relocalization_failure_reason;
  bool debug_verification_only = false;
  int debug_verified_rejected_topk_count = 0;
  int debug_verified_best_submap = -1;
  double debug_verified_best_descriptor_distance = std::numeric_limits<double>::quiet_NaN();
  double debug_verified_best_score = std::numeric_limits<double>::quiet_NaN();
  int debug_verified_best_inliers = 0;
  double debug_verified_best_residual = std::numeric_limits<double>::quiet_NaN();
  bool debug_verified_success = false;
  bool verify_raw_topk_enable = false;
  bool verify_raw_topk_used = false;
  int verify_raw_topk_candidate_count = 0;
  int verify_raw_topk_best_submap = -1;
  double verify_raw_topk_best_distance = std::numeric_limits<double>::quiet_NaN();
  double verify_raw_topk_best_score = std::numeric_limits<double>::quiet_NaN();
  int verify_raw_topk_best_inliers = 0;
  double verify_raw_topk_best_residual = std::numeric_limits<double>::quiet_NaN();
  bool verify_raw_topk_success = false;
  std::string recovery_state;
  int recovering_stable_count = 0;
  int recovering_required_stable_frames = 0;
  std::string recovering_transition_reason;
  bool descriptor_passed_main_threshold = false;
  bool descriptor_passed_raw_topk_experiment = false;
  std::string smoother_guard_action;
  std::string target_rebuild_guard_action;
  int confirmation_window_remaining = 0;
  double pending_correction_delta_t = std::numeric_limits<double>::quiet_NaN();
  bool pending_correction_consistent = false;
  double registration_ms = 0.0;
  double target_build_ms = 0.0;
  double total_create_factors_ms = 0.0;
};

struct OdometryEstimationLocalizationCPUParams : public glim::OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationLocalizationCPUParams();
  ~OdometryEstimationLocalizationCPUParams() override;

  LocalizationOptions localization;
};

class OdometryEstimationLocalizationCPU : public glim::OdometryEstimationIMU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit OdometryEstimationLocalizationCPU(const OdometryEstimationLocalizationCPUParams& params = OdometryEstimationLocalizationCPUParams());
  ~OdometryEstimationLocalizationCPU() override;

  glim::EstimationFrame::ConstPtr insert_frame(
    const glim::PreprocessedFrame::Ptr& frame,
    std::vector<glim::EstimationFrame::ConstPtr>& marginalized_frames) override;

protected:
  void create_frame(glim::EstimationFrame::Ptr& frame) override;
  gtsam::NonlinearFactorGraph create_factors(
    const int current,
    const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor,
    gtsam::Values& new_values) override;
  void update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) override;

private:
  void load_map();
  void apply_initial_pose();
  void build_relocalizer();
  void validate_initial_pose(const Eigen::Isometry3d& T_map_imu, const char* source);
  void set_status(LocalizationStatus status, const std::string& reason);
  void update_tracking_status_after_accept(const std::string& reason);
  LocalTargetMap::Ptr build_or_update_target_map(const Eigen::Isometry3d& T_map_imu);
  bool should_update_target_map(const Eigen::Isometry3d& T_map_imu) const;
  void apply_corrected_pose_to_frame(const int current, const Eigen::Isometry3d& corrected_T_map_imu, gtsam::Values& new_values);
  void add_graph_injection_factors(
    const int current,
    const Eigen::Isometry3d& corrected_T_map_imu,
    gtsam::NonlinearFactorGraph& factors,
    bool include_between_factor) const;
  void update_map_odom_for_continuity(const Eigen::Isometry3d& corrected_T_map_imu, const Eigen::Isometry3d& reference_T_map_imu);
  gtsam::NonlinearFactorGraph attempt_relocalization(const int current, gtsam::Values& new_values);
  gtsam::NonlinearFactorGraph create_scan_to_map_factor_or_prior(
    const int current,
    const LocalTargetMap::ConstPtr& target,
    gtsam::Values& new_values);
  void write_trajectory(const glim::EstimationFrame::ConstPtr& frame);
  void open_debug_csv();
  void write_debug_csv(const LocalizationDebugFrameRecord& record);
  void update_debug_record_from_registration(
    const RegistrationResult& result,
    const Eigen::Isometry3d& predicted_T_map_imu,
    const Eigen::Isometry3d& corrected_T_map_imu,
    const LocalTargetMap::ConstPtr& target,
    double registration_ms);
  void finalize_debug_record(
    const int current,
    const LocalizationStatus state_after,
    const gtsam::NonlinearFactorGraph& factors,
    const std::chrono::steady_clock::time_point& started_at);
  bool apply_smoother_hold_guard(const int current, gtsam::Values& new_values, gtsam::NonlinearFactorGraph& factors, const char* reason);
  bool apply_target_rebuild_confirmation_guard(const int current, const LocalTargetMap::ConstPtr& target);
  void fill_relocalization_debug_from_scan_context(const std::string& fallback_reason);
  void verify_rejected_topk_for_debug(const int current);
  std::vector<RelocalizationCandidate> collect_raw_topk_recovery_candidates() const;
  gtsam::NonlinearFactorGraph accept_verified_relocalization(
    const int current,
    const RelocalizationResult& result,
    const Eigen::Isometry3d& reference_T_map_imu,
    double registration_ms,
    gtsam::Values& new_values,
    bool verify_raw_topk_used);
  int required_recovering_stable_frames() const;

private:
  LocalizationOptions options_;
  LocalizationStatus status_;
  std::string status_reason_;
  bool map_loaded_;
  bool initial_pose_ready_;
  bool target_map_ready_;
  bool tracking_started_;
  int consecutive_rejections_;
  int stable_tracking_successes_;
  int recovery_frames_remaining_;
  int relocalization_attempts_;
  int last_relocalization_candidate_count_;
  int last_relocalization_verified_rank_;
  int target_map_rebuild_count_;
  int target_map_recenter_count_;
  int target_map_reuse_count_;
  std::string last_relocalization_message_;
  double last_relocalization_descriptor_distance_;
  double last_continuity_translation_;
  double last_continuity_angle_;
  bool last_continuity_adjusted_;
  bool last_accepted_pose_ready_;
  Eigen::Isometry3d last_accepted_T_map_imu_;
  int lost_frames_without_prior_;
  int consecutive_hold_priors_;
  int recovering_stable_count_;
  int last_relocalization_query_frame_;
  int target_rebuild_guard_rejections_;
  int target_rebuild_guard_confirmation_remaining_;
  int target_rebuild_guard_good_confirmations_;
  bool target_rebuild_guard_pending_valid_;
  Eigen::Isometry3d target_rebuild_guard_pending_pose_;
  double target_rebuild_guard_pending_delta_t_;

  LocalizationMap::Ptr map_;
  LocalTargetMap::Ptr target_map_;
  MapRegistrationBase::Ptr registration_;
  RelocalizerBase::Ptr relocalizer_;
  std::unique_ptr<ConfigInitialPoseProvider> initial_pose_provider_;
  std::unique_ptr<TrajectoryWriter> trajectory_writer_;
  std::unique_ptr<GeometricVerifier> geometric_verifier_;

  // In this localization-only module, GLIM's internal "world" frame is treated
  // as the fixed map frame.  Therefore frame->T_world_imu is T_map_imu and
  // frame->T_world_lidar is T_map_lidar.  T_map_odom is reserved for the later
  // ROS output layer that keeps odom continuous while anchoring it to map.
  Eigen::Isometry3d initial_T_map_imu_;
  Eigen::Isometry3d T_map_odom_;

  RegistrationResult last_registration_result_;
  LocalizationResult last_result_;
  LocalizationDebugFrameRecord debug_record_;
  bool debug_record_active_;
  std::ofstream debug_csv_;
  bool debug_csv_warned_;
  bool current_frame_hold_prior_;
};

}  // namespace glim_localization
