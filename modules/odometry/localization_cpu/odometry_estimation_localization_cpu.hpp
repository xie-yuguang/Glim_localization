#pragma once

#include <memory>

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
  LocalTargetMap::Ptr build_or_update_target_map(const Eigen::Isometry3d& T_map_imu);
  bool should_update_target_map(const Eigen::Isometry3d& T_map_imu) const;
  gtsam::NonlinearFactorGraph attempt_relocalization(const int current, gtsam::Values& new_values);
  gtsam::NonlinearFactorGraph create_scan_to_map_factor_or_prior(
    const int current,
    const LocalTargetMap::ConstPtr& target,
    gtsam::Values& new_values);
  void write_trajectory(const glim::EstimationFrame::ConstPtr& frame);

private:
  LocalizationOptions options_;
  LocalizationStatus status_;
  bool map_loaded_;
  bool initial_pose_ready_;
  bool target_map_ready_;
  int consecutive_rejections_;

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
};

}  // namespace glim_localization
