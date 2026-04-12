#include <cstdlib>
#include <iostream>
#include <vector>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/odometry/estimation_frame.hpp>
#include <glim_localization/map/local_target_map.hpp>
#include <glim_localization/registration/cpu_gicp_map_registration.hpp>

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    std::exit(1);
  }
}

Eigen::Matrix4d test_covariance() {
  Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
  cov.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.01;
  return cov;
}

gtsam_points::PointCloudCPU::Ptr make_cloud(const std::vector<Eigen::Vector4d>& points) {
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);

  std::vector<Eigen::Matrix4d> covs(points.size(), test_covariance());
  cloud->add_covs(covs);

  return cloud;
}

std::vector<Eigen::Vector4d> make_target_points() {
  std::vector<Eigen::Vector4d> points;
  for (int x = -3; x <= 3; x++) {
    for (int y = -3; y <= 3; y++) {
      for (int z = 0; z <= 2; z++) {
        points.emplace_back(0.4 * x, 0.35 * y, 0.5 * z, 1.0);
      }
    }
  }
  return points;
}

Eigen::Isometry3d make_pose(double x, double y, double z, double yaw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

double pose_error(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  const Eigen::Isometry3d delta = a.inverse() * b;
  const double trans_error = delta.translation().norm();
  const double rot_error = Eigen::AngleAxisd(delta.linear()).angle();
  return trans_error + rot_error;
}

}  // namespace

int main() {
  const auto target_points = make_target_points();
  const Eigen::Isometry3d true_T_map_imu = make_pose(0.35, -0.15, 0.08, 0.03);
  const Eigen::Isometry3d initial_T_map_imu = make_pose(0.45, -0.22, 0.11, 0.06);

  std::vector<Eigen::Vector4d> source_points;
  source_points.reserve(target_points.size());
  for (const auto& point_map : target_points) {
    source_points.push_back(true_T_map_imu.inverse() * point_map);
  }

  auto submap = std::make_shared<glim::SubMap>();
  submap->id = 7;
  submap->T_world_origin.setIdentity();
  submap->frame = make_cloud(target_points);

  auto frame = std::make_shared<glim::EstimationFrame>();
  frame->id = 0;
  frame->stamp = 1.0;
  frame->T_lidar_imu.setIdentity();
  frame->T_world_imu = initial_T_map_imu;
  frame->T_world_lidar = initial_T_map_imu;
  frame->frame_id = glim::FrameID::IMU;
  frame->frame = make_cloud(source_points);

  glim_localization::LocalTargetMap::SubMapConstPtr const_submap = submap;
  auto target = std::make_shared<glim_localization::LocalTargetMap>(Eigen::Isometry3d::Identity(), std::vector<glim_localization::LocalTargetMap::SubMapConstPtr>{const_submap});

  glim_localization::MatchingOptions options;
  options.max_iterations = 30;
  options.max_correspondence_distance = 1.5;
  options.min_score = 0.0;
  options.num_threads = 1;

  glim_localization::CpuGicpMapRegistration registration(options);
  const auto result = registration.align(frame, target, initial_T_map_imu);

  expect(result.converged, "registration must converge on synthetic translated scan");
  expect(result.accepted, "registration must be accepted with min_score=0.0");
  expect(result.num_source_points == static_cast<int>(source_points.size()), "source point count mismatch");
  expect(result.num_target_points == static_cast<int>(target_points.size()), "target point count mismatch");
  expect(result.score >= 0.0, "score must be populated");
  expect(result.residual >= 0.0, "residual must be populated");

  const double initial_error = pose_error(true_T_map_imu, initial_T_map_imu);
  const double corrected_error = pose_error(true_T_map_imu, result.T_map_imu);
  expect(corrected_error < initial_error, "corrected pose must be closer to true pose than initial pose");

  glim_localization::MatchingOptions strict_options = options;
  strict_options.min_inliers = static_cast<int>(source_points.size()) + 1;
  glim_localization::CpuGicpMapRegistration strict_registration(strict_options);
  const auto rejected = strict_registration.align(frame, target, initial_T_map_imu);
  expect(!rejected.accepted, "registration must be rejected when min_inliers is impossible");
  expect(rejected.reject_reason == "few_inliers", "registration reject reason must identify inlier gating");

  std::cout << "test_cpu_map_registration passed: initial_error=" << initial_error << " corrected_error=" << corrected_error << " score=" << result.score
            << " residual=" << result.residual << std::endl;
  return 0;
}
