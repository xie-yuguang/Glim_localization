#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/mapping/sub_map.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace glim_localization {

class LocalTargetMap {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LocalTargetMap>;
  using ConstPtr = std::shared_ptr<const LocalTargetMap>;
  using SubMapConstPtr = glim::SubMap::ConstPtr;

  LocalTargetMap();
  LocalTargetMap(const Eigen::Isometry3d& T_map_target_center, const std::vector<SubMapConstPtr>& submaps);

  bool empty() const;
  std::size_t size() const;

  const std::vector<SubMapConstPtr>& submaps() const;
  const std::vector<int>& active_submap_ids() const;
  std::vector<int> submap_ids() const;
  std::vector<gtsam_points::PointCloud::ConstPtr> target_frames() const;
  gtsam_points::PointCloudCPU::ConstPtr merged_target_cloud() const;

  const Eigen::Isometry3d& T_map_target_center() const;
  Eigen::Vector3d target_center() const;

  const Eigen::Isometry3d& T_map_target() const;
  void set_T_map_target(const Eigen::Isometry3d& T_map_target);

  double distance_from_center(const Eigen::Isometry3d& T_map_sensor) const;
  double angle_from_center(const Eigen::Isometry3d& T_map_sensor) const;
  bool needs_update(const Eigen::Isometry3d& T_map_sensor, double update_distance, double update_angle) const;

  bool reused_last_time() const;
  int reuse_count() const;
  void mark_reused();
  void mark_rebuilt();

private:
  void refresh_active_submap_ids();

private:
  Eigen::Isometry3d T_map_target_center_;
  std::vector<SubMapConstPtr> submaps_;
  std::vector<int> active_submap_ids_;
  mutable gtsam_points::PointCloudCPU::Ptr merged_target_cloud_;
  bool reused_last_time_;
  int reuse_count_;
};

}  // namespace glim_localization
