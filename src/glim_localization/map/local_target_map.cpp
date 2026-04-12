#include <glim_localization/map/local_target_map.hpp>

#include <algorithm>
#include <cmath>

namespace glim_localization {

LocalTargetMap::LocalTargetMap() : reused_last_time_(false), reuse_count_(0) {
  T_map_target_center_.setIdentity();
}

LocalTargetMap::LocalTargetMap(const Eigen::Isometry3d& T_map_target_center, const std::vector<SubMapConstPtr>& submaps)
: T_map_target_center_(T_map_target_center),
  submaps_(submaps),
  reused_last_time_(false),
  reuse_count_(0) {
  refresh_active_submap_ids();
}

bool LocalTargetMap::empty() const {
  return submaps_.empty();
}

std::size_t LocalTargetMap::size() const {
  return submaps_.size();
}

const std::vector<LocalTargetMap::SubMapConstPtr>& LocalTargetMap::submaps() const {
  return submaps_;
}

const std::vector<int>& LocalTargetMap::active_submap_ids() const {
  return active_submap_ids_;
}

std::vector<int> LocalTargetMap::submap_ids() const {
  return active_submap_ids_;
}

std::vector<gtsam_points::PointCloud::ConstPtr> LocalTargetMap::target_frames() const {
  std::vector<gtsam_points::PointCloud::ConstPtr> frames;
  frames.reserve(submaps_.size());
  for (const auto& submap : submaps_) {
    if (submap && submap->frame) {
      frames.push_back(submap->frame);
    }
  }

  return frames;
}

gtsam_points::PointCloudCPU::ConstPtr LocalTargetMap::merged_target_cloud() const {
  if (merged_target_cloud_) {
    return merged_target_cloud_;
  }

  std::vector<Eigen::Vector4d> points;
  std::vector<Eigen::Vector4d> normals;
  std::vector<Eigen::Matrix4d> covs;

  bool all_have_normals = true;
  bool all_have_covs = true;

  for (const auto& submap : submaps_) {
    if (!submap || !submap->frame) {
      continue;
    }

    const auto transformed = gtsam_points::transform(submap->frame, submap->T_world_origin);
    if (!transformed) {
      continue;
    }

    all_have_normals = all_have_normals && transformed->has_normals();
    all_have_covs = all_have_covs && transformed->has_covs();

    for (int i = 0; i < transformed->size(); i++) {
      points.push_back(transformed->points[i]);
      if (transformed->has_normals()) {
        normals.push_back(transformed->normals[i]);
      }
      if (transformed->has_covs()) {
        covs.push_back(transformed->covs[i]);
      }
    }
  }

  if (points.empty()) {
    return nullptr;
  }

  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  if (all_have_normals && normals.size() == points.size()) {
    cloud->add_normals(normals);
  }
  if (all_have_covs && covs.size() == points.size()) {
    cloud->add_covs(covs);
  }

  merged_target_cloud_ = cloud;
  return merged_target_cloud_;
}

const Eigen::Isometry3d& LocalTargetMap::T_map_target_center() const {
  return T_map_target_center_;
}

Eigen::Vector3d LocalTargetMap::target_center() const {
  return T_map_target_center_.translation();
}

const Eigen::Isometry3d& LocalTargetMap::T_map_target() const {
  return T_map_target_center_;
}

void LocalTargetMap::set_T_map_target(const Eigen::Isometry3d& T_map_target) {
  T_map_target_center_ = T_map_target;
}

double LocalTargetMap::distance_from_center(const Eigen::Isometry3d& T_map_sensor) const {
  return (T_map_sensor.translation() - T_map_target_center_.translation()).norm();
}

double LocalTargetMap::angle_from_center(const Eigen::Isometry3d& T_map_sensor) const {
  const Eigen::Matrix3d delta_R = T_map_target_center_.linear().transpose() * T_map_sensor.linear();
  const double cos_angle = std::max(-1.0, std::min(1.0, (delta_R.trace() - 1.0) * 0.5));
  return std::acos(cos_angle);
}

bool LocalTargetMap::needs_update(const Eigen::Isometry3d& T_map_sensor, double update_distance, double update_angle) const {
  if (empty()) {
    return true;
  }

  const bool distance_exceeded = update_distance > 0.0 && distance_from_center(T_map_sensor) >= update_distance;
  const bool angle_exceeded = update_angle > 0.0 && angle_from_center(T_map_sensor) >= update_angle;
  return distance_exceeded || angle_exceeded;
}

bool LocalTargetMap::reused_last_time() const {
  return reused_last_time_;
}

int LocalTargetMap::reuse_count() const {
  return reuse_count_;
}

void LocalTargetMap::mark_reused() {
  reused_last_time_ = true;
  reuse_count_++;
}

void LocalTargetMap::mark_rebuilt() {
  reused_last_time_ = false;
  reuse_count_ = 0;
}

void LocalTargetMap::refresh_active_submap_ids() {
  active_submap_ids_.clear();
  active_submap_ids_.reserve(submaps_.size());

  for (const auto& submap : submaps_) {
    if (submap) {
      active_submap_ids_.push_back(submap->id);
    }
  }
}

}  // namespace glim_localization
