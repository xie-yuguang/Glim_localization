#include <glim_localization/map/localization_map.hpp>

#include <algorithm>
#include <limits>

namespace glim_localization {
namespace {

struct SubmapDistance {
  double distance = 0.0;
  LocalizationMap::SubMapConstPtr submap;
};

}  // namespace

LocalizationMap::LocalizationMap() {}

LocalizationMap::LocalizationMap(const std::vector<glim::SubMap::Ptr>& submaps) {
  set_submaps(submaps);
}

LocalizationMap::LocalizationMap(const std::vector<SubMapConstPtr>& submaps) : submaps_(submaps) {}

bool LocalizationMap::empty() const {
  return submaps_.empty();
}

std::size_t LocalizationMap::size() const {
  return submaps_.size();
}

void LocalizationMap::clear() {
  submaps_.clear();
  metadata_ = LocalizationMapMetadata();
  clear_index();
}

void LocalizationMap::set_submaps(const std::vector<glim::SubMap::Ptr>& submaps) {
  clear_index();
  submaps_.clear();
  submaps_.reserve(submaps.size());
  for (const auto& submap : submaps) {
    submaps_.push_back(submap);
  }
}

void LocalizationMap::set_submaps(const std::vector<SubMapConstPtr>& submaps) {
  clear_index();
  submaps_ = submaps;
}

void LocalizationMap::set_metadata(const LocalizationMapMetadata& metadata) {
  metadata_ = metadata;
}

const std::vector<LocalizationMap::SubMapConstPtr>& LocalizationMap::submaps() const {
  return submaps_;
}

LocalizationMap::SubMapConstPtr LocalizationMap::at(std::size_t i) const {
  return submaps_.at(i);
}

std::vector<int> LocalizationMap::submap_ids() const {
  std::vector<int> ids;
  ids.reserve(submaps_.size());
  for (const auto& submap : submaps_) {
    if (submap) {
      ids.push_back(submap->id);
    }
  }

  return ids;
}

const LocalizationMapMetadata& LocalizationMap::metadata() const {
  return metadata_;
}

LocalizationMapStats LocalizationMap::stats() const {
  LocalizationMapStats result;
  result.num_submaps = submaps_.size();

  for (const auto& submap : submaps_) {
    if (!submap) {
      continue;
    }

    const Eigen::Vector3d origin = submap->T_world_origin.translation();
    if (!result.has_bounds) {
      result.origin_min = origin;
      result.origin_max = origin;
      result.has_bounds = true;
    } else {
      result.origin_min = result.origin_min.cwiseMin(origin);
      result.origin_max = result.origin_max.cwiseMax(origin);
    }

    if (submap->frame) {
      result.num_points += static_cast<std::size_t>(submap->frame->size());
    }
  }

  return result;
}

void LocalizationMap::build_index(double resolution) {
  index_ = std::make_shared<SubmapIndex>(resolution);
  index_->build(submaps_);
}

void LocalizationMap::clear_index() {
  if (index_) {
    index_->clear();
  }
  index_.reset();
}

bool LocalizationMap::has_index() const {
  return index_ && !index_->empty();
}

SubmapIndex::Stats LocalizationMap::index_stats() const {
  return index_ ? index_->stats() : SubmapIndex::Stats();
}

std::vector<LocalizationMap::SubMapConstPtr> LocalizationMap::query_nearby(const Eigen::Isometry3d& T_map_sensor, int max_num_submaps, double max_distance) const {
  if (has_index()) {
    return index_->query_nearby(T_map_sensor, max_num_submaps, max_distance);
  }

  std::vector<SubmapDistance> candidates;
  candidates.reserve(submaps_.size());

  for (const auto& submap : submaps_) {
    if (!submap) {
      continue;
    }

    const double distance = (submap->T_world_origin.translation() - T_map_sensor.translation()).norm();
    if (max_distance <= 0.0 || distance <= max_distance) {
      candidates.push_back({distance, submap});
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const SubmapDistance& lhs, const SubmapDistance& rhs) {
    if (lhs.distance == rhs.distance) {
      const int lhs_id = lhs.submap ? lhs.submap->id : std::numeric_limits<int>::max();
      const int rhs_id = rhs.submap ? rhs.submap->id : std::numeric_limits<int>::max();
      return lhs_id < rhs_id;
    }

    return lhs.distance < rhs.distance;
  });

  if (max_num_submaps > 0 && static_cast<int>(candidates.size()) > max_num_submaps) {
    candidates.resize(max_num_submaps);
  }

  std::vector<SubMapConstPtr> result;
  result.reserve(candidates.size());
  for (const auto& candidate : candidates) {
    result.push_back(candidate.submap);
  }

  return result;
}

LocalTargetMap::Ptr LocalizationMap::build_target(const std::vector<SubMapConstPtr>& submaps, const Eigen::Isometry3d& T_map_target_center) const {
  return std::make_shared<LocalTargetMap>(T_map_target_center, submaps);
}

LocalTargetMap::Ptr LocalizationMap::query_target(const Eigen::Isometry3d& T_map_sensor, int max_num_submaps, double max_distance) const {
  return build_target(query_nearby(T_map_sensor, max_num_submaps, max_distance), T_map_sensor);
}

}  // namespace glim_localization
