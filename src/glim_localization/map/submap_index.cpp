#include <glim_localization/map/submap_index.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace glim_localization {
namespace {

double normalized_resolution(double resolution) {
  return resolution > 1e-6 ? resolution : 20.0;
}

}  // namespace

SubmapIndex::SubmapIndex(double resolution) : resolution_(normalized_resolution(resolution)) {}

void SubmapIndex::clear() {
  submaps_.clear();
  cells_.clear();
}

void SubmapIndex::build(const std::vector<SubMapConstPtr>& submaps) {
  clear();
  submaps_ = submaps;

  for (const auto& submap : submaps_) {
    if (!submap) {
      continue;
    }

    cells_[point_to_cell(submap->T_world_origin.translation())].push_back(submap);
  }
}

bool SubmapIndex::empty() const {
  return cells_.empty();
}

SubmapIndex::Stats SubmapIndex::stats() const {
  Stats stats;
  stats.num_submaps = submaps_.size();
  stats.num_cells = cells_.size();
  stats.resolution = resolution_;

  for (const auto& cell : cells_) {
    stats.max_cell_size = std::max(stats.max_cell_size, cell.second.size());
  }

  return stats;
}

std::vector<SubmapIndex::SubMapConstPtr> SubmapIndex::query_nearby(const Eigen::Isometry3d& T_map_sensor, int max_num_submaps, double max_distance) const {
  const Eigen::Vector3d center = T_map_sensor.translation();
  const auto candidates = collect_candidates(center, max_distance);

  std::vector<Candidate> sorted = candidates;
  std::sort(sorted.begin(), sorted.end(), [](const Candidate& lhs, const Candidate& rhs) {
    if (lhs.distance == rhs.distance) {
      const int lhs_id = lhs.submap ? lhs.submap->id : std::numeric_limits<int>::max();
      const int rhs_id = rhs.submap ? rhs.submap->id : std::numeric_limits<int>::max();
      return lhs_id < rhs_id;
    }
    return lhs.distance < rhs.distance;
  });

  if (max_num_submaps > 0 && static_cast<int>(sorted.size()) > max_num_submaps) {
    sorted.resize(max_num_submaps);
  }

  std::vector<SubMapConstPtr> result;
  result.reserve(sorted.size());
  for (const auto& candidate : sorted) {
    result.push_back(candidate.submap);
  }

  return result;
}

std::size_t SubmapIndex::CellKeyHash::operator()(const CellKey& key) const {
  const std::size_t hx = std::hash<int>()(key.x);
  const std::size_t hy = std::hash<int>()(key.y);
  const std::size_t hz = std::hash<int>()(key.z);
  return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6) + (hx >> 2)) ^ (hz + 0x9e3779b97f4a7c15ULL + (hy << 6) + (hy >> 2));
}

SubmapIndex::CellKey SubmapIndex::point_to_cell(const Eigen::Vector3d& point) const {
  return CellKey{
    static_cast<int>(std::floor(point.x() / resolution_)),
    static_cast<int>(std::floor(point.y() / resolution_)),
    static_cast<int>(std::floor(point.z() / resolution_))};
}

std::vector<SubmapIndex::Candidate> SubmapIndex::collect_candidates(const Eigen::Vector3d& center, double max_distance) const {
  std::vector<Candidate> candidates;
  if (cells_.empty()) {
    return candidates;
  }

  if (max_distance <= 0.0) {
    candidates.reserve(submaps_.size());
    for (const auto& submap : submaps_) {
      if (!submap) {
        continue;
      }
      candidates.push_back({(submap->T_world_origin.translation() - center).norm(), submap});
    }
    return candidates;
  }

  const CellKey center_cell = point_to_cell(center);
  const int radius = std::max(0, static_cast<int>(std::ceil(max_distance / resolution_)));

  for (int dx = -radius; dx <= radius; dx++) {
    for (int dy = -radius; dy <= radius; dy++) {
      for (int dz = -radius; dz <= radius; dz++) {
        const CellKey key{center_cell.x + dx, center_cell.y + dy, center_cell.z + dz};
        const auto found = cells_.find(key);
        if (found == cells_.end()) {
          continue;
        }

        for (const auto& submap : found->second) {
          if (!submap) {
            continue;
          }

          const double distance = (submap->T_world_origin.translation() - center).norm();
          if (distance <= max_distance) {
            candidates.push_back({distance, submap});
          }
        }
      }
    }
  }

  return candidates;
}

}  // namespace glim_localization
