#include <glim_localization/relocalization/scan_context_relocalizer.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <spdlog/spdlog.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace glim_localization {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

}  // namespace

ScanContextRelocalizer::ScanContextRelocalizer(const RelocalizationOptions& options) : options_(options) {}

ScanContextRelocalizer::~ScanContextRelocalizer() {}

bool ScanContextRelocalizer::build(const LocalizationMap::ConstPtr& map) {
  entries_.clear();
  if (!map || map->empty()) {
    spdlog::warn("ScanContext relocalizer build skipped: localization map is empty");
    return false;
  }

  entries_.reserve(map->size());
  for (const auto& submap : map->submaps()) {
    if (!submap || !submap->frame) {
      continue;
    }

    const auto transformed = gtsam_points::transform(submap->frame, submap->T_world_origin);
    if (!transformed || transformed->size() == 0) {
      continue;
    }

    Entry entry;
    entry.submap_id = submap->id;
    entry.submap = submap;
    entry.descriptor = make_descriptor(transformed, submap->T_world_origin.translation());
    entries_.push_back(entry);
  }

  spdlog::info("ScanContext relocalizer indexed {} submaps", entries_.size());
  return !entries_.empty();
}

bool ScanContextRelocalizer::ready() const {
  return !entries_.empty();
}

std::vector<RelocalizationCandidate> ScanContextRelocalizer::query(const glim::EstimationFrame::ConstPtr& frame, int max_candidates) const {
  std::vector<RelocalizationCandidate> candidates;
  last_query_diagnostics_ = QueryDiagnostics();
  last_query_diagnostics_.query_enable = true;
  last_query_diagnostics_.database_size = static_cast<int>(entries_.size());
  last_query_diagnostics_.topk_requested = max_candidates > 0 ? max_candidates : options_.max_candidates;

  if (!ready()) {
    last_query_diagnostics_.failure_reason = "database_empty";
    return candidates;
  }
  if (!frame || !frame->frame || frame->frame->size() == 0) {
    last_query_diagnostics_.failure_reason = "empty_query_cloud";
    return candidates;
  }

  last_query_diagnostics_.query_points = frame->frame->size();
  const Eigen::MatrixXd query_descriptor = make_descriptor(frame->frame, Eigen::Vector3d::Zero());
  last_query_diagnostics_.descriptor_nonempty_bins = count_nonempty_bins(query_descriptor);
  last_query_diagnostics_.descriptor_valid = last_query_diagnostics_.descriptor_nonempty_bins > 0;
  if (!last_query_diagnostics_.descriptor_valid) {
    last_query_diagnostics_.failure_reason = "descriptor_invalid";
    return candidates;
  }
  const int limit = max_candidates > 0 ? max_candidates : options_.max_candidates;

  candidates.reserve(entries_.size());
  for (const auto& entry : entries_) {
    int best_shift = 0;
    const double distance = descriptor_distance(entry.descriptor, query_descriptor, best_shift);
    if (!std::isfinite(distance)) {
      last_query_diagnostics_.filtered_by_other++;
      continue;
    }

    const double yaw = -static_cast<double>(best_shift) * 2.0 * kPi / static_cast<double>(options_.num_sectors);

    RelocalizationCandidate candidate;
    candidate.submap_id = entry.submap_id;
    candidate.submap = entry.submap;
    candidate.descriptor_distance = distance;
    candidate.yaw = yaw;
    candidate.T_map_imu_guess = initial_guess_from_candidate(entry, yaw, *frame);
    candidate.translation_distance = (candidate.T_map_imu_guess.translation() - frame->T_world_imu.translation()).norm();
    candidate.ranking_score = candidate.descriptor_distance + options_.candidate_translation_weight * candidate.translation_distance;

    QueryCandidateDebug debug_candidate;
    debug_candidate.submap_id = candidate.submap_id;
    debug_candidate.descriptor_distance = candidate.descriptor_distance;
    debug_candidate.yaw = candidate.yaw;
    debug_candidate.translation_distance = candidate.translation_distance;
    debug_candidate.passed_descriptor = distance <= options_.max_descriptor_distance;
    debug_candidate.passed_translation =
      !(options_.max_candidate_translation_delta > 0.0 && candidate.translation_distance > options_.max_candidate_translation_delta);
    debug_candidate.candidate = candidate;
    last_query_diagnostics_.topk.push_back(debug_candidate);

    if (!std::isfinite(distance) || distance > options_.max_descriptor_distance) {
      last_query_diagnostics_.filtered_by_descriptor++;
      continue;
    }

    if (options_.max_candidate_translation_delta > 0.0 && candidate.translation_distance > options_.max_candidate_translation_delta) {
      last_query_diagnostics_.filtered_by_translation++;
      continue;
    }
    candidates.push_back(candidate);
  }

  last_query_diagnostics_.candidates_before_filter = static_cast<int>(last_query_diagnostics_.topk.size());

  std::sort(candidates.begin(), candidates.end(), [](const RelocalizationCandidate& lhs, const RelocalizationCandidate& rhs) {
    if (lhs.ranking_score == rhs.ranking_score) {
      if (lhs.descriptor_distance == rhs.descriptor_distance) {
        return lhs.submap_id < rhs.submap_id;
      }
      return lhs.descriptor_distance < rhs.descriptor_distance;
    }
    return lhs.ranking_score < rhs.ranking_score;
  });

  std::sort(last_query_diagnostics_.topk.begin(), last_query_diagnostics_.topk.end(), [](const QueryCandidateDebug& lhs, const QueryCandidateDebug& rhs) {
    if (lhs.descriptor_distance == rhs.descriptor_distance) {
      return lhs.submap_id < rhs.submap_id;
    }
    return lhs.descriptor_distance < rhs.descriptor_distance;
  });

  last_query_diagnostics_.topk_returned = static_cast<int>(last_query_diagnostics_.topk.size());

  if (limit > 0 && static_cast<int>(candidates.size()) > limit) {
    candidates.resize(limit);
  }
  last_query_diagnostics_.candidates_after_filter = static_cast<int>(candidates.size());
  if (candidates.empty()) {
    if (last_query_diagnostics_.topk.empty()) {
      last_query_diagnostics_.failure_reason = "no_topk";
    } else if (last_query_diagnostics_.filtered_by_descriptor >= last_query_diagnostics_.candidates_before_filter) {
      last_query_diagnostics_.failure_reason = "all_filtered_by_descriptor_distance";
    } else if (last_query_diagnostics_.filtered_by_translation > 0) {
      last_query_diagnostics_.failure_reason = "all_filtered_by_translation";
    } else {
      last_query_diagnostics_.failure_reason = "unknown";
    }
  } else {
    last_query_diagnostics_.failure_reason = "success";
  }

  return candidates;
}

ScanContextRelocalizer::QueryDiagnostics ScanContextRelocalizer::last_query_diagnostics() const {
  return last_query_diagnostics_;
}

Eigen::MatrixXd ScanContextRelocalizer::make_descriptor(const gtsam_points::PointCloud::ConstPtr& cloud, const Eigen::Vector3d& center) const {
  const int rings = std::max(1, options_.num_rings);
  const int sectors = std::max(1, options_.num_sectors);
  const double min_radius = std::max(0.0, options_.min_radius);
  const double max_radius = std::max(min_radius + 1e-3, options_.max_radius);
  const double ring_step = (max_radius - min_radius) / static_cast<double>(rings);
  const double sector_step = 2.0 * kPi / static_cast<double>(sectors);

  Eigen::MatrixXd descriptor = Eigen::MatrixXd::Constant(rings, sectors, -std::numeric_limits<double>::infinity());
  if (!cloud) {
    descriptor.setZero();
    return descriptor;
  }

  for (int i = 0; i < cloud->size(); i++) {
    const Eigen::Vector3d p = cloud->points[i].head<3>() - center;
    const double radius = std::hypot(p.x(), p.y());
    if (radius < min_radius || radius >= max_radius) {
      continue;
    }

    double angle = std::atan2(p.y(), p.x());
    if (angle < 0.0) {
      angle += 2.0 * kPi;
    }

    const int ring = static_cast<int>((radius - min_radius) / ring_step);
    const int sector = static_cast<int>(angle / sector_step);
    if (ring < 0 || ring >= rings || sector < 0 || sector >= sectors) {
      continue;
    }

    descriptor(ring, sector) = std::max(descriptor(ring, sector), p.z());
  }

  for (int r = 0; r < descriptor.rows(); r++) {
    for (int c = 0; c < descriptor.cols(); c++) {
      if (!std::isfinite(descriptor(r, c))) {
        descriptor(r, c) = 0.0;
      }
    }
  }

  return descriptor;
}

int ScanContextRelocalizer::count_nonempty_bins(const Eigen::MatrixXd& descriptor) const {
  int count = 0;
  for (int r = 0; r < descriptor.rows(); r++) {
    for (int c = 0; c < descriptor.cols(); c++) {
      if (std::isfinite(descriptor(r, c)) && std::abs(descriptor(r, c)) > 1e-9) {
        count++;
      }
    }
  }
  return count;
}

double ScanContextRelocalizer::descriptor_distance(const Eigen::MatrixXd& target, const Eigen::MatrixXd& query, int& best_shift) const {
  best_shift = 0;
  if (target.rows() != query.rows() || target.cols() != query.cols() || target.cols() == 0) {
    return std::numeric_limits<double>::infinity();
  }

  double best_distance = std::numeric_limits<double>::infinity();
  for (int shift = 0; shift < target.cols(); shift++) {
    double distance_sum = 0.0;
    int valid_columns = 0;

    for (int c = 0; c < target.cols(); c++) {
      const int shifted_col = (c + shift) % target.cols();
      const Eigen::VectorXd target_col = target.col(c);
      const Eigen::VectorXd query_col = query.col(shifted_col);
      const double target_norm = target_col.norm();
      const double query_norm = query_col.norm();
      if (target_norm < 1e-6 || query_norm < 1e-6) {
        continue;
      }

      const double cos_sim = clamp(target_col.dot(query_col) / (target_norm * query_norm), -1.0, 1.0);
      distance_sum += 1.0 - cos_sim;
      valid_columns++;
    }

    if (valid_columns == 0) {
      continue;
    }

    const double distance = distance_sum / static_cast<double>(valid_columns);
    if (distance < best_distance) {
      best_distance = distance;
      best_shift = shift;
    }
  }

  return best_distance;
}

Eigen::Isometry3d ScanContextRelocalizer::initial_guess_from_candidate(const Entry& entry, double yaw, const glim::EstimationFrame& frame) const {
  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  if (entry.submap) {
    T_map_lidar = entry.submap->T_world_origin;
  }

  T_map_lidar.linear() = T_map_lidar.linear() * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return T_map_lidar * frame.T_lidar_imu;
}

}  // namespace glim_localization
