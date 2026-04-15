#include <cstdlib>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/odometry/estimation_frame.hpp>
#include <glim_localization/map/localization_map.hpp>
#include <glim_localization/relocalization/scan_context_relocalizer.hpp>

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    std::exit(1);
  }
}

std::vector<Eigen::Vector4d> make_pattern(int pattern_id) {
  std::vector<Eigen::Vector4d> points;
  for (int sector = 0; sector < 36; sector++) {
    const double angle = sector * 2.0 * 3.14159265358979323846 / 36.0;
    for (int ring = 1; ring <= 8; ring++) {
      const double radius = 1.0 + ring;
      double z = 0.1 * ring;
      if (pattern_id == 2) {
        z += ((sector + 2) % 5) == 0 ? 2.0 : 0.0;
        z += (ring % 3) == 0 ? 0.4 : 0.0;
      } else {
        z += (sector % 7) == 0 ? 0.6 : 0.0;
        z += (ring % 4) == 0 ? 0.1 : 0.0;
      }
      points.emplace_back(radius * std::cos(angle), radius * std::sin(angle), z, 1.0);
    }
  }
  return points;
}

glim::SubMap::Ptr make_submap(int id, const Eigen::Vector3d& translation, int pattern_id) {
  auto submap = std::make_shared<glim::SubMap>();
  submap->id = id;
  submap->T_world_origin.setIdentity();
  submap->T_world_origin.translation() = translation;
  submap->frame = std::make_shared<gtsam_points::PointCloudCPU>(make_pattern(pattern_id));
  return submap;
}

}  // namespace

int main() {
  std::vector<glim::SubMap::Ptr> submaps;
  submaps.push_back(make_submap(1, Eigen::Vector3d(0.0, 0.0, 0.0), 0));
  submaps.push_back(make_submap(2, Eigen::Vector3d(20.0, 0.0, 0.0), 2));

  auto map = std::make_shared<glim_localization::LocalizationMap>(submaps);

  glim_localization::RelocalizationOptions options;
  options.num_rings = 20;
  options.num_sectors = 60;
  options.max_radius = 20.0;
  options.max_descriptor_distance = 0.5;
  options.max_candidates = 2;

  glim_localization::ScanContextRelocalizer relocalizer(options);
  expect(relocalizer.build(map), "relocalizer must build from synthetic map");
  expect(relocalizer.ready(), "relocalizer must be ready after build");

  auto frame = std::make_shared<glim::EstimationFrame>();
  frame->T_lidar_imu.setIdentity();
  frame->T_world_imu.setIdentity();
  frame->T_world_imu.translation() = Eigen::Vector3d(19.0, 0.0, 0.0);
  frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(make_pattern(2));

  const auto candidates = relocalizer.query(frame, 2);
  expect(!candidates.empty(), "query must return candidates");
  if (candidates.front().submap_id != 2) {
    std::cerr << "candidate ranking:";
    for (const auto& candidate : candidates) {
      std::cerr << " [id=" << candidate.submap_id << " dist=" << candidate.descriptor_distance << "]";
    }
    std::cerr << std::endl;
  }
  expect(candidates.front().submap_id == 2, "best candidate must match the similar submap");
  expect(static_cast<bool>(candidates.front().submap), "candidate must carry submap pointer");
  expect(candidates.front().ranking_score >= candidates.front().descriptor_distance, "ranking score must include descriptor term");
  expect(candidates.front().translation_distance >= 0.0, "translation distance must be non-negative");

  std::cout << "test_scan_context_relocalizer passed: best_submap=" << candidates.front().submap_id
            << " descriptor_distance=" << candidates.front().descriptor_distance << std::endl;
  return 0;
}
