#include <cstdlib>
#include <iostream>
#include <vector>

#include <glim_localization/map/localization_map.hpp>

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    std::exit(1);
  }
}

glim::SubMap::Ptr make_submap(int id, double x, double y = 0.0, double z = 0.0) {
  auto submap = std::make_shared<glim::SubMap>();
  submap->id = id;
  submap->T_world_origin.setIdentity();
  submap->T_world_origin.translation() = Eigen::Vector3d(x, y, z);
  return submap;
}

Eigen::Isometry3d make_pose(double x, double y = 0.0, double z = 0.0) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

}  // namespace

int main() {
  std::vector<glim::SubMap::Ptr> submaps;
  submaps.push_back(make_submap(10, 0.0));
  submaps.push_back(make_submap(20, 10.0));
  submaps.push_back(make_submap(30, 20.0));
  submaps.push_back(make_submap(40, -5.0));

  glim_localization::LocalizationMap map(submaps);
  expect(!map.empty(), "map must not be empty");
  expect(map.size() == 4, "map size must match input submap count");

  const auto ids = map.submap_ids();
  expect((ids == std::vector<int>{10, 20, 30, 40}), "global map ids must preserve inserted order");

  const auto nearby_all = map.query_nearby(make_pose(1.0), 0, 100.0);
  expect(nearby_all.size() == 4, "query with max_num_submaps=0 must return all candidates within distance");
  expect(nearby_all[0]->id == 10, "nearest submap to x=1 must be id=10");
  expect(nearby_all[1]->id == 40, "second nearest submap to x=1 must be id=40");
  expect(nearby_all[2]->id == 20, "third nearest submap to x=1 must be id=20");
  expect(nearby_all[3]->id == 30, "fourth nearest submap to x=1 must be id=30");

  const auto nearby_limited_distance = map.query_nearby(make_pose(1.0), 0, 6.1);
  expect(nearby_limited_distance.size() == 2, "max_distance must filter far submaps");
  expect(nearby_limited_distance[0]->id == 10, "distance-filtered first id mismatch");
  expect(nearby_limited_distance[1]->id == 40, "distance-filtered second id mismatch");

  const auto nearby_limited_count = map.query_nearby(make_pose(1.0), 2, 100.0);
  expect(nearby_limited_count.size() == 2, "max_num_submaps must limit output count");
  expect(nearby_limited_count[0]->id == 10, "count-limited first id mismatch");
  expect(nearby_limited_count[1]->id == 40, "count-limited second id mismatch");

  const auto target = map.build_target(nearby_limited_count, make_pose(1.0));
  expect(static_cast<bool>(target), "target map must be created");
  expect(target->size() == 2, "target map size mismatch");
  expect((target->active_submap_ids() == std::vector<int>{10, 40}), "active submap ids mismatch");
  expect((target->submap_ids() == std::vector<int>{10, 40}), "submap_ids alias mismatch");
  expect((target->target_center() - Eigen::Vector3d(1.0, 0.0, 0.0)).norm() < 1e-9, "target center mismatch");
  expect(target->target_frames().empty(), "manually constructed test submaps have no target frames");
  expect(!target->needs_update(make_pose(1.5), 2.0, 0.2), "target map must be reusable inside update thresholds");
  expect(target->needs_update(make_pose(3.1), 2.0, 0.2), "target map must update after translation threshold");

  Eigen::Isometry3d rotated_pose = make_pose(1.0);
  rotated_pose.linear() = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  expect(target->needs_update(rotated_pose, 2.0, 0.2), "target map must update after rotation threshold");

  expect(!target->reused_last_time(), "newly built target map must not be marked as reused");
  target->mark_reused();
  expect(target->reused_last_time(), "target map reuse marker must be set");
  expect(target->reuse_count() == 1, "target map reuse count must increase");
  target->mark_rebuilt();
  expect(!target->reused_last_time(), "rebuilt target map must clear reuse marker");
  expect(target->reuse_count() == 0, "rebuilt target map must reset reuse count");

  const auto queried_target = map.query_target(make_pose(19.0), 1, 100.0);
  expect(queried_target->size() == 1, "query_target must build a one-submap target");
  expect(queried_target->active_submap_ids()[0] == 30, "query_target nearest id mismatch");

  const auto no_candidates = map.query_nearby(make_pose(1000.0), 2, 1.0);
  expect(no_candidates.empty(), "query outside max_distance must return no candidates");

  std::cout << "test_localization_map passed" << std::endl;
  return 0;
}
