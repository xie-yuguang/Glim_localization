#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>

#include <glim_localization/map/submap_index.hpp>

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
  std::vector<glim_localization::SubmapIndex::SubMapConstPtr> submaps;
  submaps.push_back(make_submap(1, 0.0));
  submaps.push_back(make_submap(2, 5.0));
  submaps.push_back(make_submap(3, 20.0));
  submaps.push_back(make_submap(4, -6.0));

  glim_localization::SubmapIndex index(10.0);
  index.build(submaps);
  expect(!index.empty(), "index must not be empty");

  const auto stats = index.stats();
  expect(stats.num_submaps == submaps.size(), "index stats must preserve submap count");
  expect(stats.num_cells >= 2, "index must create multiple cells for spread submaps");

  const auto nearby = index.query_nearby(make_pose(1.0), 2, 8.0);
  expect(nearby.size() == 2, "nearby query must return two closest submaps");
  expect(nearby[0]->id == 1, "nearest submap id mismatch");
  expect(nearby[1]->id == 2, "second nearest submap id mismatch");

  const auto all = index.query_nearby(make_pose(1.0), 0, 0.0);
  expect(all.size() == submaps.size(), "max_distance <= 0 must query all indexed submaps");

  std::cout << "test_submap_index passed" << std::endl;
  return 0;
}
