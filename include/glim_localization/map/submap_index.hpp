#pragma once

#include <cstddef>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/mapping/sub_map.hpp>

namespace glim_localization {

class SubmapIndex {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<SubmapIndex>;
  using ConstPtr = std::shared_ptr<const SubmapIndex>;
  using SubMapConstPtr = glim::SubMap::ConstPtr;

  struct Stats {
    std::size_t num_submaps = 0;
    std::size_t num_cells = 0;
    std::size_t max_cell_size = 0;
    double resolution = 20.0;
  };

  explicit SubmapIndex(double resolution = 20.0);

  void clear();
  void build(const std::vector<SubMapConstPtr>& submaps);
  bool empty() const;
  Stats stats() const;

  std::vector<SubMapConstPtr> query_nearby(const Eigen::Isometry3d& T_map_sensor, int max_num_submaps, double max_distance) const;

private:
  struct CellKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const CellKey& rhs) const { return x == rhs.x && y == rhs.y && z == rhs.z; }
  };

  struct CellKeyHash {
    std::size_t operator()(const CellKey& key) const;
  };

  struct Candidate {
    double distance = 0.0;
    SubMapConstPtr submap;
  };

  CellKey point_to_cell(const Eigen::Vector3d& point) const;
  std::vector<Candidate> collect_candidates(const Eigen::Vector3d& center, double max_distance) const;

private:
  double resolution_;
  std::vector<SubMapConstPtr> submaps_;
  std::unordered_map<CellKey, std::vector<SubMapConstPtr>, CellKeyHash> cells_;
};

}  // namespace glim_localization
