#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/mapping/sub_map.hpp>
#include <glim_localization/map/local_target_map.hpp>
#include <glim_localization/map_loader/map_load_options.hpp>
#include <glim_localization/map_loader/map_format_checker.hpp>
#include <glim_localization/map/submap_index.hpp>

namespace glim_localization {

struct LocalizationMapMetadata {
  std::string map_path;
  std::string detected_format = "unknown";
  std::string compatibility = "unsupported";
  bool strict = true;
  bool load_voxelmaps = true;
  bool load_raw_frames = false;
  int requested_submaps = 0;
  int loaded_submaps = 0;
  int skipped_submaps = 0;
  bool has_graph_txt = false;
  bool has_graph_bin = false;
  bool has_values_bin = false;
};

struct LocalizationMapStats {
  std::size_t num_submaps = 0;
  std::size_t num_points = 0;
  bool has_bounds = false;
  Eigen::Vector3d origin_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d origin_max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::max());
};

class LocalizationMap {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LocalizationMap>;
  using ConstPtr = std::shared_ptr<const LocalizationMap>;
  using SubMapConstPtr = glim::SubMap::ConstPtr;

  LocalizationMap();
  explicit LocalizationMap(const std::vector<glim::SubMap::Ptr>& submaps);
  explicit LocalizationMap(const std::vector<SubMapConstPtr>& submaps);

  bool empty() const;
  std::size_t size() const;

  void clear();
  void set_submaps(const std::vector<glim::SubMap::Ptr>& submaps);
  void set_submaps(const std::vector<SubMapConstPtr>& submaps);
  void set_metadata(const LocalizationMapMetadata& metadata);
  const std::vector<SubMapConstPtr>& submaps() const;
  SubMapConstPtr at(std::size_t i) const;
  std::vector<int> submap_ids() const;
  const LocalizationMapMetadata& metadata() const;
  LocalizationMapStats stats() const;

  void build_index(double resolution);
  void clear_index();
  bool has_index() const;
  SubmapIndex::Stats index_stats() const;

  std::vector<SubMapConstPtr> query_nearby(const Eigen::Isometry3d& T_map_sensor, int max_num_submaps, double max_distance) const;
  LocalTargetMap::Ptr build_target(const std::vector<SubMapConstPtr>& submaps, const Eigen::Isometry3d& T_map_target_center) const;
  LocalTargetMap::Ptr query_target(const Eigen::Isometry3d& T_map_sensor, int max_num_submaps, double max_distance) const;

private:
  std::vector<SubMapConstPtr> submaps_;
  SubmapIndex::Ptr index_;
  LocalizationMapMetadata metadata_;
};

}  // namespace glim_localization
