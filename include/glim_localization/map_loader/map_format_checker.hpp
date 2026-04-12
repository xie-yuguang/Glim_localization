#pragma once

#include <string>
#include <vector>

namespace glim_localization {

struct MapFormatCheckResult {
  bool valid = false;
  std::string map_path;
  int num_submaps = 0;
  int num_all_frames = 0;
  int num_matching_cost_factors = 0;
  bool has_graph_txt = false;
  bool has_graph_bin = false;
  bool has_values_bin = false;
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
};

class MapFormatChecker {
public:
  MapFormatCheckResult check(const std::string& map_path) const;

private:
  bool parse_graph_txt(const std::string& graph_txt_path, MapFormatCheckResult& result) const;
  void check_submap_directories(const std::string& map_path, MapFormatCheckResult& result) const;
};

}  // namespace glim_localization
