#include <glim_localization/map_loader/map_format_checker.hpp>

#include <fstream>
#include <filesystem>
#include <boost/format.hpp>
#include <spdlog/spdlog.h>

namespace glim_localization {
namespace fs = std::filesystem;

MapFormatCheckResult MapFormatChecker::check(const std::string& map_path) const {
  MapFormatCheckResult result;
  result.map_path = map_path;

  if (map_path.empty()) {
    result.errors.push_back("map path is empty");
    return result;
  }

  const fs::path root(map_path);
  if (!fs::exists(root)) {
    result.errors.push_back("map path does not exist: " + map_path);
    return result;
  }
  if (!fs::is_directory(root)) {
    result.errors.push_back("map path is not a directory: " + map_path);
    return result;
  }

  const fs::path graph_txt = root / "graph.txt";
  const fs::path graph_bin = root / "graph.bin";
  const fs::path values_bin = root / "values.bin";

  result.has_graph_txt = fs::exists(graph_txt);
  result.has_graph_bin = fs::exists(graph_bin);
  result.has_values_bin = fs::exists(values_bin);

  if (!result.has_graph_txt) {
    result.errors.push_back("missing graph.txt");
    return result;
  }

  if (!parse_graph_txt(graph_txt.string(), result)) {
    return result;
  }

  result.detected_format = "glim_dump";
  result.compatibility = "supported";

  if (!result.has_graph_bin) {
    result.warnings.push_back("missing graph.bin; localization loader can still load submaps, but full graph recovery is unavailable");
  }
  if (!result.has_values_bin) {
    result.warnings.push_back("missing values.bin; localization loader can still load submaps, but full graph values are unavailable");
  }

  check_submap_directories(map_path, result);
  result.valid = result.errors.empty();

  if (result.valid) {
    spdlog::info("valid GLIM map dump: {} submaps in {}", result.num_submaps, map_path);
  } else {
    spdlog::warn("invalid GLIM map dump: {}", map_path);
  }

  return result;
}

bool MapFormatChecker::parse_graph_txt(const std::string& graph_txt_path, MapFormatCheckResult& result) const {
  std::ifstream ifs(graph_txt_path);
  if (!ifs) {
    result.errors.push_back("failed to open graph.txt: " + graph_txt_path);
    return false;
  }

  std::string token;
  ifs >> token >> result.num_submaps;
  if (!ifs || token != "num_submaps:" || result.num_submaps < 0) {
    result.errors.push_back("invalid graph.txt: expected 'num_submaps: <non-negative int>'");
    return false;
  }

  ifs >> token >> result.num_all_frames;
  if (!ifs || token != "num_all_frames:" || result.num_all_frames < 0) {
    result.errors.push_back("invalid graph.txt: expected 'num_all_frames: <non-negative int>'");
    return false;
  }

  ifs >> token >> result.num_matching_cost_factors;
  if (!ifs || token != "num_matching_cost_factors:" || result.num_matching_cost_factors < 0) {
    result.errors.push_back("invalid graph.txt: expected 'num_matching_cost_factors: <non-negative int>'");
    return false;
  }

  return true;
}

void MapFormatChecker::check_submap_directories(const std::string& map_path, MapFormatCheckResult& result) const {
  for (int i = 0; i < result.num_submaps; i++) {
    const std::string submap_path = (boost::format("%s/%06d") % map_path % i).str();
    const fs::path submap_dir(submap_path);
    const fs::path data_txt = submap_dir / "data.txt";

    if (!fs::exists(submap_dir) || !fs::is_directory(submap_dir)) {
      result.missing_submap_directories++;
      result.errors.push_back("missing submap directory: " + submap_path);
      continue;
    }

    if (!fs::exists(data_txt)) {
      result.missing_submap_data_files++;
      result.errors.push_back("missing submap data.txt: " + data_txt.string());
    }
  }
}

}  // namespace glim_localization
