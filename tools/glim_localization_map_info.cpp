#include <cstdlib>
#include <cstddef>
#include <iostream>
#include <limits>

#include <Eigen/Core>

#include <glim_localization/map_loader/glim_map_loader.hpp>

namespace {

void print_usage() {
  std::cerr << "usage: glim_localization_map_info <map_path> [index_resolution]" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    print_usage();
    return 1;
  }

  glim_localization::MapLoadOptions options;
  options.map_path = argv[1];
  options.strict = true;

  const double index_resolution = argc >= 3 ? std::atof(argv[2]) : 20.0;

  glim_localization::GlimMapLoader loader;
  const auto check = loader.checker().check(options.map_path);
  std::cout << "map_path: " << check.map_path << std::endl;
  std::cout << "valid: " << (check.valid ? "true" : "false") << std::endl;
  std::cout << "detected_format: " << check.detected_format << std::endl;
  std::cout << "compatibility: " << check.compatibility << std::endl;
  std::cout << "graph_txt: " << (check.has_graph_txt ? "true" : "false") << std::endl;
  std::cout << "graph_bin: " << (check.has_graph_bin ? "true" : "false") << std::endl;
  std::cout << "values_bin: " << (check.has_values_bin ? "true" : "false") << std::endl;
  std::cout << "num_submaps: " << check.num_submaps << std::endl;
  std::cout << "num_all_frames: " << check.num_all_frames << std::endl;
  std::cout << "num_matching_cost_factors: " << check.num_matching_cost_factors << std::endl;
  std::cout << "missing_submap_directories: " << check.missing_submap_directories << std::endl;
  std::cout << "missing_submap_data_files: " << check.missing_submap_data_files << std::endl;

  for (const auto& warning : check.warnings) {
    std::cout << "warning: " << warning << std::endl;
  }
  for (const auto& error : check.errors) {
    std::cout << "error: " << error << std::endl;
  }

  if (!check.valid) {
    return 2;
  }

  const auto map = loader.load(options);
  if (!map || map->empty()) {
    std::cerr << "failed to load localization map" << std::endl;
    return 3;
  }

  const auto metadata = map->metadata();
  const auto map_stats = map->stats();

  map->build_index(index_resolution);
  const auto stats = map->index_stats();

  std::cout << "loaded_submaps: " << map->size() << std::endl;
  std::cout << "loaded_vs_requested: " << metadata.loaded_submaps << "/" << metadata.requested_submaps << std::endl;
  std::cout << "skipped_submaps: " << metadata.skipped_submaps << std::endl;
  std::cout << "merged_submap_points: " << map_stats.num_points << std::endl;
  if (map_stats.has_bounds) {
    std::cout << "origin_min: " << map_stats.origin_min.transpose() << std::endl;
    std::cout << "origin_max: " << map_stats.origin_max.transpose() << std::endl;
  }
  std::cout << "index_resolution: " << stats.resolution << std::endl;
  std::cout << "index_cells: " << stats.num_cells << std::endl;
  std::cout << "index_max_cell_size: " << stats.max_cell_size << std::endl;

  return 0;
}
