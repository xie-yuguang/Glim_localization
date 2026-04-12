#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <glim_localization/map_loader/glim_map_loader.hpp>

namespace {

void print_usage() {
  std::cerr << "usage: benchmark_localization <map_path> [num_queries] [max_distance] [max_num_submaps] [index_resolution]" << std::endl;
}

double elapsed_ms(const std::chrono::steady_clock::time_point& begin, const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - begin).count();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    print_usage();
    return 1;
  }

  const std::string map_path = argv[1];
  const int num_queries = argc >= 3 ? std::atoi(argv[2]) : 1000;
  const double max_distance = argc >= 4 ? std::atof(argv[3]) : 40.0;
  const int max_num_submaps = argc >= 5 ? std::atoi(argv[4]) : 8;
  const double index_resolution = argc >= 6 ? std::atof(argv[5]) : 20.0;

  glim_localization::MapLoadOptions options;
  options.map_path = map_path;

  glim_localization::GlimMapLoader loader;
  const auto load_begin = std::chrono::steady_clock::now();
  const auto map = loader.load(options);
  const auto load_end = std::chrono::steady_clock::now();
  if (!map || map->empty()) {
    std::cerr << "failed to load map: " << map_path << std::endl;
    return 2;
  }

  std::vector<Eigen::Isometry3d> queries;
  queries.reserve(num_queries);
  for (int i = 0; i < num_queries; i++) {
    const auto& submap = map->at(static_cast<std::size_t>(i) % map->size());
    Eigen::Isometry3d query = Eigen::Isometry3d::Identity();
    query.translation() = submap->T_world_origin.translation();
    queries.push_back(query);
  }

  const auto linear_begin = std::chrono::steady_clock::now();
  std::size_t linear_hits = 0;
  for (const auto& query : queries) {
    linear_hits += map->query_nearby(query, max_num_submaps, max_distance).size();
  }
  const auto linear_end = std::chrono::steady_clock::now();

  map->build_index(index_resolution);
  const auto stats = map->index_stats();

  const auto index_begin = std::chrono::steady_clock::now();
  std::size_t index_hits = 0;
  for (const auto& query : queries) {
    index_hits += map->query_nearby(query, max_num_submaps, max_distance).size();
  }
  const auto index_end = std::chrono::steady_clock::now();

  const double load_time = elapsed_ms(load_begin, load_end);
  const double linear_time = elapsed_ms(linear_begin, linear_end);
  const double index_time = elapsed_ms(index_begin, index_end);

  std::cout << "map_path: " << map_path << std::endl;
  std::cout << "submaps: " << map->size() << std::endl;
  std::cout << "load_ms: " << load_time << std::endl;
  std::cout << "queries: " << num_queries << std::endl;
  std::cout << "max_distance: " << max_distance << std::endl;
  std::cout << "max_num_submaps: " << max_num_submaps << std::endl;
  std::cout << "linear_total_ms: " << linear_time << std::endl;
  std::cout << "linear_avg_ms: " << linear_time / std::max(1, num_queries) << std::endl;
  std::cout << "linear_hits: " << linear_hits << std::endl;
  std::cout << "index_resolution: " << stats.resolution << std::endl;
  std::cout << "index_cells: " << stats.num_cells << std::endl;
  std::cout << "index_max_cell_size: " << stats.max_cell_size << std::endl;
  std::cout << "index_total_ms: " << index_time << std::endl;
  std::cout << "index_avg_ms: " << index_time / std::max(1, num_queries) << std::endl;
  std::cout << "index_hits: " << index_hits << std::endl;

  return 0;
}
