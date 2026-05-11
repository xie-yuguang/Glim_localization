#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

#include <glim/odometry/estimation_frame.hpp>

#include <glim_localization/map_loader/glim_map_loader.hpp>
#include <glim_localization/relocalization/scan_context_relocalizer.hpp>

namespace {

void print_usage() {
  std::cerr << "usage: check_scan_context_relocalizer <map_path> [top_k]" << std::endl;
}

double mean_or_nan(const std::vector<double>& values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double max_or_nan(const std::vector<double>& values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return *std::max_element(values.begin(), values.end());
}

double min_or_nan(const std::vector<double>& values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return *std::min_element(values.begin(), values.end());
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    print_usage();
    return 1;
  }

  const std::string map_path = argv[1];
  const int top_k = argc >= 3 ? std::max(1, std::atoi(argv[2])) : 10;

  glim_localization::MapLoadOptions map_options;
  map_options.map_path = map_path;
  map_options.strict = true;

  glim_localization::GlimMapLoader loader;
  const auto map = loader.load(map_options);
  if (!map || map->empty()) {
    std::cerr << "failed to load localization map: " << map_path << std::endl;
    return 2;
  }

  glim_localization::RelocalizationOptions relocalization_options;
  relocalization_options.max_candidates = top_k;
  glim_localization::ScanContextRelocalizer relocalizer(relocalization_options);
  if (!relocalizer.build(map)) {
    std::cerr << "failed to build ScanContext relocalizer" << std::endl;
    return 3;
  }

  int query_count = 0;
  int top1_self = 0;
  int top5_self = 0;
  int failed_queries = 0;
  std::vector<double> top1_distances;
  std::vector<double> top5_distances;
  std::vector<double> top1_yaws;
  std::vector<int> failed_submaps;

  for (const auto& submap : map->submaps()) {
    if (!submap || !submap->frame || submap->frame->size() == 0) {
      continue;
    }

    auto frame = std::make_shared<glim::EstimationFrame>();
    frame->id = submap->id;
    frame->stamp = 0.0;
    frame->T_lidar_imu.setIdentity();
    frame->T_world_lidar = submap->T_world_origin;
    frame->T_world_imu = submap->T_world_origin;
    frame->frame_id = glim::FrameID::LIDAR;
    frame->frame = submap->frame;

    const auto candidates = relocalizer.query(frame, top_k);
    const auto diagnostics = relocalizer.last_query_diagnostics();
    query_count++;

    if (diagnostics.topk.empty()) {
      failed_queries++;
      failed_submaps.push_back(submap->id);
      continue;
    }

    const auto& top1 = diagnostics.topk.front();
    top1_distances.push_back(top1.descriptor_distance);
    top1_yaws.push_back(top1.yaw);
    if (top1.submap_id == submap->id) {
      top1_self++;
    }

    bool in_top5 = false;
    for (int i = 0; i < std::min<int>(5, diagnostics.topk.size()); i++) {
      if (diagnostics.topk[i].submap_id == submap->id) {
        in_top5 = true;
        top5_distances.push_back(diagnostics.topk[i].descriptor_distance);
        break;
      }
    }
    if (in_top5) {
      top5_self++;
    }

    if (candidates.empty()) {
      failed_queries++;
      failed_submaps.push_back(submap->id);
    }
  }

  std::cout << "map_path: " << map_path << std::endl;
  std::cout << "database_size: " << map->size() << std::endl;
  std::cout << "query_count: " << query_count << std::endl;
  std::cout << "top_k: " << top_k << std::endl;
  std::cout << "top1_self: " << top1_self << std::endl;
  std::cout << "top5_self: " << top5_self << std::endl;
  std::cout << "failed_self_query: " << failed_queries << std::endl;
  std::cout << "top1_distance_min: " << min_or_nan(top1_distances) << std::endl;
  std::cout << "top1_distance_mean: " << mean_or_nan(top1_distances) << std::endl;
  std::cout << "top1_distance_max: " << max_or_nan(top1_distances) << std::endl;
  std::cout << "top5_self_distance_mean: " << mean_or_nan(top5_distances) << std::endl;
  std::cout << "top1_yaw_min: " << min_or_nan(top1_yaws) << std::endl;
  std::cout << "top1_yaw_mean: " << mean_or_nan(top1_yaws) << std::endl;
  std::cout << "top1_yaw_max: " << max_or_nan(top1_yaws) << std::endl;
  if (!failed_submaps.empty()) {
    std::cout << "failed_submap_ids:";
    for (const int id : failed_submaps) {
      std::cout << ' ' << id;
    }
    std::cout << std::endl;
  }

  return failed_queries == 0 ? 0 : 4;
}
