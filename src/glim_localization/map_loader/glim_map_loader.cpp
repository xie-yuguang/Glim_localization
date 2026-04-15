#include <glim_localization/map_loader/glim_map_loader.hpp>

#include <algorithm>
#include <boost/format.hpp>
#include <spdlog/spdlog.h>

#include <glim/mapping/sub_map.hpp>

namespace glim_localization {

GlimMapLoader::GlimMapLoader() {}

LocalizationMap::Ptr GlimMapLoader::load(const std::string& map_path) const {
  MapLoadOptions options;
  options.map_path = map_path;
  return load(options);
}

LocalizationMap::Ptr GlimMapLoader::load(const MapLoadOptions& options) const {
  const auto check = checker_.check(options.map_path);
  if (!check.valid) {
    for (const auto& error : check.errors) {
      spdlog::error("map format error: {}", error);
    }
    if (options.strict) {
      return nullptr;
    }
  }

  for (const auto& warning : check.warnings) {
    spdlog::warn("map format warning: {}", warning);
  }

  std::vector<glim::SubMap::Ptr> submaps;
  submaps.reserve(std::max(0, check.num_submaps));
  int skipped_submaps = 0;

  spdlog::info("loading {} GLIM submaps from {}", check.num_submaps, options.map_path);
  for (int i = 0; i < check.num_submaps; i++) {
    const std::string submap_path = (boost::format("%s/%06d") % options.map_path % i).str();
    auto submap = glim::SubMap::load(submap_path);
    if (!submap) {
      spdlog::error("failed to load submap {}", submap_path);
      if (options.strict) {
        return nullptr;
      }
      skipped_submaps++;
      continue;
    }

    // GLIM dump stores submap data in numbered directories.  For localization
    // we keep the stored id/session pose and avoid loading the full
    // GlobalMapping graph as a runtime object.
    if (!options.load_voxelmaps) {
      submap->voxelmaps.clear();
    }
    if (!options.load_raw_frames) {
      submap->frames.clear();
      submap->odom_frames.clear();
    }

    submaps.push_back(submap);
  }

  auto map = std::make_shared<LocalizationMap>(submaps);
  LocalizationMapMetadata metadata;
  metadata.map_path = options.map_path;
  metadata.detected_format = check.detected_format;
  metadata.compatibility = check.compatibility;
  metadata.strict = options.strict;
  metadata.load_voxelmaps = options.load_voxelmaps;
  metadata.load_raw_frames = options.load_raw_frames;
  metadata.requested_submaps = check.num_submaps;
  metadata.loaded_submaps = static_cast<int>(submaps.size());
  metadata.skipped_submaps = skipped_submaps;
  metadata.has_graph_txt = check.has_graph_txt;
  metadata.has_graph_bin = check.has_graph_bin;
  metadata.has_values_bin = check.has_values_bin;
  map->set_metadata(metadata);

  const auto stats = map->stats();
  spdlog::info(
    "loaded localization map: format={} compatibility={} loaded={}/{} skipped={} points={}",
    metadata.detected_format,
    metadata.compatibility,
    metadata.loaded_submaps,
    metadata.requested_submaps,
    metadata.skipped_submaps,
    stats.num_points);
  if (stats.has_bounds) {
    spdlog::info(
      "localization map bounds: origin_min=({:.3f}, {:.3f}, {:.3f}) origin_max=({:.3f}, {:.3f}, {:.3f})",
      stats.origin_min.x(),
      stats.origin_min.y(),
      stats.origin_min.z(),
      stats.origin_max.x(),
      stats.origin_max.y(),
      stats.origin_max.z());
  }
  return map;
}

const MapFormatChecker& GlimMapLoader::checker() const {
  return checker_;
}

}  // namespace glim_localization
