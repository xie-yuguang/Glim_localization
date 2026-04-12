#pragma once

#include <string>

namespace glim_localization {

struct MapLoadOptions {
  std::string map_path;
  bool load_voxelmaps = true;
  bool load_raw_frames = false;
  bool strict = true;
};

}  // namespace glim_localization
