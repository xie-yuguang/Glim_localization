#pragma once

#include <memory>
#include <string>

#include <glim_localization/map/localization_map.hpp>
#include <glim_localization/map_loader/map_format_checker.hpp>
#include <glim_localization/map_loader/map_load_options.hpp>

namespace glim_localization {

class GlimMapLoader {
public:
  using Ptr = std::shared_ptr<GlimMapLoader>;

  GlimMapLoader();

  LocalizationMap::Ptr load(const MapLoadOptions& options) const;
  LocalizationMap::Ptr load(const std::string& map_path) const;

  const MapFormatChecker& checker() const;

private:
  MapFormatChecker checker_;
};

}  // namespace glim_localization
