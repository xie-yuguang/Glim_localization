#pragma once

#include <memory>
#include <vector>

#include <glim_localization/core/localization_options.hpp>
#include <glim_localization/map/localization_map.hpp>
#include <glim_localization/registration/map_registration_base.hpp>
#include <glim_localization/relocalization/relocalizer_base.hpp>

namespace glim_localization {

class GeometricVerifier {
public:
  GeometricVerifier(
    const MatchingOptions& matching_options,
    const RelocalizationOptions& relocalization_options,
    const LocalizationMap::ConstPtr& map,
    const MapRegistrationBase::Ptr& registration);

  RelocalizationResult verify(
    const glim::EstimationFrame::ConstPtr& frame,
    const std::vector<RelocalizationCandidate>& candidates) const;

private:
  MatchingOptions matching_options_;
  RelocalizationOptions relocalization_options_;
  LocalizationMap::ConstPtr map_;
  MapRegistrationBase::Ptr registration_;
};

}  // namespace glim_localization
