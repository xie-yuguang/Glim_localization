#pragma once

#include <memory>
#include <vector>

#include <glim_localization/core/localization_options.hpp>
#include <glim_localization/registration/map_registration_base.hpp>
#include <glim_localization/relocalization/relocalizer_base.hpp>

namespace glim_localization {

class GeometricVerifier {
public:
  GeometricVerifier(const MatchingOptions& options, const MapRegistrationBase::Ptr& registration);

  RelocalizationResult verify(
    const glim::EstimationFrame::ConstPtr& frame,
    const std::vector<RelocalizationCandidate>& candidates) const;

private:
  MatchingOptions options_;
  MapRegistrationBase::Ptr registration_;
};

}  // namespace glim_localization
