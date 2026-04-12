#include <glim_localization/relocalization/geometric_verifier.hpp>

#include <spdlog/spdlog.h>

namespace glim_localization {

GeometricVerifier::GeometricVerifier(const MatchingOptions& options, const MapRegistrationBase::Ptr& registration)
: options_(options),
  registration_(registration) {}

RelocalizationResult GeometricVerifier::verify(
  const glim::EstimationFrame::ConstPtr& frame,
  const std::vector<RelocalizationCandidate>& candidates) const {
  RelocalizationResult result;
  if (!registration_) {
    result.message = "registration_unavailable";
    return result;
  }

  if (!frame || !frame->frame) {
    result.message = "empty_frame";
    return result;
  }

  for (const auto& candidate : candidates) {
    if (!candidate.submap) {
      continue;
    }

    auto target = std::make_shared<LocalTargetMap>(
      candidate.T_map_imu_guess,
      std::vector<LocalTargetMap::SubMapConstPtr>{candidate.submap});

    auto registration_result = registration_->align(frame, target, candidate.T_map_imu_guess);
    spdlog::debug(
      "relocalization verify candidate submap={} descriptor_distance={:.3f} accepted={} reason={} score={:.3f} residual={:.6f}",
      candidate.submap_id,
      candidate.descriptor_distance,
      registration_result.accepted,
      registration_result.reject_reason.empty() ? "none" : registration_result.reject_reason,
      registration_result.score,
      registration_result.residual);

    if (!registration_result.accepted) {
      continue;
    }

    result.success = true;
    result.message = "verified";
    result.candidate = candidate;
    result.registration = registration_result;
    result.target_map = target;
    return result;
  }

  result.message = "no_verified_candidate";
  return result;
}

}  // namespace glim_localization
