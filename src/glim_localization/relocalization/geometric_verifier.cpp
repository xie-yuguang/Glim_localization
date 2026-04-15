#include <glim_localization/relocalization/geometric_verifier.hpp>

#include <spdlog/spdlog.h>

namespace glim_localization {

GeometricVerifier::GeometricVerifier(
  const MatchingOptions& matching_options,
  const RelocalizationOptions& relocalization_options,
  const LocalizationMap::ConstPtr& map,
  const MapRegistrationBase::Ptr& registration)
: matching_options_(matching_options),
  relocalization_options_(relocalization_options),
  map_(map),
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

  RegistrationResult best_registration;
  RelocalizationCandidate best_candidate;
  LocalTargetMap::Ptr best_target;
  bool best_found = false;

  for (int i = 0; i < static_cast<int>(candidates.size()); i++) {
    const auto& candidate = candidates[i];
    if (!candidate.submap) {
      continue;
    }

    LocalTargetMap::Ptr target;
    if (map_ && !map_->empty()) {
      const int verification_submaps = std::max(1, relocalization_options_.verification_target_max_submaps);
      const double verification_distance =
        relocalization_options_.verification_target_max_distance > 0.0 ? relocalization_options_.verification_target_max_distance : 20.0;
      target = map_->query_target(candidate.T_map_imu_guess, verification_submaps, verification_distance);
    }

    if (!target || target->empty()) {
      target = std::make_shared<LocalTargetMap>(
        candidate.T_map_imu_guess,
        std::vector<LocalTargetMap::SubMapConstPtr>{candidate.submap});
    }

    auto registration_result = registration_->align(frame, target, candidate.T_map_imu_guess);
    result.evaluated_candidates++;
    spdlog::debug(
      "relocalization verify candidate rank={} submap={} descriptor_distance={:.3f} translation_distance={:.3f} accepted={} reason={} score={:.3f} residual={:.6f} target_submaps={}",
      i,
      candidate.submap_id,
      candidate.descriptor_distance,
      candidate.translation_distance,
      registration_result.accepted,
      registration_result.reject_reason.empty() ? "none" : registration_result.reject_reason,
      registration_result.score,
      registration_result.residual,
      target->active_submap_ids().size());

    if (!registration_result.accepted) {
      continue;
    }

    result.accepted_candidates++;
    const bool better =
      !best_found || registration_result.score > best_registration.score ||
      (registration_result.score == best_registration.score && registration_result.residual < best_registration.residual);
    if (!better) {
      continue;
    }

    best_found = true;
    best_candidate = candidate;
    best_registration = registration_result;
    best_target = target;
    result.verified_candidate_rank = i;
  }

  if (best_found) {
    result.success = true;
    result.message = "verified";
    result.candidate = best_candidate;
    result.registration = best_registration;
    result.target_map = best_target;
    return result;
  }

  result.message = "no_verified_candidate";
  return result;
}

}  // namespace glim_localization
