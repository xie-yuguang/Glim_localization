#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/estimation_frame.hpp>
#include <glim_localization/map/localization_map.hpp>
#include <glim_localization/map/local_target_map.hpp>
#include <glim_localization/registration/registration_result.hpp>

namespace glim_localization {

struct RelocalizationCandidate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<RelocalizationCandidate>;

  int submap_id = -1;
  double descriptor_distance = 0.0;
  double yaw = 0.0;
  double translation_distance = 0.0;
  double ranking_score = 0.0;
  LocalizationMap::SubMapConstPtr submap;
  Eigen::Isometry3d T_map_imu_guess = Eigen::Isometry3d::Identity();
};

struct RelocalizationResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool success = false;
  std::string message;
  RelocalizationCandidate candidate;
  RegistrationResult registration;
  LocalTargetMap::Ptr target_map;
  int evaluated_candidates = 0;
  int accepted_candidates = 0;
  int verified_candidate_rank = -1;
};

class RelocalizerBase {
public:
  using Ptr = std::shared_ptr<RelocalizerBase>;
  using ConstPtr = std::shared_ptr<const RelocalizerBase>;

  virtual ~RelocalizerBase() {}

  virtual bool build(const LocalizationMap::ConstPtr& map) = 0;
  virtual bool ready() const = 0;
  virtual std::vector<RelocalizationCandidate> query(const glim::EstimationFrame::ConstPtr& frame, int max_candidates) const = 0;
};

}  // namespace glim_localization
