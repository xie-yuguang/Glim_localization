#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/mapping/sub_map.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim_localization/map/localization_map.hpp>
#include <glim_localization/relocalization/geometric_verifier.hpp>

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    std::exit(1);
  }
}

class FakeRegistration final : public glim_localization::MapRegistrationBase {
public:
  FakeRegistration() : glim_localization::MapRegistrationBase(glim_localization::MatchingOptions()) {}

  glim_localization::RegistrationResult align(
    const glim::EstimationFrame::ConstPtr& frame,
    const glim_localization::LocalTargetMap::ConstPtr& target,
    const Eigen::Isometry3d& initial_T_map_imu) override {
    (void)frame;

    glim_localization::RegistrationResult result;
    result.backend_name = "fake";
    result.score_type = "fake_score";
    result.T_map_imu = initial_T_map_imu;
    result.converged = true;
    result.accepted = false;

    const auto ids = target ? target->active_submap_ids() : std::vector<int>();
    if (ids.empty()) {
      result.reject_reason = "empty_target";
      return result;
    }

    if (ids.front() == 1) {
      result.score = 0.45;
      result.residual = 10.0;
      result.accepted = true;
      return result;
    }

    if (ids.front() == 2) {
      result.score = 0.85;
      result.residual = 5.0;
      result.accepted = true;
      return result;
    }

    result.reject_reason = "unmatched_target";
    return result;
  }
};

glim::SubMap::Ptr make_submap(int id, double x) {
  auto submap = std::make_shared<glim::SubMap>();
  submap->id = id;
  submap->T_world_origin.setIdentity();
  submap->T_world_origin.translation() = Eigen::Vector3d(x, 0.0, 0.0);
  submap->frame = std::make_shared<gtsam_points::PointCloudCPU>(std::vector<Eigen::Vector4d>{
    Eigen::Vector4d(x, 0.0, 0.0, 1.0),
    Eigen::Vector4d(x + 0.5, 0.0, 0.0, 1.0),
  });
  return submap;
}

}  // namespace

int main() {
  auto map = std::make_shared<glim_localization::LocalizationMap>(std::vector<glim::SubMap::Ptr>{
    make_submap(1, 0.0),
    make_submap(2, 10.0),
  });

  glim_localization::MatchingOptions matching_options;
  glim_localization::RelocalizationOptions relocalization_options;
  relocalization_options.verification_target_max_submaps = 1;
  relocalization_options.verification_target_max_distance = 1.0;

  auto registration = std::make_shared<FakeRegistration>();
  glim_localization::GeometricVerifier verifier(matching_options, relocalization_options, map, registration);

  auto frame = std::make_shared<glim::EstimationFrame>();
  frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(std::vector<Eigen::Vector4d>{
    Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
  });

  glim_localization::RelocalizationCandidate candidate_a;
  candidate_a.submap_id = 1;
  candidate_a.submap = map->submaps()[0];
  candidate_a.descriptor_distance = 0.10;
  candidate_a.T_map_imu_guess = Eigen::Isometry3d::Identity();

  glim_localization::RelocalizationCandidate candidate_b;
  candidate_b.submap_id = 2;
  candidate_b.submap = map->submaps()[1];
  candidate_b.descriptor_distance = 0.20;
  candidate_b.T_map_imu_guess = Eigen::Translation3d(10.0, 0.0, 0.0) * Eigen::Isometry3d::Identity();

  const auto result = verifier.verify(frame, {candidate_a, candidate_b});
  expect(result.success, "geometric verifier must select an accepted candidate");
  expect(result.candidate.submap_id == 2, "geometric verifier must keep the best accepted candidate");
  expect(result.verified_candidate_rank == 1, "geometric verifier rank contract changed");
  expect(result.accepted_candidates == 2, "accepted candidate count mismatch");
  expect(result.evaluated_candidates == 2, "evaluated candidate count mismatch");
  expect(result.registration.score > 0.8, "best accepted score mismatch");

  std::cout << "test_geometric_verifier passed" << std::endl;
  return 0;
}
