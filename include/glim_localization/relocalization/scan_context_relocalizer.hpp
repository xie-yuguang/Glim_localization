#pragma once

#include <vector>
#include <Eigen/Dense>

#include <gtsam_points/types/point_cloud.hpp>

#include <glim_localization/core/localization_options.hpp>
#include <glim_localization/relocalization/relocalizer_base.hpp>

namespace glim_localization {

class ScanContextRelocalizer : public RelocalizerBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ScanContextRelocalizer(const RelocalizationOptions& options = RelocalizationOptions());
  ~ScanContextRelocalizer() override;

  bool build(const LocalizationMap::ConstPtr& map) override;
  bool ready() const override;
  std::vector<RelocalizationCandidate> query(const glim::EstimationFrame::ConstPtr& frame, int max_candidates) const override;

private:
  struct Entry {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int submap_id = -1;
    LocalizationMap::SubMapConstPtr submap;
    Eigen::MatrixXd descriptor;
  };

  Eigen::MatrixXd make_descriptor(const gtsam_points::PointCloud::ConstPtr& cloud, const Eigen::Vector3d& center) const;
  double descriptor_distance(const Eigen::MatrixXd& target, const Eigen::MatrixXd& query, int& best_shift) const;
  Eigen::Isometry3d initial_guess_from_candidate(const Entry& entry, double yaw, const glim::EstimationFrame& frame) const;

private:
  RelocalizationOptions options_;
  std::vector<Entry> entries_;
};

}  // namespace glim_localization
