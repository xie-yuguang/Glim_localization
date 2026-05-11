#pragma once

#include <limits>
#include <vector>
#include <string>
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

  struct QueryCandidateDebug {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int submap_id = -1;
    double descriptor_distance = std::numeric_limits<double>::quiet_NaN();
    double yaw = std::numeric_limits<double>::quiet_NaN();
    double translation_distance = std::numeric_limits<double>::quiet_NaN();
    bool passed_descriptor = false;
    bool passed_translation = false;
    RelocalizationCandidate candidate;
  };

  struct QueryDiagnostics {
    bool query_enable = false;
    std::string failure_reason = "not_requested";
    int query_points = 0;
    bool descriptor_valid = false;
    int descriptor_nonempty_bins = 0;
    int database_size = 0;
    int topk_requested = 0;
    int topk_returned = 0;
    int filtered_by_descriptor = 0;
    int filtered_by_translation = 0;
    int filtered_by_other = 0;
    int candidates_before_filter = 0;
    int candidates_after_filter = 0;
    std::vector<QueryCandidateDebug> topk;
  };

  QueryDiagnostics last_query_diagnostics() const;

private:
  struct Entry {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int submap_id = -1;
    LocalizationMap::SubMapConstPtr submap;
    Eigen::MatrixXd descriptor;
  };

  Eigen::MatrixXd make_descriptor(const gtsam_points::PointCloud::ConstPtr& cloud, const Eigen::Vector3d& center) const;
  int count_nonempty_bins(const Eigen::MatrixXd& descriptor) const;
  double descriptor_distance(const Eigen::MatrixXd& target, const Eigen::MatrixXd& query, int& best_shift) const;
  Eigen::Isometry3d initial_guess_from_candidate(const Entry& entry, double yaw, const glim::EstimationFrame& frame) const;

private:
  RelocalizationOptions options_;
  std::vector<Entry> entries_;
  mutable QueryDiagnostics last_query_diagnostics_;
};

}  // namespace glim_localization
