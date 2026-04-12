#include "odometry_estimation_localization_cpu.hpp"

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  glim_localization::OdometryEstimationLocalizationCPUParams params;
  return new glim_localization::OdometryEstimationLocalizationCPU(params);
}
