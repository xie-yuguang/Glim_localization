#include <glim_localization/initialization/runtime_initial_pose.hpp>

#include <mutex>

namespace glim_localization {
namespace {

std::mutex runtime_initial_pose_mutex;
std::optional<RuntimeInitialPose> runtime_initial_pose;

}  // namespace

void set_runtime_initial_pose(const RuntimeInitialPose& pose) {
  std::lock_guard<std::mutex> lock(runtime_initial_pose_mutex);
  runtime_initial_pose = pose;
}

std::optional<RuntimeInitialPose> get_runtime_initial_pose() {
  std::lock_guard<std::mutex> lock(runtime_initial_pose_mutex);
  return runtime_initial_pose;
}

void clear_runtime_initial_pose() {
  std::lock_guard<std::mutex> lock(runtime_initial_pose_mutex);
  runtime_initial_pose.reset();
}

}  // namespace glim_localization
