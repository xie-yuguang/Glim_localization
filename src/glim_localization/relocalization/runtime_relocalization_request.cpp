#include <glim_localization/relocalization/runtime_relocalization_request.hpp>

#include <mutex>

namespace glim_localization {
namespace {

std::mutex relocalization_request_mutex;
bool relocalization_requested = false;

}  // namespace

void request_runtime_relocalization() {
  std::lock_guard<std::mutex> lock(relocalization_request_mutex);
  relocalization_requested = true;
}

bool consume_runtime_relocalization_request() {
  std::lock_guard<std::mutex> lock(relocalization_request_mutex);
  const bool requested = relocalization_requested;
  relocalization_requested = false;
  return requested;
}

}  // namespace glim_localization
