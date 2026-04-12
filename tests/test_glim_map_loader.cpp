#include <cstdlib>
#include <iostream>
#include <filesystem>

#include <glim_localization/map_loader/glim_map_loader.hpp>

namespace fs = std::filesystem;

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    std::exit(1);
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string map_path;
  if (argc > 1) {
    map_path = argv[1];
  } else if (const char* env = std::getenv("GLIM_LOCALIZATION_TEST_MAP")) {
    map_path = env;
  }

  if (map_path.empty()) {
    std::cout << "GLIM_LOCALIZATION_TEST_MAP is not set; skipping real GLIM dump loading test" << std::endl;
    return 0;
  }

  expect(fs::exists(map_path), "fixture path must exist");

  glim_localization::MapLoadOptions options;
  options.map_path = map_path;
  options.strict = true;

  glim_localization::GlimMapLoader loader;
  auto map = loader.load(options);

  expect(static_cast<bool>(map), "loader must return a LocalizationMap");
  expect(!map->empty(), "fixture map must contain at least one submap");

  const auto& submaps = map->submaps();
  expect(static_cast<bool>(submaps.front()), "first submap must be valid");
  expect(submaps.front()->T_world_origin.matrix().allFinite(), "first submap pose must be finite");
  expect(submaps.front()->frame != nullptr, "first submap point cloud must be loaded");

  std::cout << "test_glim_map_loader passed: loaded " << map->size() << " submaps" << std::endl;
  return 0;
}
