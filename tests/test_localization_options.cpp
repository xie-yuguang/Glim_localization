#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include <glim/util/config.hpp>
#include <glim_localization/core/localization_options.hpp>

namespace fs = std::filesystem;

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    std::exit(1);
  }
}

fs::path unique_temp_dir() {
  const auto base = fs::temp_directory_path();
  for (int i = 0; i < 100; i++) {
    const auto candidate = base / ("glim_localization_options_test_" + std::to_string(std::rand()));
    if (!fs::exists(candidate)) {
      fs::create_directories(candidate);
      return candidate;
    }
  }

  throw std::runtime_error("failed to create temp directory");
}

void write_file(const fs::path& path, const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

}  // namespace

int main() {
  const auto temp_dir = unique_temp_dir();

  write_file(
    temp_dir / "config.json",
    "{\n"
    "  \"global\": {\n"
    "    \"config_path\": \"\",\n"
    "    \"config_localization\": \"localization.json\"\n"
    "  }\n"
    "}\n");

  write_file(
    temp_dir / "localization.json",
    "{\n"
    "  \"localization\": {\n"
    "    \"map_path\": \"/tmp/test_map\",\n"
    "    \"trajectory_path\": \"/tmp/test_traj.txt\",\n"
    "    \"map\": {\n"
    "      \"load_voxelmaps\": false,\n"
    "      \"load_raw_frames\": true,\n"
    "      \"strict\": false\n"
    "    },\n"
    "    \"initial_pose\": {\n"
    "      \"source\": \"config\",\n"
    "      \"xyz\": [1.0, 2.0, 3.0],\n"
    "      \"rpy\": [0.1, 0.2, 0.3]\n"
    "    },\n"
    "    \"target_map\": {\n"
    "      \"max_num_submaps\": 6,\n"
    "      \"use_submap_index\": false\n"
    "    },\n"
    "    \"matching\": {\n"
    "      \"method\": \"cpu_gicp\",\n"
    "      \"min_score\": 0.42,\n"
    "      \"num_threads\": 7\n"
    "    },\n"
    "    \"ros\": {\n"
    "      \"pose_topic\": \"/test_pose\"\n"
    "    }\n"
    "  }\n"
    "}\n");

  glim::GlobalConfig::instance(temp_dir.string(), true);
  const auto options = glim_localization::LocalizationOptions::load();

  expect(options.map.map_path == "/tmp/test_map", "map_path must be parsed");
  expect(options.trajectory_path == "/tmp/test_traj.txt", "trajectory_path must be parsed");
  expect(options.map.load_voxelmaps == false, "map.load_voxelmaps must be parsed");
  expect(options.map.load_raw_frames == true, "map.load_raw_frames must be parsed");
  expect(options.map.strict == false, "map.strict must be parsed");
  expect((options.initial_pose.xyz - Eigen::Vector3d(1.0, 2.0, 3.0)).norm() < 1e-9, "initial pose xyz must be parsed");
  expect(options.target_map.max_num_submaps == 6, "target_map.max_num_submaps must be parsed");
  expect(options.target_map.use_submap_index == false, "target_map.use_submap_index must be parsed");
  expect(std::abs(options.matching.min_score - 0.42) < 1e-9, "matching.min_score must be parsed");
  expect(options.matching.num_threads == 7, "matching.num_threads must be parsed");
  expect(options.ros.pose_topic == "/test_pose", "ros.pose_topic must be parsed");

  fs::remove_all(temp_dir);
  std::cout << "test_localization_options passed" << std::endl;
  return 0;
}
