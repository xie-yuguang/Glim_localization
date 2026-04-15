#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <glim_localization/output/trajectory_writer.hpp>

namespace fs = std::filesystem;

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    std::exit(1);
  }
}

fs::path make_temp_dir() {
  const fs::path dir = fs::temp_directory_path() / ("glim_localization_traj_writer_test_" + std::to_string(std::rand()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

double parse_double(const std::string& text) {
  return std::stod(text);
}

}  // namespace

int main() {
  const fs::path temp_dir = make_temp_dir();
  const fs::path traj_path = temp_dir / "traj.txt";

  glim_localization::TrajectoryWriter writer;
  expect(writer.open(traj_path.string()), "trajectory writer must open temp path");
  expect(writer.is_open(), "trajectory writer must report open state");

  glim_localization::LocalizationResult result;
  result.stamp = 12.3456789;
  result.status = glim_localization::LocalizationStatus::TRACKING;
  result.matching_score = 0.875;
  result.T_map_imu.setIdentity();
  result.T_map_imu.translation() = Eigen::Vector3d(1.25, -2.5, 0.75);
  result.T_map_imu.linear() = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  writer.write(result);
  expect(writer.num_written() == 1, "trajectory writer must increment row count");
  writer.close();

  std::ifstream ifs(traj_path);
  expect(static_cast<bool>(ifs), "trajectory file must exist after write");

  std::string line;
  std::getline(ifs, line);
  expect(!line.empty(), "trajectory line must not be empty");

  std::istringstream iss(line);
  std::vector<std::string> fields;
  std::string field;
  while (iss >> field) {
    fields.push_back(field);
  }

  expect(fields.size() == 10, "trajectory contract requires 10 whitespace-separated columns");
  expect(std::abs(parse_double(fields[0]) - result.stamp) < 1e-6, "trajectory stamp column mismatch");
  expect(std::abs(parse_double(fields[1]) - result.T_map_imu.translation().x()) < 1e-6, "trajectory x column mismatch");
  expect(std::abs(parse_double(fields[2]) - result.T_map_imu.translation().y()) < 1e-6, "trajectory y column mismatch");
  expect(std::abs(parse_double(fields[3]) - result.T_map_imu.translation().z()) < 1e-6, "trajectory z column mismatch");
  expect(std::abs(parse_double(fields[8]) - static_cast<int>(result.status)) < 1e-9, "trajectory status column mismatch");
  expect(std::abs(parse_double(fields[9]) - result.matching_score) < 1e-6, "trajectory score column mismatch");

  fs::remove_all(temp_dir);
  std::cout << "test_trajectory_writer passed" << std::endl;
  return 0;
}
