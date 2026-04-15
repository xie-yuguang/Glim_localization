#include <cstdlib>
#include <iostream>
#include <string>

#include <glim_localization/core/localization_options.hpp>
#include <glim_localization/core/localization_status.hpp>

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    std::exit(1);
  }
}

}  // namespace

int main() {
  const glim_localization::LocalizationOptions options;

  expect(options.map_frame == "map", "default map_frame must be map");
  expect(options.odom_frame == "odom", "default odom_frame must be odom");
  expect(options.base_frame == "base_link", "default base_frame must be base_link");
  expect(options.sensor_frame == "lidar", "default sensor_frame must be lidar");
  expect(options.trajectory_path == "/tmp/glim_localization_traj.txt", "default trajectory path contract changed");

  expect(options.ros.initial_pose_topic == "/initialpose", "default initial_pose topic contract changed");
  expect(options.ros.relocalization_service == "/localization/relocalize", "default relocalization service contract changed");
  expect(options.ros.status_topic == "/localization/status", "default status topic contract changed");
  expect(options.ros.diagnostic_topic == "/localization/diagnostics", "default diagnostics topic contract changed");
  expect(options.ros.publish_diagnostics, "default diagnostics publish contract changed");
  expect(options.ros.pose_topic == "/localization/pose", "default pose topic contract changed");
  expect(options.ros.odom_topic == "/localization/odom", "default odom topic contract changed");
  expect(options.ros.trajectory_topic == "/localization/trajectory", "default trajectory topic contract changed");
  expect(options.ros.input_scan_topic == "/localization/debug/input_scan", "default input scan topic contract changed");
  expect(options.ros.current_scan_topic == "/localization/debug/current_scan", "default current scan topic contract changed");
  expect(options.ros.target_map_topic == "/localization/debug/local_target_map", "default target map topic contract changed");
  expect(options.ros.active_submaps_topic == "/localization/debug/active_submaps", "default active submaps topic contract changed");

  expect(std::string(glim_localization::to_string(glim_localization::LocalizationStatus::WAIT_MAP)) == "WAIT_MAP", "WAIT_MAP string contract changed");
  expect(
    std::string(glim_localization::to_string(glim_localization::LocalizationStatus::WAIT_INITIAL_POSE)) == "WAIT_INITIAL_POSE",
    "WAIT_INITIAL_POSE string contract changed");
  expect(
    std::string(glim_localization::to_string(glim_localization::LocalizationStatus::INITIALIZING)) == "INITIALIZING",
    "INITIALIZING string contract changed");
  expect(std::string(glim_localization::to_string(glim_localization::LocalizationStatus::DEGRADED)) == "DEGRADED", "DEGRADED string contract changed");
  expect(std::string(glim_localization::to_string(glim_localization::LocalizationStatus::TRACKING)) == "TRACKING", "TRACKING string contract changed");
  expect(std::string(glim_localization::to_string(glim_localization::LocalizationStatus::LOST)) == "LOST", "LOST string contract changed");
  expect(
    std::string(glim_localization::to_string(glim_localization::LocalizationStatus::RELOCALIZING)) == "RELOCALIZING",
    "RELOCALIZING string contract changed");
  expect(
    std::string(glim_localization::to_string(glim_localization::LocalizationStatus::RECOVERING)) == "RECOVERING",
    "RECOVERING string contract changed");

  expect(
    glim_localization::LocalizationStatus::WAITING_FOR_MAP == glim_localization::LocalizationStatus::WAIT_MAP,
    "WAITING_FOR_MAP alias contract changed");
  expect(
    glim_localization::LocalizationStatus::WAITING_FOR_INITIAL_POSE == glim_localization::LocalizationStatus::WAIT_INITIAL_POSE,
    "WAITING_FOR_INITIAL_POSE alias contract changed");

  std::cout << "test_localization_contracts passed" << std::endl;
  return 0;
}
