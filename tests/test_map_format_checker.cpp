#include <cstdlib>
#include <fstream>
#include <iostream>
#include <filesystem>

#include <glim_localization/map_loader/map_format_checker.hpp>

namespace fs = std::filesystem;

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    std::exit(1);
  }
}

fs::path make_temp_dir(const std::string& name) {
  const fs::path dir = fs::temp_directory_path() / name;
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

void write_graph_txt(const fs::path& dir, int num_submaps) {
  std::ofstream ofs(dir / "graph.txt");
  ofs << "num_submaps: " << num_submaps << "\n";
  ofs << "num_all_frames: 0\n";
  ofs << "num_matching_cost_factors: 0\n";
}

}  // namespace

int main() {
  glim_localization::MapFormatChecker checker;

  const auto missing = checker.check((fs::temp_directory_path() / "glim_localization_missing_map").string());
  expect(!missing.valid, "missing path must be invalid");
  expect(!missing.errors.empty(), "missing path must report an error");

  const fs::path invalid_dir = make_temp_dir("glim_localization_invalid_map");
  {
    std::ofstream ofs(invalid_dir / "graph.txt");
    ofs << "not_a_graph\n";
  }
  const auto invalid = checker.check(invalid_dir.string());
  expect(!invalid.valid, "invalid graph.txt must be invalid");
  expect(!invalid.errors.empty(), "invalid graph.txt must report an error");

  const fs::path incomplete_dir = make_temp_dir("glim_localization_incomplete_map");
  write_graph_txt(incomplete_dir, 1);
  const auto incomplete = checker.check(incomplete_dir.string());
  expect(!incomplete.valid, "map with missing submap directory must be invalid");
  expect(!incomplete.errors.empty(), "missing submap must report an error");

  const fs::path valid_empty_dir = make_temp_dir("glim_localization_valid_empty_map");
  write_graph_txt(valid_empty_dir, 0);
  const auto valid_empty = checker.check(valid_empty_dir.string());
  expect(valid_empty.valid, "empty but well-formed map must be valid");
  expect(valid_empty.num_submaps == 0, "empty map must report zero submaps");
  expect(valid_empty.has_graph_txt, "valid map must report graph.txt");

  fs::remove_all(invalid_dir);
  fs::remove_all(incomplete_dir);
  fs::remove_all(valid_empty_dir);

  std::cout << "test_map_format_checker passed" << std::endl;
  return 0;
}
