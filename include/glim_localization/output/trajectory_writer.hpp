#pragma once

#include <cstddef>
#include <fstream>
#include <string>

#include <glim_localization/output/localization_result.hpp>

namespace glim_localization {

class TrajectoryWriter {
public:
  TrajectoryWriter();
  explicit TrajectoryWriter(const std::string& path);
  ~TrajectoryWriter();

  bool open(const std::string& path);
  bool is_open() const;
  const std::string& path() const;
  std::size_t num_written() const;
  void close();
  void write(const LocalizationResult& result);

private:
  std::string path_;
  std::size_t num_written_;
  std::ofstream ofs_;
};

}  // namespace glim_localization
