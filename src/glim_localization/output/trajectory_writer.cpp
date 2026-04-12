#include <glim_localization/output/trajectory_writer.hpp>

#include <filesystem>
#include <iomanip>

namespace glim_localization {

TrajectoryWriter::TrajectoryWriter() : num_written_(0) {}

TrajectoryWriter::TrajectoryWriter(const std::string& path) {
  num_written_ = 0;
  open(path);
}

TrajectoryWriter::~TrajectoryWriter() {
  close();
}

bool TrajectoryWriter::open(const std::string& path) {
  close();
  path_ = path;
  num_written_ = 0;

  const std::filesystem::path output_path(path);
  const auto parent = output_path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }

  ofs_.open(path);
  return ofs_.is_open();
}

bool TrajectoryWriter::is_open() const {
  return ofs_.is_open();
}

const std::string& TrajectoryWriter::path() const {
  return path_;
}

std::size_t TrajectoryWriter::num_written() const {
  return num_written_;
}

void TrajectoryWriter::close() {
  if (ofs_.is_open()) {
    ofs_.flush();
    ofs_.close();
  }
}

void TrajectoryWriter::write(const LocalizationResult& result) {
  if (!ofs_.is_open()) {
    return;
  }

  const Eigen::Vector3d t = result.T_map_imu.translation();
  const Eigen::Quaterniond q(result.T_map_imu.linear());

  ofs_ << std::fixed << std::setprecision(9) << result.stamp << " "  //
       << t.x() << " " << t.y() << " " << t.z() << " "              //
       << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
       << static_cast<int>(result.status) << " " << result.matching_score << "\n";
  num_written_++;
}

}  // namespace glim_localization
