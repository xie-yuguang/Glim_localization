#define GLIM_ROS2

#include <memory>
#include <sstream>
#include <vector>

#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/odometry/callbacks.hpp>

#include <glim_localization/core/localization_options.hpp>
#include <glim_localization/initialization/runtime_initial_pose.hpp>
#include <glim_localization/map/local_target_map.hpp>
#include <glim_localization/output/localization_result.hpp>
#include <glim_localization/relocalization/runtime_relocalization_request.hpp>

namespace glim_localization {
namespace {

std::string join_ids(const std::vector<int>& ids) {
  std::ostringstream oss;
  for (int i = 0; i < static_cast<int>(ids.size()); i++) {
    if (i) {
      oss << ",";
    }
    oss << ids[i];
  }
  return oss.str();
}

std::string format_double(double value, int precision = 3) {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(precision);
  oss << value;
  return oss.str();
}

diagnostic_msgs::msg::KeyValue make_key_value(const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

Eigen::Isometry3d pose_msg_to_isometry(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << pose.position.x, pose.position.y, pose.position.z;
  T.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).normalized().toRotationMatrix();
  return T;
}

void assign_pose(geometry_msgs::msg::Pose& pose, const Eigen::Isometry3d& T) {
  const Eigen::Vector3d t = T.translation();
  const Eigen::Quaterniond q(T.linear());
  pose.position.x = t.x();
  pose.position.y = t.y();
  pose.position.z = t.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
}

gtsam_points::PointCloudCPU::Ptr build_debug_target_cloud(const LocalTargetMap::ConstPtr& target) {
  if (!target || target->empty()) {
    return nullptr;
  }

  return std::const_pointer_cast<gtsam_points::PointCloudCPU>(target->merged_target_cloud());
}

gtsam_points::PointCloudCPU::Ptr build_aligned_scan_cloud(const glim::EstimationFrame& frame) {
  if (!frame.frame || frame.frame->size() == 0) {
    return nullptr;
  }

  std::vector<Eigen::Vector4d> transformed(frame.frame->size());
  for (int i = 0; i < frame.frame->size(); i++) {
    transformed[i] = frame.T_world_sensor() * frame.frame->points[i];
  }

  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(transformed);
  if (frame.frame->times) {
    cloud->times_storage.assign(frame.frame->times, frame.frame->times + frame.frame->size());
    cloud->times = cloud->times_storage.data();
  }
  if (frame.frame->intensities) {
    cloud->intensities_storage.assign(frame.frame->intensities, frame.frame->intensities + frame.frame->size());
    cloud->intensities = cloud->intensities_storage.data();
  }

  return cloud;
}

gtsam_points::PointCloudCPU::Ptr build_input_scan_cloud(const glim::EstimationFrame& frame) {
  if (!frame.raw_frame || frame.raw_frame->points.empty()) {
    return nullptr;
  }

  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(frame.raw_frame->points);
  if (!frame.raw_frame->times.empty()) {
    cloud->add_times(frame.raw_frame->times);
  }
  if (!frame.raw_frame->intensities.empty()) {
    cloud->add_intensities(frame.raw_frame->intensities);
  }

  return cloud;
}

std_msgs::msg::ColorRGBA status_color(const LocalizationStatus status) {
  std_msgs::msg::ColorRGBA color;
  color.a = 1.0f;

  switch (status) {
    case LocalizationStatus::TRACKING:
      color.r = 0.2f;
      color.g = 0.8f;
      color.b = 0.2f;
      break;
    case LocalizationStatus::DEGRADED:
      color.r = 0.95f;
      color.g = 0.45f;
      color.b = 0.1f;
      break;
    case LocalizationStatus::INITIALIZING:
    case LocalizationStatus::RELOCALIZING:
    case LocalizationStatus::RECOVERING:
      color.r = 0.95f;
      color.g = 0.75f;
      color.b = 0.2f;
      break;
    case LocalizationStatus::WAIT_MAP:
    case LocalizationStatus::WAIT_INITIAL_POSE:
      color.r = 0.3f;
      color.g = 0.7f;
      color.b = 1.0f;
      break;
    case LocalizationStatus::LOST:
      color.r = 1.0f;
      color.g = 0.25f;
      color.b = 0.25f;
      break;
  }

  return color;
}

visualization_msgs::msg::Marker make_active_submaps_marker(const LocalizationOptions& options, const LocalizationResult& result) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = glim::from_sec(result.stamp);
  marker.header.frame_id = options.map_frame;
  marker.ns = "glim_localization";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = result.T_map_imu.translation().x();
  marker.pose.position.y = result.T_map_imu.translation().y();
  marker.pose.position.z = result.T_map_imu.translation().z() + 1.5;
  marker.scale.z = 0.8;
  marker.color = status_color(result.status);
  std::ostringstream oss;
  oss << "status=" << to_string(result.status) << "\nreason=" << result.status_reason << "\nscore=" << format_double(result.matching_score)
      << "\nbackend=" << (result.backend_name.empty() ? "n/a" : result.backend_name)
      << "\nrecover_left=" << result.recovery_frames_remaining << "\nactive_submaps=[" << join_ids(result.target_submap_ids) << "]";
  marker.text = oss.str();
  return marker;
}

diagnostic_msgs::msg::DiagnosticStatus make_diagnostic_status(const LocalizationResult& result) {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "glim_localization/runtime";
  status.hardware_id = "glim_localization";
  status.message = result.status_reason.empty() ? to_string(result.status) : result.status_reason;

  switch (result.status) {
    case LocalizationStatus::TRACKING:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      break;
    case LocalizationStatus::LOST:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      break;
    default:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;
  }

  status.values.push_back(make_key_value("status", to_string(result.status)));
  status.values.push_back(make_key_value("reason", result.status_reason));
  status.values.push_back(make_key_value("matching_score", format_double(result.matching_score)));
  status.values.push_back(make_key_value("backend", result.backend_name));
  status.values.push_back(make_key_value("score_type", result.score_type));
  status.values.push_back(make_key_value("reject_reason", result.reject_reason));
  status.values.push_back(make_key_value("consecutive_rejections", std::to_string(result.consecutive_rejections)));
  status.values.push_back(make_key_value("relocalization_message", result.relocalization_message));
  status.values.push_back(make_key_value("relocalization_attempts", std::to_string(result.relocalization_attempts)));
  status.values.push_back(make_key_value("relocalization_candidate_count", std::to_string(result.relocalization_candidate_count)));
  status.values.push_back(make_key_value("relocalization_verified_rank", std::to_string(result.relocalization_verified_rank)));
  status.values.push_back(make_key_value("recovery_frames_remaining", std::to_string(result.recovery_frames_remaining)));
  status.values.push_back(make_key_value("stable_tracking_successes", std::to_string(result.stable_tracking_successes)));
  status.values.push_back(make_key_value("descriptor_distance", format_double(result.descriptor_distance)));
  status.values.push_back(make_key_value("pose_delta_translation", format_double(result.pose_delta_translation)));
  status.values.push_back(make_key_value("pose_delta_angle", format_double(result.pose_delta_angle)));
  status.values.push_back(make_key_value("continuity_adjusted", result.continuity_adjusted ? "true" : "false"));
  status.values.push_back(make_key_value("continuity_translation", format_double(result.continuity_translation)));
  status.values.push_back(make_key_value("continuity_angle", format_double(result.continuity_angle)));
  status.values.push_back(make_key_value("active_submaps", join_ids(result.target_submap_ids)));
  return status;
}

}  // namespace

class LocalizationPublisher : public glim::ExtensionModuleROS2 {
public:
  LocalizationPublisher() : options_(LocalizationOptions::load()), last_trajectory_stamp_(-1.0), odometry_callback_id_(-1) {
    odometry_callback_id_ =
      glim::OdometryEstimationCallbacks::on_update_new_frame.add([this](const glim::EstimationFrame::ConstPtr& frame) { on_update_frame(frame); });
  }

  ~LocalizationPublisher() override {
    if (odometry_callback_id_ >= 0) {
      glim::OdometryEstimationCallbacks::on_update_new_frame.remove(odometry_callback_id_);
    }
  }

  std::vector<glim::GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override {
    status_pub_ = node.create_publisher<std_msgs::msg::String>(options_.ros.status_topic, 10);
    if (options_.ros.publish_diagnostics) {
      diagnostics_pub_ = node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(options_.ros.diagnostic_topic, 10);
    }
    odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(options_.ros.odom_topic, 10);
    pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(options_.ros.pose_topic, 10);
    trajectory_pub_ = node.create_publisher<nav_msgs::msg::Path>(options_.ros.trajectory_topic, rclcpp::QoS(1).transient_local());
    input_scan_pub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(options_.ros.input_scan_topic, 10);
    current_scan_pub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(options_.ros.current_scan_topic, 10);
    active_submaps_pub_ = node.create_publisher<visualization_msgs::msg::Marker>(options_.ros.active_submaps_topic, rclcpp::QoS(1).transient_local());

    if (options_.ros.publish_debug_target_map) {
      rclcpp::QoS target_qos(1);
      target_qos.transient_local();
      target_map_pub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(options_.ros.target_map_topic, target_qos);
    }

    if (options_.ros.publish_tf) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    }

    relocalization_service_ = node.create_service<std_srvs::srv::Trigger>(
      options_.ros.relocalization_service,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) { on_relocalization_service(request, response); });

    std::vector<glim::GenericTopicSubscription::Ptr> subscriptions;
    subscriptions.emplace_back(new glim::TopicSubscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      options_.ros.initial_pose_topic,
      [this](const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& msg) { on_initial_pose(msg); }));

    spdlog::info("localization publisher extension initialized");
    spdlog::info("  initial_pose_topic={}", options_.ros.initial_pose_topic);
    spdlog::info("  relocalization_service={}", options_.ros.relocalization_service);
    spdlog::info("  status_topic={}", options_.ros.status_topic);
    spdlog::info("  diagnostic_topic={}", options_.ros.diagnostic_topic);
    spdlog::info("  odom_topic={}", options_.ros.odom_topic);
    spdlog::info("  pose_topic={}", options_.ros.pose_topic);
    spdlog::info("  trajectory_topic={}", options_.ros.trajectory_topic);
    spdlog::info("  input_scan_topic={}", options_.ros.input_scan_topic);
    spdlog::info("  current_scan_topic={}", options_.ros.current_scan_topic);
    spdlog::info("  target_map_topic={}", options_.ros.target_map_topic);
    spdlog::info("  active_submaps_topic={}", options_.ros.active_submaps_topic);

    return subscriptions;
  }

private:
  void on_initial_pose(const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& msg) {
    RuntimeInitialPose pose;
    pose.stamp = glim::to_sec(msg->header.stamp);
    pose.T_map_imu = pose_msg_to_isometry(msg->pose.pose);
    set_runtime_initial_pose(pose);

    spdlog::info(
      "received /initialpose T_map_imu=({:.3f}, {:.3f}, {:.3f})",
      pose.T_map_imu.translation().x(),
      pose.T_map_imu.translation().y(),
      pose.T_map_imu.translation().z());
  }

  void on_relocalization_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    (void)request;
    request_runtime_relocalization();
    response->success = true;
    response->message = "relocalization requested";
    spdlog::info("runtime relocalization requested via {}", options_.ros.relocalization_service);
  }

  void on_update_frame(const glim::EstimationFrame::ConstPtr& frame) {
    if (!frame) {
      return;
    }

    const auto result = frame->get_custom_data<LocalizationResult>(kLocalizationResultCustomDataKey);
    if (!result) {
      return;
    }

    publish_status(*result);
    publish_diagnostics(*result);
    publish_pose(*result, *frame);
    publish_trajectory(*result);
    publish_input_scan(*frame);
    publish_current_scan(*frame, *result);
    publish_tf(*result, *frame);
    publish_debug_target_map(frame);
    publish_active_submaps(*result);
  }

  void publish_status(const LocalizationResult& result) {
    if (!status_pub_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = std::string(to_string(result.status)) + " reason=" + result.status_reason + " score=" + format_double(result.matching_score) +
               " reject=" + (result.reject_reason.empty() ? "none" : result.reject_reason);
    status_pub_->publish(msg);
  }

  void publish_diagnostics(const LocalizationResult& result) {
    if (!diagnostics_pub_) {
      return;
    }

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = glim::from_sec(result.stamp);
    msg.status.push_back(make_diagnostic_status(result));
    diagnostics_pub_->publish(msg);
  }

  void publish_pose(const LocalizationResult& result, const glim::EstimationFrame& frame) {
    const auto stamp = glim::from_sec(result.stamp);

    if (odom_pub_) {
      const Eigen::Isometry3d T_odom_imu = result.T_map_odom.inverse() * result.T_map_imu;

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = options_.odom_frame;
      odom.child_frame_id = options_.base_frame;
      assign_pose(odom.pose.pose, T_odom_imu);
      odom.twist.twist.linear.x = frame.v_world_imu.x();
      odom.twist.twist.linear.y = frame.v_world_imu.y();
      odom.twist.twist.linear.z = frame.v_world_imu.z();
      odom_pub_->publish(odom);
    }

    if (pose_pub_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = options_.map_frame;
      assign_pose(pose.pose, result.T_map_imu);
      pose_pub_->publish(pose);
    }
  }

  void publish_trajectory(const LocalizationResult& result) {
    if (!trajectory_pub_) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = glim::from_sec(result.stamp);
    pose.header.frame_id = options_.map_frame;
    assign_pose(pose.pose, result.T_map_imu);

    if (last_trajectory_stamp_ >= 0.0 && result.stamp < last_trajectory_stamp_) {
      trajectory_msg_.poses.clear();
    }
    if (last_trajectory_stamp_ < 0.0 || result.stamp > last_trajectory_stamp_) {
      trajectory_msg_.poses.push_back(pose);
      last_trajectory_stamp_ = result.stamp;
    } else if (!trajectory_msg_.poses.empty()) {
      trajectory_msg_.poses.back() = pose;
    } else {
      trajectory_msg_.poses.push_back(pose);
      last_trajectory_stamp_ = result.stamp;
    }

    trajectory_msg_.header = pose.header;
    trajectory_pub_->publish(trajectory_msg_);
  }

  void publish_current_scan(const glim::EstimationFrame& frame, const LocalizationResult& result) {
    if (!current_scan_pub_ || current_scan_pub_->get_subscription_count() == 0) {
      return;
    }

    const auto cloud = build_aligned_scan_cloud(frame);
    if (!cloud || cloud->size() == 0) {
      return;
    }

    auto msg = glim::frame_to_pointcloud2(options_.map_frame, result.stamp, *cloud);
    current_scan_pub_->publish(*msg);
  }

  void publish_input_scan(const glim::EstimationFrame& frame) {
    if (!input_scan_pub_ || input_scan_pub_->get_subscription_count() == 0) {
      return;
    }

    const auto cloud = build_input_scan_cloud(frame);
    if (!cloud || cloud->size() == 0) {
      return;
    }

    auto msg = glim::frame_to_pointcloud2(options_.sensor_frame, frame.stamp, *cloud);
    input_scan_pub_->publish(*msg);
  }

  void publish_tf(const LocalizationResult& result, const glim::EstimationFrame& frame) {
    if (!tf_broadcaster_) {
      return;
    }

    const auto stamp = glim::from_sec(result.stamp);

    geometry_msgs::msg::TransformStamped map_odom;
    map_odom.header.stamp = stamp;
    map_odom.header.frame_id = options_.map_frame;
    map_odom.child_frame_id = options_.odom_frame;
    const Eigen::Quaterniond q_map_odom(result.T_map_odom.linear());
    map_odom.transform.translation.x = result.T_map_odom.translation().x();
    map_odom.transform.translation.y = result.T_map_odom.translation().y();
    map_odom.transform.translation.z = result.T_map_odom.translation().z();
    map_odom.transform.rotation.x = q_map_odom.x();
    map_odom.transform.rotation.y = q_map_odom.y();
    map_odom.transform.rotation.z = q_map_odom.z();
    map_odom.transform.rotation.w = q_map_odom.w();
    tf_broadcaster_->sendTransform(map_odom);

    geometry_msgs::msg::TransformStamped odom_base;
    odom_base.header.stamp = stamp;
    odom_base.header.frame_id = options_.odom_frame;
    odom_base.child_frame_id = options_.base_frame;

    const Eigen::Isometry3d T_odom_imu = result.T_map_odom.inverse() * result.T_map_imu;
    const Eigen::Quaterniond q_odom_imu(T_odom_imu.linear());
    odom_base.transform.translation.x = T_odom_imu.translation().x();
    odom_base.transform.translation.y = T_odom_imu.translation().y();
    odom_base.transform.translation.z = T_odom_imu.translation().z();
    odom_base.transform.rotation.x = q_odom_imu.x();
    odom_base.transform.rotation.y = q_odom_imu.y();
    odom_base.transform.rotation.z = q_odom_imu.z();
    odom_base.transform.rotation.w = q_odom_imu.w();
    tf_broadcaster_->sendTransform(odom_base);

    (void)frame;
  }

  void publish_debug_target_map(const glim::EstimationFrame::ConstPtr& frame) {
    if (!target_map_pub_ || target_map_pub_->get_subscription_count() == 0) {
      return;
    }

    const auto target = frame->get_custom_data<LocalTargetMap>(kLocalizationTargetMapCustomDataKey);
    if (!target || target->empty()) {
      return;
    }

    const auto cloud = build_debug_target_cloud(std::shared_ptr<const LocalTargetMap>(frame, target));
    if (!cloud || cloud->size() == 0) {
      return;
    }

    auto msg = glim::frame_to_pointcloud2(options_.map_frame, frame->stamp, *cloud);
    target_map_pub_->publish(*msg);
  }

  void publish_active_submaps(const LocalizationResult& result) {
    if (!active_submaps_pub_ || active_submaps_pub_->get_subscription_count() == 0) {
      return;
    }

    active_submaps_pub_->publish(make_active_submaps_marker(options_, result));
  }

private:
  LocalizationOptions options_;
  nav_msgs::msg::Path trajectory_msg_;
  double last_trajectory_stamp_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr active_submaps_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalization_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  int odometry_callback_id_;
};

}  // namespace glim_localization

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim_localization::LocalizationPublisher();
}
