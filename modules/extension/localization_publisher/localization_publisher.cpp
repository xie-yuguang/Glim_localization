#define GLIM_ROS2

#include <memory>
#include <vector>

#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

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

}  // namespace

class LocalizationPublisher : public glim::ExtensionModuleROS2 {
public:
  LocalizationPublisher() : options_(LocalizationOptions::load()), odometry_callback_id_(-1) {
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
    odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(options_.ros.odom_topic, 10);
    pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(options_.ros.pose_topic, 10);

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
    spdlog::info("  odom_topic={}", options_.ros.odom_topic);
    spdlog::info("  pose_topic={}", options_.ros.pose_topic);
    spdlog::info("  target_map_topic={}", options_.ros.target_map_topic);

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
    publish_pose(*result, *frame);
    publish_tf(*result, *frame);
    publish_debug_target_map(frame);
  }

  void publish_status(const LocalizationResult& result) {
    if (!status_pub_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = std::string(to_string(result.status)) + " score=" + std::to_string(result.matching_score);
    status_pub_->publish(msg);
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

    // P2 keeps T_map_odom as identity, so T_odom_base == T_map_imu.  A later
    // relocalization/reset stage can maintain a non-identity T_map_odom.
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

private:
  LocalizationOptions options_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalization_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  int odometry_callback_id_;
};

}  // namespace glim_localization

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim_localization::LocalizationPublisher();
}
