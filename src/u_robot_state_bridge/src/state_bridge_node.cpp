#include "u_robot_state_bridge/odometry_utils.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace u_robot_state_bridge
{
namespace
{
diagnostic_msgs::msg::KeyValue key_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}
}  // namespace

class StateBridgeNode final : public rclcpp::Node
{
public:
  StateBridgeNode()
  : Node("state_bridge"),
    source_topic_(declare_parameter<std::string>("source_topic", "/dog_odom")),
    output_topic_(declare_parameter<std::string>("output_topic", "/odometry/a2")),
    expected_source_child_frame_(
      declare_parameter<std::string>("expected_source_child_frame", "robot_center")),
    odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    publish_tf_(declare_parameter<bool>("publish_tf", true)),
    replace_zero_covariance_(declare_parameter<bool>("replace_zero_covariance", true)),
    output_period_(1.0 / declare_positive_parameter("output_rate_hz", 100.0)),
    source_timeout_(
      std::chrono::milliseconds(declare_parameter<int>("source_timeout_ms", 100))),
    pose_covariance_diagonal_(declare_parameter<std::vector<double>>(
      "pose_covariance_diagonal", {0.01, 0.01, 0.04, 0.0025, 0.0025, 0.01})),
    twist_covariance_diagonal_(declare_parameter<std::vector<double>>(
      "twist_covariance_diagonal", {0.04, 0.04, 0.09, 0.01, 0.01, 0.04}))
  {
    if (source_timeout_.count() <= 0) {
      throw std::invalid_argument("source_timeout_ms must be positive");
    }

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    if (publish_tf_) {
      transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    source_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      source_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {on_odometry(*message);});
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostics();});

    RCLCPP_INFO(
      get_logger(), "Normalizing %s (%s) to %s (%s -> %s) at %.1f Hz",
      source_topic_.c_str(), expected_source_child_frame_.c_str(), output_topic_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str(), 1.0 / output_period_.count());
  }

private:
  double declare_positive_parameter(const std::string & name, const double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (value <= 0.0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return value;
  }

  void on_odometry(const nav_msgs::msg::Odometry & source)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    source_seen_ = true;
    last_source_time_ = steady_now;

    if (!expected_source_child_frame_.empty() &&
      source.child_frame_id != expected_source_child_frame_)
    {
      frame_mismatch_count_++;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Unexpected source child frame '%s' (expected '%s')",
        source.child_frame_id.c_str(), expected_source_child_frame_.c_str());
      return;
    }

    if (last_output_time_.time_since_epoch().count() != 0 &&
      steady_now - last_output_time_ < output_period_)
    {
      return;
    }
    last_output_time_ = steady_now;

    nav_msgs::msg::Odometry output = source;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;

    if (replace_zero_covariance_ && covariance_is_zero(output.pose.covariance)) {
      set_covariance_diagonal(output.pose.covariance, pose_covariance_diagonal_);
      pose_covariance_replacements_++;
    }
    if (replace_zero_covariance_ && covariance_is_zero(output.twist.covariance)) {
      set_covariance_diagonal(output.twist.covariance, twist_covariance_diagonal_);
      twist_covariance_replacements_++;
    }

    odometry_publisher_->publish(output);
    output_count_++;

    if (transform_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = output.child_frame_id;
      transform.transform.translation.x = output.pose.pose.position.x;
      transform.transform.translation.y = output.pose.pose.position.y;
      transform.transform.translation.z = output.pose.pose.position.z;
      transform.transform.rotation = output.pose.pose.orientation;
      transform_broadcaster_->sendTransform(transform);
    }
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "u_robot/state_bridge";
    status.hardware_id = "unitree_a2";

    const bool stale = !source_seen_ ||
      std::chrono::steady_clock::now() - last_source_time_ > source_timeout_;
    status.level = stale ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = stale ? "source odometry stale" : "source odometry healthy";
    status.values.push_back(key_value("source_topic", source_topic_));
    status.values.push_back(key_value("output_topic", output_topic_));
    status.values.push_back(key_value("output_count", std::to_string(output_count_)));
    status.values.push_back(
      key_value("frame_mismatches", std::to_string(frame_mismatch_count_)));
    status.values.push_back(key_value(
      "pose_covariance_replacements", std::to_string(pose_covariance_replacements_)));
    status.values.push_back(key_value(
      "twist_covariance_replacements", std::to_string(twist_covariance_replacements_)));
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  const std::string source_topic_;
  const std::string output_topic_;
  const std::string expected_source_child_frame_;
  const std::string odom_frame_;
  const std::string base_frame_;
  const bool publish_tf_;
  const bool replace_zero_covariance_;
  const std::chrono::duration<double> output_period_;
  const std::chrono::milliseconds source_timeout_;
  const std::vector<double> pose_covariance_diagonal_;
  const std::vector<double> twist_covariance_diagonal_;

  bool source_seen_{false};
  std::chrono::steady_clock::time_point last_source_time_{};
  std::chrono::steady_clock::time_point last_output_time_{};
  std::uint64_t output_count_{0};
  std::uint64_t frame_mismatch_count_{0};
  std::uint64_t pose_covariance_replacements_{0};
  std::uint64_t twist_covariance_replacements_{0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr source_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
};

}  // namespace u_robot_state_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<u_robot_state_bridge::StateBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
