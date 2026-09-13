#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace u_robot_localization
{
class InitialPoseAdapterNode final : public rclcpp::Node
{
public:
  InitialPoseAdapterNode()
  : Node("initial_pose_adapter"),
    input_topic_(declare_parameter<std::string>("input_topic", "/operator/initialpose")),
    output_topic_(declare_parameter<std::string>("output_topic", "/initialpose")),
    map_frame_(declare_parameter<std::string>("map_frame", "map")),
    nomotion_service_(
      declare_parameter<std::string>("nomotion_service", "/request_nomotion_update")),
    position_stddev_(positive_parameter("position_stddev", 0.50)),
    yaw_stddev_(positive_parameter("yaw_stddev", 0.35))
  {
    if (input_topic_.empty() || output_topic_.empty() || input_topic_ == output_topic_) {
      throw std::invalid_argument("initial-pose input/output topics must be non-empty and distinct");
    }

    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    nomotion_client_ = create_client<std_srvs::srv::Empty>(nomotion_service_);
    subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        on_initial_pose(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Initial-pose adapter ready: %s -> %s (sigma_xy=%.2f m, sigma_yaw=%.1f deg)",
      input_topic_.c_str(), output_topic_.c_str(), position_stddev_,
      yaw_stddev_ * 180.0 / 3.14159265358979323846);
  }

private:
  double positive_parameter(const std::string & name, const double fallback)
  {
    const double value = declare_parameter<double>(name, fallback);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  void on_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & input)
  {
    const auto & p = input.pose.pose.position;
    const auto & q = input.pose.pose.orientation;
    const double quaternion_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(quaternion_norm) || quaternion_norm < 0.90 || quaternion_norm > 1.10)
    {
      RCLCPP_ERROR(get_logger(), "Rejected invalid operator initial pose");
      return;
    }

    auto output = input;
    output.header.frame_id = map_frame_;
    output.header.stamp = now();

    // Foxglove may publish zero covariance. That tells AMCL the mouse click is
    // millimetre-accurate and collapses the particle cloud around a merely
    // approximate pose. Preserve larger user covariance, but enforce a useful
    // search region around every operator click.
    auto & covariance = output.pose.covariance;
    const double position_variance = position_stddev_ * position_stddev_;
    const double yaw_variance = yaw_stddev_ * yaw_stddev_;
    covariance[0] = std::max(covariance[0], position_variance);
    covariance[7] = std::max(covariance[7], position_variance);
    covariance[35] = std::max(covariance[35], yaw_variance);

    publisher_->publish(output);
    RCLCPP_WARN(
      get_logger(),
      "Forwarded operator initial pose with sigma_xy=%.2f m and sigma_yaw=%.1f deg; "
      "wait for the particle cloud to converge before navigation",
      std::sqrt(covariance[0]), std::sqrt(covariance[35]) * 180.0 /
      3.14159265358979323846);

    // Let AMCL consume the new pose first, then force one laser update even if
    // odometry is perfectly stationary.
    nomotion_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
        nomotion_timer_->cancel();
        if (!nomotion_client_->service_is_ready()) {
          RCLCPP_WARN(get_logger(), "%s is not ready", nomotion_service_.c_str());
          return;
        }
        nomotion_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
        RCLCPP_INFO(get_logger(), "Requested a stationary AMCL laser update");
      });
  }

  const std::string input_topic_;
  const std::string output_topic_;
  const std::string map_frame_;
  const std::string nomotion_service_;
  const double position_stddev_;
  const double yaw_stddev_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr nomotion_client_;
  rclcpp::TimerBase::SharedPtr nomotion_timer_;
};
}  // namespace u_robot_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<u_robot_localization::InitialPoseAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
