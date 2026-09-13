#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

class VelocityDeadbandAdapter final : public rclcpp::Node
{
public:
  VelocityDeadbandAdapter()
  : Node("velocity_deadband_adapter"),
    input_topic_(declare_parameter<std::string>("input_topic", "/cmd_vel")),
    output_topic_(declare_parameter<std::string>("output_topic", "/cmd_vel_shaped")),
    activation_epsilon_(declare_parameter<double>("activation_epsilon", 0.005)),
    minimum_vx_(declare_parameter<double>("minimum_vx", 0.08)),
    minimum_vy_(declare_parameter<double>("minimum_vy", 0.08)),
    minimum_wz_(declare_parameter<double>("minimum_wz", 0.12)),
    minimum_wz_while_translating_(
      declare_parameter<double>("minimum_wz_while_translating", 0.0)),
    maximum_wz_(declare_parameter<double>("maximum_wz", 0.45)),
    forward_yaw_gain_(declare_parameter<double>("forward_yaw_gain", 1.0)),
    reverse_yaw_gain_(declare_parameter<double>("reverse_yaw_gain", 1.0))
  {
    if (input_topic_.empty() || output_topic_.empty() || activation_epsilon_ < 0.0 ||
      minimum_vx_ <= activation_epsilon_ || minimum_vy_ <= activation_epsilon_ ||
      minimum_wz_ <= activation_epsilon_ ||
      minimum_wz_while_translating_ < 0.0 ||
      minimum_wz_while_translating_ > minimum_wz_ || maximum_wz_ < minimum_wz_ ||
      forward_yaw_gain_ <= 0.0 || reverse_yaw_gain_ <= 0.0)
    {
      throw std::invalid_argument("invalid velocity deadband adapter parameters");
    }
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr input) {
        auto output = *input;
        const bool translating =
          std::abs(input->linear.x) > activation_epsilon_ ||
          std::abs(input->linear.y) > activation_epsilon_;
        if (translating) {
          // Preserve MPPI curvature through the A2 gait deadband. Raising vx
          // alone turns (0.02, 0.01) into (0.08, 0.01), quadrupling the turn
          // radius and sweeping the rear body into door corners. Scaling the
          // complete planar twist retains vx/wz and vy/wz ratios.
          double scale = 1.0;
          if (std::abs(input->linear.x) > activation_epsilon_) {
            scale = std::max(scale, minimum_vx_ / std::abs(input->linear.x));
          }
          if (std::abs(input->linear.y) > activation_epsilon_) {
            scale = std::max(scale, minimum_vy_ / std::abs(input->linear.y));
          }
          if (std::abs(input->angular.z) > activation_epsilon_) {
            scale = std::min(scale, maximum_wz_ / std::abs(input->angular.z));
          }
          output.linear.x = input->linear.x * scale;
          output.linear.y = input->linear.y * scale;
          output.angular.z = input->angular.z * scale;
          // The A2 does not reproduce forward and yaw commands with equal
          // gain.  A direction-dependent correction keeps the curvature seen
          // by MPPI instead of letting the physical gait cut inside turns.
          if (input->linear.x > activation_epsilon_) {
            output.angular.z *= forward_yaw_gain_;
          } else if (input->linear.x < -activation_epsilon_) {
            output.angular.z *= reverse_yaw_gain_;
          }
          output.angular.z = std::clamp(output.angular.z, -maximum_wz_, maximum_wz_);
          if (minimum_wz_while_translating_ > activation_epsilon_) {
            output.angular.z = lift(output.angular.z, minimum_wz_while_translating_);
          }
        } else {
          output.angular.z = lift(output.angular.z, minimum_wz_);
        }
        if (output.linear.x != input->linear.x || output.linear.y != input->linear.y ||
          output.angular.z != input->angular.z)
        {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Lifted A2 deadband command: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)",
            input->linear.x, input->linear.y, input->angular.z,
            output.linear.x, output.linear.y, output.angular.z);
        }
        publisher_->publish(output);
      });
  }

private:
  double lift(const double value, const double minimum) const
  {
    const double magnitude = std::abs(value);
    if (!std::isfinite(value) || magnitude <= activation_epsilon_ || magnitude >= minimum) {
      return value;
    }
    return std::copysign(minimum, value);
  }

  const std::string input_topic_;
  const std::string output_topic_;
  const double activation_epsilon_;
  const double minimum_vx_;
  const double minimum_vy_;
  const double minimum_wz_;
  const double minimum_wz_while_translating_;
  const double maximum_wz_;
  const double forward_yaw_gain_;
  const double reverse_yaw_gain_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityDeadbandAdapter>());
  rclcpp::shutdown();
  return 0;
}
