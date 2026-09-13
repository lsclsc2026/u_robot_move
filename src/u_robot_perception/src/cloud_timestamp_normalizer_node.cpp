#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

class CloudTimestampNormalizer final : public rclcpp::Node
{
public:
  CloudTimestampNormalizer()
  : Node("mapping_cloud_timestamp_normalizer"),
    input_topic_(declare_parameter<std::string>(
      "input_topic", "/unitree/slam_lidar/points")),
    output_topic_(declare_parameter<std::string>(
      "output_topic", "/mapping/points_time_aligned")),
    minimum_age_(declare_parameter<double>("minimum_age_to_restamp", 0.20))
  {
    if (input_topic_.empty() || output_topic_.empty() || minimum_age_ < 0.0) {
      throw std::invalid_argument("invalid cloud timestamp normalizer parameters");
    }
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {
        auto output = *input;
        const auto receive_time = now();
        const auto source_time = rclcpp::Time(input->header.stamp, get_clock()->get_clock_type());
        const double age = (receive_time - source_time).seconds();
        if (std::isfinite(age) && age > minimum_age_) {
          output.header.stamp = receive_time;
          ++restamped_count_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Correcting stale fused-cloud timestamp (age %.3f s, corrected %zu clouds)",
            age, restamped_count_);
        }
        publisher_->publish(output);
      });
  }

private:
  const std::string input_topic_;
  const std::string output_topic_;
  const double minimum_age_;
  std::size_t restamped_count_{0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudTimestampNormalizer>());
  rclcpp::shutdown();
  return 0;
}
