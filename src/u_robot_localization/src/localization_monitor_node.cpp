#include "u_robot_localization/localization_health.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace u_robot_localization
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

double quaternion_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 *
    (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & position = pose.position;
  const auto & orientation = pose.orientation;
  const double quaternion_norm =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
    std::isfinite(position.z) && std::isfinite(orientation.x) &&
    std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
    std::isfinite(orientation.w) && quaternion_norm > 1.0e-6;
}
}  // namespace

class LocalizationMonitorNode final : public rclcpp::Node
{
public:
  LocalizationMonitorNode()
  : Node("localization_monitor"),
    map_frame_(declare_parameter<std::string>("map_frame", "map")),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    pose_topic_(declare_parameter<std::string>("pose_topic", "/amcl_pose")),
    scan_topic_(declare_parameter<std::string>("scan_topic", "/scan")),
    map_topic_(declare_parameter<std::string>("map_topic", "/map")),
    scan_timeout_(milliseconds_parameter("scan_timeout_ms", 500)),
    position_stddev_warn_(positive_parameter("position_stddev_warn", 0.50)),
    yaw_stddev_warn_(positive_parameter("yaw_stddev_warn", 0.50)),
    pose_jump_warn_(positive_parameter("pose_jump_warn", 0.75)),
    yaw_jump_warn_(positive_parameter("yaw_jump_warn", 0.80)),
    jump_warning_duration_(milliseconds_parameter("jump_warning_duration_ms", 3000)),
    alignment_topic_(declare_parameter<std::string>(
        "alignment_topic", "/localization/scan_alignment")),
    quality_topic_(declare_parameter<std::string>(
        "quality_topic", "/localization/quality_ok")),
    occupied_threshold_(declare_parameter<std::int64_t>("occupied_threshold", 50)),
    scan_match_radius_(positive_parameter("scan_match_radius", 0.15)),
    scan_match_min_range_(positive_parameter("scan_match_min_range", 0.55)),
    scan_match_max_range_(positive_parameter("scan_match_max_range", 15.0)),
    scan_match_max_beams_(declare_parameter<std::int64_t>("scan_match_max_beams", 160)),
    scan_match_min_points_(declare_parameter<std::int64_t>("scan_match_min_points", 40)),
    scan_alignment_warn_(fraction_parameter("scan_alignment_warn", 0.60)),
    scan_alignment_recover_(fraction_parameter("scan_alignment_recover", 0.66)),
    scan_alignment_ema_alpha_(fraction_parameter("scan_alignment_ema_alpha", 0.25)),
    scan_alignment_bad_samples_(
      declare_parameter<std::int64_t>("scan_alignment_bad_samples", 3)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    if (occupied_threshold_ < 1 || occupied_threshold_ > 100) {
      throw std::invalid_argument("occupied_threshold must be in [1, 100]");
    }
    if (scan_match_max_range_ <= scan_match_min_range_) {
      throw std::invalid_argument("scan_match_max_range must exceed scan_match_min_range");
    }
    if (scan_match_max_beams_ < 1 || scan_match_min_points_ < 1 ||
      scan_alignment_bad_samples_ < 1)
    {
      throw std::invalid_argument("scan matching count parameters must be positive");
    }
    if (scan_alignment_recover_ <= scan_alignment_warn_) {
      throw std::invalid_argument("scan_alignment_recover must exceed scan_alignment_warn");
    }

    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    alignment_publisher_ = create_publisher<std_msgs::msg::Float32>(
      alignment_topic_, latched_qos);
    quality_publisher_ = create_publisher<std_msgs::msg::Bool>(quality_topic_, latched_qos);
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        on_pose(*message);
      });
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
        scan_seen_ = true;
        last_scan_time_ = std::chrono::steady_clock::now();
        last_scan_ = *message;
      });
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
        map_seen_ = true;
        map_valid_ = message->info.resolution > 0.0F && message->info.width > 0U &&
          message->info.height > 0U &&
          message->data.size() ==
          static_cast<std::size_t>(message->info.width) * message->info.height;
        map_width_ = message->info.width;
        map_height_ = message->info.height;
        map_resolution_ = message->info.resolution;
        map_ = *message;
      });
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostics();});
  }

private:
  std::chrono::milliseconds milliseconds_parameter(
    const std::string & name, const std::int64_t default_value)
  {
    const std::int64_t value = declare_parameter<std::int64_t>(name, default_value);
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return std::chrono::milliseconds(value);
  }

  double positive_parameter(const std::string & name, const double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  double fraction_parameter(const std::string & name, const double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
      throw std::invalid_argument(name + " must be in (0, 1]");
    }
    return value;
  }

  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & message)
  {
    pose_seen_ = true;
    last_pose_time_ = std::chrono::steady_clock::now();
    pose_valid_ = finite_pose(message.pose.pose);
    covariance_ = covariance_metrics(message.pose.covariance);
    pose_valid_ = pose_valid_ && covariance_.valid;
    if (!pose_valid_) {
      invalid_pose_count_++;
      return;
    }

    const double x = message.pose.pose.position.x;
    const double y = message.pose.pose.position.y;
    const double yaw = quaternion_yaw(message.pose.pose.orientation);
    if (last_valid_pose_.has_value()) {
      const auto & previous = *last_valid_pose_;
      if (planar_distance(previous.x, previous.y, x, y) > pose_jump_warn_ ||
        angular_distance(previous.yaw, yaw) > yaw_jump_warn_)
      {
        jump_count_++;
        last_jump_time_ = last_pose_time_;
      }
    }
    last_valid_pose_ = PlanarPose{x, y, yaw};
  }

  void publish_diagnostics()
  {
    update_scan_alignment();
    const auto steady_now = std::chrono::steady_clock::now();
    // AMCL publishes a durable pose when it performs an update, not at a fixed
    // heartbeat while the robot is stationary. Current TF and scan freshness
    // are the liveness checks; the pose topic only needs to have been received.
    const bool pose_missing = !pose_seen_;
    const bool scan_stale = !scan_seen_ || steady_now - last_scan_time_ > scan_timeout_;
    const bool map_frame_seen = tf_buffer_._frameExists(map_frame_);
    const bool base_frame_seen = tf_buffer_._frameExists(base_frame_);
    const bool transform_available = map_frame_seen && base_frame_seen &&
      tf_buffer_.canTransform(
        map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.0));
    const bool high_uncertainty = pose_seen_ && pose_valid_ &&
      (covariance_.position_stddev > position_stddev_warn_ ||
      covariance_.yaw_stddev > yaw_stddev_warn_);
    const bool recent_jump = last_jump_time_.time_since_epoch().count() != 0 &&
      steady_now - last_jump_time_ < jump_warning_duration_;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "u_robot/localization";
    status.hardware_id = "unitree_a2";
    if ((pose_seen_ && !pose_valid_) || (map_seen_ && !map_valid_)) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "invalid localization input or estimate";
    } else if (!map_seen_ || pose_missing || scan_stale || !transform_available) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for complete localization data path";
    } else if (alignment_ready_ && !localization_quality_ok_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "laser scan no longer agrees with the static map";
    } else if (high_uncertainty || recent_jump) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = high_uncertainty ?
        "localization uncertainty above commissioning threshold" :
        "recent AMCL pose jump";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "static-map localization healthy";
    }

    status.values.push_back(key_value("map_received", map_seen_ ? "true" : "false"));
    status.values.push_back(key_value("map_valid", map_valid_ ? "true" : "false"));
    status.values.push_back(key_value("map_width", std::to_string(map_width_)));
    status.values.push_back(key_value("map_height", std::to_string(map_height_)));
    status.values.push_back(key_value("map_resolution", std::to_string(map_resolution_)));
    status.values.push_back(key_value("scan_stale", scan_stale ? "true" : "false"));
    status.values.push_back(key_value("pose_missing", pose_missing ? "true" : "false"));
    status.values.push_back(key_value("pose_valid", pose_valid_ ? "true" : "false"));
    status.values.push_back(
      key_value("map_to_base_tf", transform_available ? "true" : "false"));
    status.values.push_back(
      key_value("position_stddev", std::to_string(covariance_.position_stddev)));
    status.values.push_back(
      key_value("yaw_stddev", std::to_string(covariance_.yaw_stddev)));
    status.values.push_back(
      key_value("scan_alignment_ready", alignment_ready_ ? "true" : "false"));
    status.values.push_back(
      key_value("scan_alignment_raw", std::to_string(scan_alignment_raw_)));
    status.values.push_back(
      key_value("scan_alignment_ema", std::to_string(scan_alignment_ema_)));
    status.values.push_back(
      key_value("scan_alignment_points", std::to_string(scan_alignment_points_)));
    status.values.push_back(
      key_value("localization_quality_ok", localization_quality_ok_ ? "true" : "false"));
    status.values.push_back(key_value("pose_jump_count", std::to_string(jump_count_)));
    status.values.push_back(
      key_value("invalid_pose_count", std::to_string(invalid_pose_count_)));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);

    std_msgs::msg::Float32 alignment;
    alignment.data = static_cast<float>(scan_alignment_ema_);
    alignment_publisher_->publish(alignment);
    std_msgs::msg::Bool quality;
    quality.data = alignment_ready_ && localization_quality_ok_;
    quality_publisher_->publish(quality);
  }

  bool map_cell_has_nearby_obstacle(const int center_x, const int center_y) const
  {
    const int radius = static_cast<int>(std::ceil(scan_match_radius_ / map_resolution_));
    for (int offset_y = -radius; offset_y <= radius; ++offset_y) {
      for (int offset_x = -radius; offset_x <= radius; ++offset_x) {
        if (std::hypot(
            static_cast<double>(offset_x) * map_resolution_,
            static_cast<double>(offset_y) * map_resolution_) > scan_match_radius_)
        {
          continue;
        }
        const int cell_x = center_x + offset_x;
        const int cell_y = center_y + offset_y;
        if (cell_x < 0 || cell_y < 0 ||
          cell_x >= static_cast<int>(map_width_) ||
          cell_y >= static_cast<int>(map_height_))
        {
          continue;
        }
        const std::size_t index = static_cast<std::size_t>(cell_y) * map_width_ +
          static_cast<std::size_t>(cell_x);
        if (map_.data[index] >= occupied_threshold_) {
          return true;
        }
      }
    }
    return false;
  }

  void update_scan_alignment()
  {
    if (!map_valid_ || !last_scan_.has_value()) {
      return;
    }
    const auto & scan = *last_scan_;
    const std::int64_t stamp_nanoseconds = rclcpp::Time(scan.header.stamp).nanoseconds();
    if (stamp_nanoseconds == last_alignment_stamp_nanoseconds_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, scan.header.frame_id, rclcpp::Time(scan.header.stamp),
        rclcpp::Duration::from_seconds(0.02));
    } catch (const tf2::TransformException &) {
      return;
    }
    last_alignment_stamp_nanoseconds_ = stamp_nanoseconds;

    const double sensor_yaw = quaternion_yaw(transform.transform.rotation);
    const double sensor_cos = std::cos(sensor_yaw);
    const double sensor_sin = std::sin(sensor_yaw);
    const double origin_yaw = quaternion_yaw(map_.info.origin.orientation);
    const double origin_cos = std::cos(origin_yaw);
    const double origin_sin = std::sin(origin_yaw);
    const std::size_t stride = std::max<std::size_t>(
      1U, (scan.ranges.size() + static_cast<std::size_t>(scan_match_max_beams_) - 1U) /
      static_cast<std::size_t>(scan_match_max_beams_));

    int valid_points = 0;
    int matching_points = 0;
    for (std::size_t index = 0; index < scan.ranges.size(); index += stride) {
      const double range = static_cast<double>(scan.ranges[index]);
      const double minimum_range = std::max(
        scan_match_min_range_, static_cast<double>(scan.range_min));
      const double maximum_range = std::min(
        scan_match_max_range_, static_cast<double>(scan.range_max));
      if (!std::isfinite(range) || range < minimum_range || range > maximum_range) {
        continue;
      }
      const double angle = static_cast<double>(scan.angle_min) +
        static_cast<double>(index) * static_cast<double>(scan.angle_increment);
      const double scan_x = range * std::cos(angle);
      const double scan_y = range * std::sin(angle);
      const double map_x = transform.transform.translation.x +
        sensor_cos * scan_x - sensor_sin * scan_y;
      const double map_y = transform.transform.translation.y +
        sensor_sin * scan_x + sensor_cos * scan_y;
      const double delta_x = map_x - map_.info.origin.position.x;
      const double delta_y = map_y - map_.info.origin.position.y;
      const double grid_x = origin_cos * delta_x + origin_sin * delta_y;
      const double grid_y = -origin_sin * delta_x + origin_cos * delta_y;
      const int cell_x = static_cast<int>(std::floor(grid_x / map_resolution_));
      const int cell_y = static_cast<int>(std::floor(grid_y / map_resolution_));
      if (cell_x < 0 || cell_y < 0 ||
        cell_x >= static_cast<int>(map_width_) ||
        cell_y >= static_cast<int>(map_height_))
      {
        continue;
      }
      const std::size_t map_index = static_cast<std::size_t>(cell_y) * map_width_ +
        static_cast<std::size_t>(cell_x);
      if (map_.data[map_index] < 0) {
        continue;
      }
      ++valid_points;
      if (map_cell_has_nearby_obstacle(cell_x, cell_y)) {
        ++matching_points;
      }
    }
    scan_alignment_points_ = valid_points;
    if (valid_points < scan_match_min_points_) {
      return;
    }

    scan_alignment_raw_ = static_cast<double>(matching_points) /
      static_cast<double>(valid_points);
    if (!alignment_ready_) {
      scan_alignment_ema_ = scan_alignment_raw_;
      alignment_ready_ = true;
      localization_quality_ok_ = scan_alignment_ema_ >= scan_alignment_recover_;
    } else {
      scan_alignment_ema_ = scan_alignment_ema_alpha_ * scan_alignment_raw_ +
        (1.0 - scan_alignment_ema_alpha_) * scan_alignment_ema_;
    }

    if (localization_quality_ok_) {
      if (scan_alignment_ema_ < scan_alignment_warn_) {
        ++bad_alignment_samples_;
        if (bad_alignment_samples_ >= scan_alignment_bad_samples_) {
          localization_quality_ok_ = false;
          good_alignment_samples_ = 0;
        }
      } else {
        bad_alignment_samples_ = 0;
      }
    } else if (scan_alignment_ema_ >= scan_alignment_recover_) {
      ++good_alignment_samples_;
      if (good_alignment_samples_ >= 2) {
        localization_quality_ok_ = true;
        bad_alignment_samples_ = 0;
      }
    } else {
      good_alignment_samples_ = 0;
    }
  }

  struct PlanarPose
  {
    double x;
    double y;
    double yaw;
  };

  const std::string map_frame_;
  const std::string base_frame_;
  const std::string pose_topic_;
  const std::string scan_topic_;
  const std::string map_topic_;
  const std::chrono::milliseconds scan_timeout_;
  const double position_stddev_warn_;
  const double yaw_stddev_warn_;
  const double pose_jump_warn_;
  const double yaw_jump_warn_;
  const std::chrono::milliseconds jump_warning_duration_;
  const std::string alignment_topic_;
  const std::string quality_topic_;
  const std::int64_t occupied_threshold_;
  const double scan_match_radius_;
  const double scan_match_min_range_;
  const double scan_match_max_range_;
  const std::int64_t scan_match_max_beams_;
  const std::int64_t scan_match_min_points_;
  const double scan_alignment_warn_;
  const double scan_alignment_recover_;
  const double scan_alignment_ema_alpha_;
  const std::int64_t scan_alignment_bad_samples_;

  bool map_seen_{false};
  bool map_valid_{false};
  bool scan_seen_{false};
  bool pose_seen_{false};
  bool pose_valid_{false};
  std::uint32_t map_width_{0};
  std::uint32_t map_height_{0};
  float map_resolution_{0.0F};
  std::uint64_t jump_count_{0};
  std::uint64_t invalid_pose_count_{0};
  CovarianceMetrics covariance_{false, 0.0, 0.0};
  nav_msgs::msg::OccupancyGrid map_;
  std::optional<sensor_msgs::msg::LaserScan> last_scan_;
  std::optional<PlanarPose> last_valid_pose_;
  std::chrono::steady_clock::time_point last_scan_time_{};
  std::chrono::steady_clock::time_point last_pose_time_{};
  std::chrono::steady_clock::time_point last_jump_time_{};
  std::int64_t last_alignment_stamp_nanoseconds_{-1};
  int scan_alignment_points_{0};
  std::int64_t bad_alignment_samples_{0};
  std::int64_t good_alignment_samples_{0};
  double scan_alignment_raw_{0.0};
  double scan_alignment_ema_{0.0};
  bool alignment_ready_{false};
  bool localization_quality_ok_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr alignment_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr quality_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace u_robot_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<u_robot_localization::LocalizationMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
