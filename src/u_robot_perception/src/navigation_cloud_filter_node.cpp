#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace u_robot_perception
{
namespace
{
struct Point
{
  float x;
  float y;
  float z;
};

struct Capsule
{
  Point start;
  Point end;
  double radius_squared;
};

struct VoxelKey
{
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto x = static_cast<std::uint32_t>(key.x);
    const auto y = static_cast<std::uint32_t>(key.y);
    const auto z = static_cast<std::uint32_t>(key.z);
    std::size_t seed = static_cast<std::size_t>(x) * 73856093U;
    seed ^= static_cast<std::size_t>(y) * 19349663U;
    seed ^= static_cast<std::size_t>(z) * 83492791U;
    return seed;
  }
};

diagnostic_msgs::msg::KeyValue key_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

bool inside_sphere(
  const double x, const double y, const double z, const Point & center,
  const double radius_squared)
{
  const double dx = x - static_cast<double>(center.x);
  const double dy = y - static_cast<double>(center.y);
  const double dz = z - static_cast<double>(center.z);
  return dx * dx + dy * dy + dz * dz <= radius_squared;
}

bool inside_capsule(const double x, const double y, const double z, const Capsule & capsule)
{
  const double segment_x = static_cast<double>(capsule.end.x - capsule.start.x);
  const double segment_y = static_cast<double>(capsule.end.y - capsule.start.y);
  const double segment_z = static_cast<double>(capsule.end.z - capsule.start.z);
  const double point_x = x - static_cast<double>(capsule.start.x);
  const double point_y = y - static_cast<double>(capsule.start.y);
  const double point_z = z - static_cast<double>(capsule.start.z);
  const double segment_length_squared =
    segment_x * segment_x + segment_y * segment_y + segment_z * segment_z;
  const double projection = segment_length_squared > std::numeric_limits<double>::epsilon() ?
    std::clamp(
    (point_x * segment_x + point_y * segment_y + point_z * segment_z) /
    segment_length_squared, 0.0, 1.0) : 0.0;
  const double dx = point_x - projection * segment_x;
  const double dy = point_y - projection * segment_y;
  const double dz = point_z - projection * segment_z;
  return dx * dx + dy * dy + dz * dz <= capsule.radius_squared;
}
}  // namespace

class NavigationCloudFilterNode final : public rclcpp::Node
{
public:
  NavigationCloudFilterNode()
  : Node("navigation_cloud_filter"),
    input_topic_(declare_parameter<std::string>("input_topic", "/unitree/slam_lidar/points")),
    output_topic_(declare_parameter<std::string>("output_topic", "/navigation/obstacles")),
    target_frame_(declare_parameter<std::string>("target_frame", "base_link")),
    transform_timeout_(positive_parameter("transform_timeout", 0.10)),
    input_timeout_(positive_parameter("input_timeout", 0.50)),
    invalid_return_radius_(positive_parameter("invalid_return_radius", 0.08)),
    dynamic_self_mask_enabled_(declare_parameter<bool>("dynamic_self_mask_enabled", true)),
    require_dynamic_self_mask_(declare_parameter<bool>("require_dynamic_self_mask", true)),
    self_min_x_(declare_parameter<double>("self_min_x", -0.52)),
    self_max_x_(declare_parameter<double>("self_max_x", 0.52)),
    self_min_y_(declare_parameter<double>("self_min_y", -0.34)),
    self_max_y_(declare_parameter<double>("self_max_y", 0.34)),
    self_min_z_(declare_parameter<double>("self_min_z", -0.35)),
    self_max_z_(declare_parameter<double>("self_max_z", 0.55)),
    min_height_(declare_parameter<double>("min_height", -0.30)),
    max_height_(declare_parameter<double>("max_height", 0.80)),
    min_range_(declare_parameter<double>("min_range", 0.10)),
    max_range_(positive_parameter("max_range", 6.0)),
    voxel_size_(positive_parameter("voxel_size", 0.05)),
    minimum_voxel_neighbors_(declare_parameter<std::int64_t>("minimum_voxel_neighbors", 0)),
    minimum_output_points_(declare_parameter<std::int64_t>("minimum_output_points", 5)),
    maximum_points_in_self_box_(
      declare_parameter<std::int64_t>("maximum_points_in_self_box", 0)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    load_invalid_return_centers();
    load_leg_configuration();
    validate_parameters();

    output_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    input_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {filter(*message);});
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostics();});

    RCLCPP_INFO(
      get_logger(), "Navigation cloud boundary: %s -> %s (%s)", input_topic_.c_str(),
      output_topic_.c_str(), target_frame_.c_str());
  }

private:
  double positive_parameter(const std::string & name, const double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  void validate_parameters() const
  {
    if (target_frame_.empty() || input_topic_.empty() || output_topic_.empty()) {
      throw std::invalid_argument("topic and target frame parameters must not be empty");
    }
    if (!(self_min_x_ < self_max_x_ && self_min_y_ < self_max_y_ &&
      self_min_z_ < self_max_z_ && min_height_ < max_height_ &&
      min_range_ >= 0.0 && min_range_ < max_range_))
    {
      throw std::invalid_argument("invalid navigation cloud filter bounds");
    }
    if (minimum_output_points_ < 0 || maximum_points_in_self_box_ < 0 ||
      minimum_voxel_neighbors_ < 0 || minimum_voxel_neighbors_ > 26)
    {
      throw std::invalid_argument("point-count thresholds must be non-negative");
    }
    if (leg_prefixes_.empty() || leg_capsule_radii_.size() != 4U ||
      std::any_of(
        leg_capsule_radii_.begin(), leg_capsule_radii_.end(),
        [](const double radius) {return !std::isfinite(radius) || radius <= 0.0;}))
    {
      throw std::invalid_argument("leg self-mask configuration is invalid");
    }
  }

  void load_invalid_return_centers()
  {
    const auto values = declare_parameter<std::vector<double>>(
      "invalid_return_centers", {0.0, 0.0, 0.0, 0.0, 0.006, -0.618});
    if (values.empty() || values.size() % 3U != 0U) {
      throw std::invalid_argument("invalid_return_centers must contain XYZ triplets");
    }
    invalid_return_centers_.reserve(values.size() / 3U);
    for (std::size_t index = 0; index < values.size(); index += 3U) {
      if (!std::isfinite(values[index]) || !std::isfinite(values[index + 1U]) ||
        !std::isfinite(values[index + 2U]))
      {
        throw std::invalid_argument("invalid_return_centers must be finite");
      }
      invalid_return_centers_.push_back(Point{
        static_cast<float>(values[index]), static_cast<float>(values[index + 1U]),
        static_cast<float>(values[index + 2U])});
    }
  }

  void load_leg_configuration()
  {
    leg_prefixes_ = declare_parameter<std::vector<std::string>>(
      "leg_prefixes", {"FL", "FR", "RL", "RR"});
    leg_capsule_radii_ = declare_parameter<std::vector<double>>(
      "leg_capsule_radii", {0.10, 0.085, 0.065, 0.060});
  }

  bool build_leg_capsules(
    const builtin_interfaces::msg::Time & stamp, std::vector<Capsule> & capsules)
  {
    capsules.clear();
    if (!dynamic_self_mask_enabled_) {
      return true;
    }

    static const std::vector<std::string> suffixes{"hip", "thigh", "calf", "foot"};
    capsules.reserve(leg_prefixes_.size() * 4U);
    try {
      for (const auto & prefix : leg_prefixes_) {
        std::vector<Point> joints;
        joints.reserve(suffixes.size());
        for (const auto & suffix : suffixes) {
          const auto transform = tf_buffer_.lookupTransform(
            target_frame_, prefix + "_" + suffix, rclcpp::Time(stamp));
          joints.push_back(Point{
            static_cast<float>(transform.transform.translation.x),
            static_cast<float>(transform.transform.translation.y),
            static_cast<float>(transform.transform.translation.z)});
        }
        for (std::size_t index = 0; index + 1U < joints.size(); ++index) {
          const double radius = leg_capsule_radii_[index];
          capsules.push_back(Capsule{joints[index], joints[index + 1U], radius * radius});
        }
        const double foot_radius = leg_capsule_radii_.back();
        capsules.push_back(Capsule{joints.back(), joints.back(), foot_radius * foot_radius});
      }
    } catch (const tf2::TransformException & exception) {
      leg_transform_failures_++;
      last_error_ = std::string("leg self-mask TF unavailable: ") + exception.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", last_error_.c_str());
      capsules.clear();
      return false;
    }
    return true;
  }

  void filter(const sensor_msgs::msg::PointCloud2 & input)
  {
    input_seen_ = true;
    last_input_time_ = std::chrono::steady_clock::now();
    last_input_points_ = static_cast<std::uint64_t>(input.width) * input.height;

    geometry_msgs::msg::TransformStamped transform_message;
    try {
      transform_message = tf_buffer_.lookupTransform(
        target_frame_, input.header.frame_id, rclcpp::Time(input.header.stamp),
        rclcpp::Duration::from_seconds(transform_timeout_));
    } catch (const tf2::TransformException & exception) {
      transform_failures_++;
      last_error_ = std::string("TF unavailable: ") + exception.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", last_error_.c_str());
      return;
    }

    tf2::Transform transform;
    tf2::fromMsg(transform_message.transform, transform);
    std::vector<Capsule> leg_capsules;
    const bool leg_mask_ready = build_leg_capsules(input.header.stamp, leg_capsules);
    last_dynamic_self_mask_ready_ = leg_mask_ready;
    if (!leg_mask_ready && require_dynamic_self_mask_) {
      return;
    }
    const double invalid_radius_squared = invalid_return_radius_ * invalid_return_radius_;
    const double min_range_squared = min_range_ * min_range_;
    const double max_range_squared = max_range_ * max_range_;

    std::vector<std::pair<Point, VoxelKey>> voxel_points;
    voxel_points.reserve(static_cast<std::size_t>(last_input_points_ / 4U));
    std::unordered_set<VoxelKey, VoxelHash> occupied_voxels;
    occupied_voxels.reserve(static_cast<std::size_t>(last_input_points_ / 4U));

    std::uint64_t nonfinite = 0;
    std::uint64_t invalid_return = 0;
    std::uint64_t body_self = 0;
    std::uint64_t leg_self = 0;
    std::uint64_t height = 0;
    std::uint64_t range = 0;
    std::uint64_t voxel = 0;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(input, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(input, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iterator(input, "z");
      for (; x_iterator != x_iterator.end(); ++x_iterator, ++y_iterator, ++z_iterator) {
        const double source_x = static_cast<double>(*x_iterator);
        const double source_y = static_cast<double>(*y_iterator);
        const double source_z = static_cast<double>(*z_iterator);
        if (!std::isfinite(source_x) || !std::isfinite(source_y) ||
          !std::isfinite(source_z))
        {
          nonfinite++;
          continue;
        }

        bool matches_invalid_return = false;
        for (const auto & center : invalid_return_centers_) {
          if (inside_sphere(
              source_x, source_y, source_z, center, invalid_radius_squared))
          {
            matches_invalid_return = true;
            break;
          }
        }
        if (matches_invalid_return) {
          invalid_return++;
          continue;
        }

        const tf2::Vector3 transformed = transform * tf2::Vector3(source_x, source_y, source_z);
        const double x = transformed.x();
        const double y = transformed.y();
        const double z = transformed.z();
        if (x >= self_min_x_ && x <= self_max_x_ && y >= self_min_y_ &&
          y <= self_max_y_ && z >= self_min_z_ && z <= self_max_z_)
        {
          body_self++;
          continue;
        }
        if (std::any_of(
            leg_capsules.begin(), leg_capsules.end(),
            [x, y, z](const Capsule & capsule) {return inside_capsule(x, y, z, capsule);}))
        {
          leg_self++;
          continue;
        }
        if (z < min_height_ || z > max_height_) {
          height++;
          continue;
        }
        const double planar_range_squared = x * x + y * y;
        if (planar_range_squared < min_range_squared ||
          planar_range_squared > max_range_squared)
        {
          range++;
          continue;
        }

        const VoxelKey key{
          static_cast<std::int32_t>(std::floor(x / voxel_size_)),
          static_cast<std::int32_t>(std::floor(y / voxel_size_)),
          static_cast<std::int32_t>(std::floor(z / voxel_size_))};
        if (!occupied_voxels.insert(key).second) {
          voxel++;
          continue;
        }
        voxel_points.emplace_back(
          Point{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)}, key);
      }
    } catch (const std::runtime_error & exception) {
      malformed_clouds_++;
      last_error_ = std::string("malformed PointCloud2: ") + exception.what();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", last_error_.c_str());
      return;
    }

    std::vector<Point> output_points;
    output_points.reserve(voxel_points.size());
    std::uint64_t isolated = 0;
    for (const auto & [point, key] : voxel_points) {
      std::int64_t neighbors = 0;
      for (std::int32_t dx = -1; dx <= 1; ++dx) {
        for (std::int32_t dy = -1; dy <= 1; ++dy) {
          for (std::int32_t dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }
            if (occupied_voxels.count(VoxelKey{key.x + dx, key.y + dy, key.z + dz}) > 0U) {
              neighbors++;
            }
          }
        }
      }
      if (neighbors < minimum_voxel_neighbors_) {
        isolated++;
        continue;
      }
      output_points.push_back(point);
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = input.header;
    output.header.frame_id = target_frame_;
    output.height = 1U;
    output.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(output_points.size());
    sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");
    for (const auto & point : output_points) {
      *output_x = point.x;
      *output_y = point.y;
      *output_z = point.z;
      ++output_x;
      ++output_y;
      ++output_z;
    }

    last_nonfinite_points_ = nonfinite;
    last_invalid_return_points_ = invalid_return;
    last_body_self_points_ = body_self;
    last_leg_self_points_ = leg_self;
    last_self_points_ = body_self + leg_self;
    last_height_points_ = height;
    last_range_points_ = range;
    last_voxel_points_ = voxel;
    last_isolated_points_ = isolated;
    last_output_points_ = output_points.size();
    last_output_time_ = std::chrono::steady_clock::now();
    output_seen_ = true;
    last_error_.clear();
    output_publisher_->publish(output);
  }

  void publish_diagnostics()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const bool input_stale = !input_seen_ ||
      std::chrono::duration<double>(steady_now - last_input_time_).count() > input_timeout_;
    const bool output_stale = !output_seen_ ||
      std::chrono::duration<double>(steady_now - last_output_time_).count() > input_timeout_;
    const bool too_few_points = output_seen_ &&
      last_output_points_ < static_cast<std::uint64_t>(minimum_output_points_);
    const bool self_leak = last_points_in_self_box_ >
      static_cast<std::uint64_t>(maximum_points_in_self_box_);

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "u_robot/navigation_cloud";
    status.hardware_id = "unitree_a2_dual_lidar";
    if (malformed_clouds_ > 0U || self_leak) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = self_leak ? "self points leaked into navigation output" : last_error_;
    } else if (input_stale || output_stale || too_few_points || !last_error_.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = !last_error_.empty() ? last_error_ :
        (too_few_points ? "filtered cloud has too few points" : "waiting for fresh filtered cloud");
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "navigation obstacle cloud healthy";
    }

    status.values.push_back(key_value("input_topic", input_topic_));
    status.values.push_back(key_value("output_topic", output_topic_));
    status.values.push_back(key_value("input_stale", input_stale ? "true" : "false"));
    status.values.push_back(key_value("output_stale", output_stale ? "true" : "false"));
    status.values.push_back(key_value("input_points", std::to_string(last_input_points_)));
    status.values.push_back(key_value("output_points", std::to_string(last_output_points_)));
    status.values.push_back(key_value("nonfinite_removed", std::to_string(last_nonfinite_points_)));
    status.values.push_back(
      key_value("invalid_return_removed", std::to_string(last_invalid_return_points_)));
    status.values.push_back(key_value("self_removed", std::to_string(last_self_points_)));
    status.values.push_back(
      key_value("body_self_removed", std::to_string(last_body_self_points_)));
    status.values.push_back(key_value("leg_self_removed", std::to_string(last_leg_self_points_)));
    status.values.push_back(
      key_value("dynamic_self_mask_ready", last_dynamic_self_mask_ready_ ? "true" : "false"));
    status.values.push_back(key_value("height_removed", std::to_string(last_height_points_)));
    status.values.push_back(key_value("range_removed", std::to_string(last_range_points_)));
    status.values.push_back(key_value("voxel_removed", std::to_string(last_voxel_points_)));
    status.values.push_back(key_value("isolated_removed", std::to_string(last_isolated_points_)));
    status.values.push_back(key_value("transform_failures", std::to_string(transform_failures_)));
    status.values.push_back(
      key_value("leg_transform_failures", std::to_string(leg_transform_failures_)));
    status.values.push_back(key_value("malformed_clouds", std::to_string(malformed_clouds_)));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  const std::string input_topic_;
  const std::string output_topic_;
  const std::string target_frame_;
  const double transform_timeout_;
  const double input_timeout_;
  const double invalid_return_radius_;
  const bool dynamic_self_mask_enabled_;
  const bool require_dynamic_self_mask_;
  const double self_min_x_;
  const double self_max_x_;
  const double self_min_y_;
  const double self_max_y_;
  const double self_min_z_;
  const double self_max_z_;
  const double min_height_;
  const double max_height_;
  const double min_range_;
  const double max_range_;
  const double voxel_size_;
  const std::int64_t minimum_voxel_neighbors_;
  const std::int64_t minimum_output_points_;
  const std::int64_t maximum_points_in_self_box_;
  std::vector<Point> invalid_return_centers_;
  std::vector<std::string> leg_prefixes_;
  std::vector<double> leg_capsule_radii_;

  bool input_seen_{false};
  bool output_seen_{false};
  std::chrono::steady_clock::time_point last_input_time_{};
  std::chrono::steady_clock::time_point last_output_time_{};
  std::uint64_t last_input_points_{0};
  std::uint64_t last_output_points_{0};
  std::uint64_t last_nonfinite_points_{0};
  std::uint64_t last_invalid_return_points_{0};
  std::uint64_t last_self_points_{0};
  std::uint64_t last_body_self_points_{0};
  std::uint64_t last_leg_self_points_{0};
  std::uint64_t last_height_points_{0};
  std::uint64_t last_range_points_{0};
  std::uint64_t last_voxel_points_{0};
  std::uint64_t last_isolated_points_{0};
  std::uint64_t last_points_in_self_box_{0};
  std::uint64_t transform_failures_{0};
  std::uint64_t leg_transform_failures_{0};
  std::uint64_t malformed_clouds_{0};
  bool last_dynamic_self_mask_ready_{false};
  std::string last_error_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};
}  // namespace u_robot_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<u_robot_perception::NavigationCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
