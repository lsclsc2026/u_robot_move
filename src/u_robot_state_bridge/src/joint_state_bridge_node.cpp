#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <u_robot_state_bridge/native_joint_packet.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace u_robot_state_bridge
{
namespace
{
constexpr std::array<const char *, 12> kJointNames = {
  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
};

// Limits are copied from the wrapped official A2 URDF in the same order as
// kJointNames. They are a validation boundary only; they do not command or clamp
// the robot. Invalid converted samples are rejected instead of hiding a bad
// calibration behind saturated visualization values.
constexpr std::array<double, 12> kLowerLimits = {
  -1.01, -2.34, -2.77,
  -1.01, -2.34, -2.77,
  -1.01, -1.56, -2.77,
  -1.01, -1.56, -2.77,
};
constexpr std::array<double, 12> kUpperLimits = {
  1.01, 3.15, -0.54,
  1.01, 3.15, -0.54,
  1.01, 3.94, -0.54,
  1.01, 3.94, -0.54,
};

diagnostic_msgs::msg::KeyValue key_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

geometry_msgs::msg::Point point(const double x, const double y, const double z = 0.0)
{
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}
}  // namespace

class JointStateBridgeNode final : public rclcpp::Node
{
public:
  JointStateBridgeNode()
  : Node("joint_state_bridge"),
    source_topic_(declare_parameter<std::string>("source_topic", "rt/lf/lowstate")),
    ipc_channel_(declare_parameter<std::string>("ipc_channel", "u_robot_a2_lowstate")),
    output_topic_(declare_parameter<std::string>("output_topic", "/joint_states")),
    raw_output_topic_(
      declare_parameter<std::string>("raw_output_topic", "/joint_states/raw_motor")),
    calibration_profile_(
      declare_parameter<std::string>("calibration_profile", "identity_unqualified")),
    calibration_confirmed_(declare_parameter<bool>("calibration_confirmed", false)),
    enforce_position_limits_(declare_parameter<bool>("enforce_position_limits", true)),
    limit_tolerance_rad_(
      declare_positive_parameter("position_limit_tolerance_rad", 0.05)),
    position_scale_(declare_parameter<std::vector<double>>(
      "position_scale", std::vector<double>(kJointNames.size(), 1.0))),
    position_direction_(declare_parameter<std::vector<double>>(
      "position_direction", std::vector<double>(kJointNames.size(), 1.0))),
    position_offset_(declare_parameter<std::vector<double>>(
      "position_offset", std::vector<double>(kJointNames.size(), 0.0))),
    footprint_topic_(
      declare_parameter<std::string>("footprint_topic", "/visualization/robot_footprint")),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    output_period_(1.0 / declare_positive_parameter("output_rate_hz", 50.0)),
    footprint_period_(
      1.0 / declare_positive_parameter("footprint_publish_rate_hz", 20.0)),
    ipc_poll_period_(1.0 / declare_positive_parameter("ipc_poll_rate_hz", 500.0)),
    source_timeout_(
      std::chrono::milliseconds(declare_parameter<int>("source_timeout_ms", 250))),
    footprint_front_(declare_positive_parameter("footprint_front", 0.44)),
    footprint_rear_(declare_positive_parameter("footprint_rear", 0.46)),
    footprint_half_width_(declare_positive_parameter("footprint_half_width", 0.29)),
    footprint_vertices_(declare_parameter<std::vector<double>>(
      "footprint_vertices",
      {0.44, 0.10, 0.44, -0.10, 0.35, -0.28, -0.32, -0.29,
        -0.46, -0.26, -0.46, 0.26, -0.32, 0.29, 0.35, 0.28})),
    footprint_visual_z_(declare_positive_parameter("footprint_visual_z", 0.55)),
    footprint_line_width_(declare_positive_parameter("footprint_line_width", 0.035))
  {
    if (source_timeout_.count() <= 0) {
      throw std::invalid_argument("source_timeout_ms must be positive");
    }
    if (ipc_channel_.empty() || ipc_channel_.size() >= sizeof(sockaddr_un::sun_path) - 1U) {
      throw std::invalid_argument("ipc_channel is empty or too long");
    }
    if (footprint_vertices_.size() < 6U || footprint_vertices_.size() % 2U != 0U ||
      std::any_of(
        footprint_vertices_.begin(), footprint_vertices_.end(),
        [](const double value) {return !std::isfinite(value);}))
    {
      throw std::invalid_argument("footprint_vertices must contain at least three finite XY pairs");
    }
    validate_calibration();

    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      output_topic_, rclcpp::SensorDataQoS());
    raw_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      raw_output_topic_, rclcpp::SensorDataQoS());
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    footprint_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      footprint_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    initialize_ipc_receiver();

    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostics();});
    footprint_timer_ = create_wall_timer(
      footprint_period_, [this]() {publish_footprint();});
    ipc_poll_timer_ = create_wall_timer(
      ipc_poll_period_, [this]() {poll_native_joint_state();});

    RCLCPP_INFO(
      get_logger(),
      "Publishing A2 joints from native DDS %s to %s at up to %.1f Hz "
      "(profile=%s, confirmed=%s)",
      source_topic_.c_str(), output_topic_.c_str(), 1.0 / output_period_.count(),
      calibration_profile_.c_str(), calibration_confirmed_ ? "true" : "false");
  }

  ~JointStateBridgeNode() override
  {
    if (ipc_socket_ >= 0) {
      ::close(ipc_socket_);
    }
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

  void validate_calibration() const
  {
    const auto validate_size = [](const std::vector<double> & values, const char * name) {
        if (values.size() != kJointNames.size()) {
          throw std::invalid_argument(
                  std::string(name) + " must contain exactly " +
                  std::to_string(kJointNames.size()) + " values");
        }
      };
    validate_size(position_scale_, "position_scale");
    validate_size(position_direction_, "position_direction");
    validate_size(position_offset_, "position_offset");

    for (std::size_t index = 0; index < kJointNames.size(); ++index) {
      if (!std::isfinite(position_scale_[index]) || position_scale_[index] <= 0.0) {
        throw std::invalid_argument("position_scale values must be finite and positive");
      }
      if (position_direction_[index] != -1.0 && position_direction_[index] != 1.0) {
        throw std::invalid_argument("position_direction values must be exactly -1 or 1");
      }
      if (!std::isfinite(position_offset_[index])) {
        throw std::invalid_argument("position_offset values must be finite");
      }
    }
  }

  void initialize_ipc_receiver()
  {
    ipc_socket_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (ipc_socket_ < 0) {
      throw std::runtime_error("failed to create native-state IPC socket");
    }
    int receive_buffer = 256 * 1024;
    (void)::setsockopt(
      ipc_socket_, SOL_SOCKET, SO_RCVBUF, &receive_buffer, sizeof(receive_buffer));

    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    address.sun_path[0] = '\0';
    std::memcpy(address.sun_path + 1, ipc_channel_.data(), ipc_channel_.size());
    const auto address_length = static_cast<socklen_t>(
      offsetof(sockaddr_un, sun_path) + 1U + ipc_channel_.size());
    if (::bind(
        ipc_socket_, reinterpret_cast<const sockaddr *>(&address), address_length) != 0)
    {
      const int error = errno;
      ::close(ipc_socket_);
      ipc_socket_ = -1;
      throw std::runtime_error(
              "failed to bind native-state IPC channel: errno=" + std::to_string(error));
    }
  }

  void poll_native_joint_state()
  {
    NativeJointPacket newest;
    bool have_packet = false;
    while (true) {
      NativeJointPacket candidate;
      const ssize_t received = ::recv(ipc_socket_, &candidate, sizeof(candidate), MSG_DONTWAIT);
      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        if (errno == EINTR) {
          continue;
        }
        invalid_message_count_++;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Native-state IPC receive failed: errno=%d", errno);
        break;
      }
      if (received != static_cast<ssize_t>(sizeof(candidate)) ||
        !has_valid_native_joint_header(candidate))
      {
        invalid_message_count_++;
        continue;
      }
      newest = candidate;
      have_packet = true;
    }
    if (have_packet) {
      on_native_joint_packet(newest);
    }
  }

  void on_native_joint_packet(const NativeJointPacket & source)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    source_seen_ = true;
    last_source_time_ = steady_now;
    if (last_output_time_.time_since_epoch().count() != 0 &&
      steady_now - last_output_time_ < output_period_)
    {
      return;
    }

    sensor_msgs::msg::JointState output;
    output.header.stamp = now();
    sensor_msgs::msg::JointState raw_output;
    raw_output.header = output.header;
    output.name.reserve(kJointNames.size());
    output.position.reserve(kJointNames.size());
    output.velocity.reserve(kJointNames.size());
    output.effort.reserve(kJointNames.size());
    raw_output.name.reserve(kJointNames.size());
    raw_output.position.reserve(kJointNames.size());
    raw_output.velocity.reserve(kJointNames.size());
    raw_output.effort.reserve(kJointNames.size());

    for (std::size_t index = 0; index < kJointNames.size(); ++index) {
      if (!std::isfinite(source.position[index]) ||
        !std::isfinite(source.velocity[index]) ||
        !std::isfinite(source.effort[index]))
      {
        invalid_message_count_++;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Discarding non-finite A2 motor state at index %zu", index);
        return;
      }
      raw_output.name.emplace_back(kJointNames[index]);
      raw_output.position.push_back(source.position[index]);
      raw_output.velocity.push_back(source.velocity[index]);
      raw_output.effort.push_back(source.effort[index]);
    }
    raw_joint_state_publisher_->publish(raw_output);

    for (std::size_t index = 0; index < kJointNames.size(); ++index) {
      const double direction = position_direction_[index];
      const double scale = position_scale_[index];
      const double position =
        direction * scale * source.position[index] + position_offset_[index];
      const double velocity = direction * scale * source.velocity[index];

      if (enforce_position_limits_ &&
        (position < kLowerLimits[index] - limit_tolerance_rad_ ||
        position > kUpperLimits[index] + limit_tolerance_rad_))
      {
        limit_violation_count_++;
        last_limit_violation_joint_ = kJointNames[index];
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejecting converted %s=%.4f rad outside [%.4f, %.4f] (profile=%s)",
          kJointNames[index], position, kLowerLimits[index], kUpperLimits[index],
          calibration_profile_.c_str());
        return;
      }
      output.name.emplace_back(kJointNames[index]);
      output.position.push_back(position);
      output.velocity.push_back(velocity);
      // tau_est semantics are not yet qualified for A2-Pro transmission-side
      // conversion. Preserve the source value for diagnostics only.
      output.effort.push_back(source.effort[index]);
    }

    last_output_time_ = steady_now;
    joint_state_publisher_->publish(output);
    output_count_++;
  }

  void publish_footprint()
  {
    visualization_msgs::msg::MarkerArray array;

    // This marker is deliberately raised above base_link for a legible top-down
    // operator view. It is visualization-only and does not alter TF or costmaps.
    visualization_msgs::msg::Marker fill;
    fill.header.stamp = now();
    fill.header.frame_id = base_frame_;
    fill.ns = "a2_navigation_footprint";
    fill.id = 0;
    fill.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    fill.action = visualization_msgs::msg::Marker::ADD;
    fill.frame_locked = true;
    fill.pose.position.z = footprint_visual_z_;
    fill.pose.orientation.w = 1.0;
    fill.scale.x = 1.0;
    fill.scale.y = 1.0;
    fill.scale.z = 1.0;
    fill.color.r = 1.0F;
    fill.color.g = 0.55F;
    fill.color.b = 0.0F;
    fill.color.a = 0.22F;
    const std::size_t footprint_point_count = footprint_vertices_.size() / 2U;
    for (std::size_t index = 1U; index + 1U < footprint_point_count; ++index) {
      fill.points.push_back(point(footprint_vertices_[0], footprint_vertices_[1]));
      fill.points.push_back(
        point(footprint_vertices_[2U * index], footprint_vertices_[2U * index + 1U]));
      fill.points.push_back(point(
        footprint_vertices_[2U * (index + 1U)],
        footprint_vertices_[2U * (index + 1U) + 1U]));
    }
    array.markers.push_back(fill);

    visualization_msgs::msg::Marker outline;
    outline.header = fill.header;
    outline.ns = fill.ns;
    outline.id = 1;
    outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
    outline.action = visualization_msgs::msg::Marker::ADD;
    outline.frame_locked = true;
    outline.pose.orientation.w = 1.0;
    outline.scale.x = footprint_line_width_;
    outline.color.r = 1.0F;
    outline.color.g = 0.75F;
    outline.color.b = 0.0F;
    outline.color.a = 1.0F;
    for (std::size_t index = 0U; index < footprint_point_count; ++index) {
      outline.points.push_back(point(
        footprint_vertices_[2U * index], footprint_vertices_[2U * index + 1U],
        footprint_visual_z_));
    }
    outline.points.push_back(point(
      footprint_vertices_[0], footprint_vertices_[1], footprint_visual_z_));
    array.markers.push_back(outline);

    visualization_msgs::msg::Marker forward;
    forward.header = outline.header;
    forward.ns = outline.ns;
    forward.id = 2;
    forward.type = visualization_msgs::msg::Marker::ARROW;
    forward.action = visualization_msgs::msg::Marker::ADD;
    forward.frame_locked = true;
    forward.pose.orientation.w = 1.0;
    forward.scale.x = footprint_line_width_;
    forward.scale.y = 0.16;
    forward.scale.z = 0.20;
    forward.color.r = 0.1F;
    forward.color.g = 0.9F;
    forward.color.b = 0.2F;
    forward.color.a = 1.0F;
    forward.points = {
      point(0.0, 0.0, footprint_visual_z_ + 0.025),
      point(footprint_front_, 0.0, footprint_visual_z_ + 0.025),
    };
    array.markers.push_back(forward);

    footprint_publisher_->publish(array);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "u_robot/joint_state_bridge";
    status.hardware_id = "unitree_a2";

    const bool stale = !source_seen_ ||
      std::chrono::steady_clock::now() - last_source_time_ > source_timeout_;
    if (stale) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "source joint state stale or absent";
    } else if (!calibration_confirmed_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "joint conversion active with provisional calibration";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "source and calibrated joint state healthy";
    }
    status.values.push_back(key_value("source_topic", source_topic_));
    status.values.push_back(key_value("ipc_channel", ipc_channel_));
    status.values.push_back(key_value("output_topic", output_topic_));
    status.values.push_back(key_value("raw_output_topic", raw_output_topic_));
    status.values.push_back(key_value("calibration_profile", calibration_profile_));
    status.values.push_back(
      key_value("calibration_confirmed", calibration_confirmed_ ? "true" : "false"));
    status.values.push_back(key_value("output_count", std::to_string(output_count_)));
    status.values.push_back(
      key_value("invalid_messages", std::to_string(invalid_message_count_)));
    status.values.push_back(
      key_value("limit_violations", std::to_string(limit_violation_count_)));
    status.values.push_back(
      key_value("last_limit_violation_joint", last_limit_violation_joint_));
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  const std::string source_topic_;
  const std::string ipc_channel_;
  const std::string output_topic_;
  const std::string raw_output_topic_;
  const std::string calibration_profile_;
  const bool calibration_confirmed_;
  const bool enforce_position_limits_;
  const double limit_tolerance_rad_;
  const std::vector<double> position_scale_;
  const std::vector<double> position_direction_;
  const std::vector<double> position_offset_;
  const std::string footprint_topic_;
  const std::string base_frame_;
  const std::chrono::duration<double> output_period_;
  const std::chrono::duration<double> footprint_period_;
  const std::chrono::duration<double> ipc_poll_period_;
  const std::chrono::milliseconds source_timeout_;
  const double footprint_front_;
  const double footprint_rear_;
  const double footprint_half_width_;
  const std::vector<double> footprint_vertices_;
  const double footprint_visual_z_;
  const double footprint_line_width_;

  bool source_seen_{false};
  std::chrono::steady_clock::time_point last_source_time_{};
  std::chrono::steady_clock::time_point last_output_time_{};
  std::uint64_t output_count_{0};
  std::uint64_t invalid_message_count_{0};
  std::uint64_t limit_violation_count_{0};
  std::string last_limit_violation_joint_{"none"};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr raw_joint_state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_publisher_;
  int ipc_socket_{-1};
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  rclcpp::TimerBase::SharedPtr footprint_timer_;
  rclcpp::TimerBase::SharedPtr ipc_poll_timer_;
};

}  // namespace u_robot_state_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<u_robot_state_bridge::JointStateBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
