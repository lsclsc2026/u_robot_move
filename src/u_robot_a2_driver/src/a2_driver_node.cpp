#include "u_robot_a2_driver/command_guard.hpp"
#include "u_robot_a2_driver/sport_command_packet.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace u_robot_a2_driver
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

class A2DriverNode final : public rclcpp::Node
{
public:
  A2DriverNode()
  : Node("a2_driver"),
    dry_run_(declare_parameter<bool>("dry_run", true)),
    input_topic_(declare_parameter<std::string>("input_topic", "/cmd_vel_safe")),
    socket_path_(declare_parameter<std::string>("sport_socket_path", "/tmp/u_robot_a2_sport.sock")),
    publish_rate_hz_(declare_parameter<double>("publish_rate_hz", 50.0)),
    guard_(
      declare_parameter<double>("max_vx", 0.5),
      declare_parameter<double>("max_vy", 0.3),
      declare_parameter<double>("max_wz", 0.6),
      std::chrono::milliseconds(declare_parameter<int>("command_timeout_ms", 250)))
  {
    if (publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }

    ipc_socket_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (ipc_socket_ < 0) throw std::runtime_error("failed to create Sport IPC socket");
    sport_address_.sun_family = AF_UNIX;
    if (socket_path_.size() >= sizeof(sport_address_.sun_path)) throw std::runtime_error("Sport socket path too long");
    std::memcpy(sport_address_.sun_path, socket_path_.c_str(), socket_path_.size() + 1);
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {on_command(*message);});
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "~/enable_control",
      [this](
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
        on_enable(request->data, *response);
      });
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    const auto command_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    command_timer_ = create_wall_timer(command_period, [this]() {on_command_timer();});
    diagnostic_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publish_diagnostics();});

    RCLCPP_WARN(
      get_logger(), "A2 driver started: dry_run=%s, control=disabled, input=%s",
      dry_run_ ? "true" : "false", input_topic_.c_str());
  }

  ~A2DriverNode() override
  {
    if (control_enabled_.load() && !dry_run_) {
      send_stop();
    }
    if (ipc_socket_ >= 0) ::close(ipc_socket_);
  }

private:
  void on_command(const geometry_msgs::msg::Twist & message)
  {
    const VelocityCommand command{message.linear.x, message.linear.y, message.angular.z};
    if (!guard_.is_finite(command)) {
      invalid_command_count_.fetch_add(1);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected non-finite velocity command");
      return;
    }

    const auto limited = guard_.clamp(command);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      latest_command_ = limited;
      last_command_time_ = std::chrono::steady_clock::now();
      have_command_ = true;
      stale_stop_sent_ = false;
    }
  }

  void on_enable(const bool enable, std_srvs::srv::SetBool::Response & response)
  {
    if (enable && dry_run_) {
      response.success = false;
      response.message = "control cannot be enabled while dry_run=true";
      return;
    }

    if (!enable && control_enabled_.exchange(false)) {
      send_stop();
    } else if (enable) {
      send_stop();
      control_enabled_.store(true);
    }

    response.success = true;
    response.message = enable ? "control enabled" : "control disabled";
    RCLCPP_WARN(get_logger(), "%s", response.message.c_str());
  }

  void on_command_timer()
  {
    if (!control_enabled_.load()) {
      return;
    }

    VelocityCommand command;
    bool should_stop = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!have_command_ || guard_.is_stale(std::chrono::steady_clock::now(), last_command_time_)) {
        if (!stale_stop_sent_) {
          stale_stop_sent_ = true;
          should_stop = true;
        }
      } else {
        command = latest_command_;
      }
    }

    if (should_stop) {
      send_stop();
      timeout_stop_count_.fetch_add(1);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Command timeout: stop requested");
      return;
    }

    if (!stale_stop_sent_) {
      send_packet(SportCommandType::move, command);
    }
  }

  void send_stop() {send_packet(SportCommandType::stop, VelocityCommand{});}

  void send_packet(const SportCommandType type, const VelocityCommand & command)
  {
    if (dry_run_) {
      return;
    }
    SportCommandPacket packet;
    packet.type = type;
    packet.vx = command.vx; packet.vy = command.vy; packet.wz = command.wz;
    const auto sent = ::sendto(ipc_socket_, &packet, sizeof(packet), MSG_DONTWAIT,
      reinterpret_cast<const sockaddr *>(&sport_address_), sizeof(sport_address_));
    if (sent != static_cast<ssize_t>(sizeof(packet))) ipc_send_failures_.fetch_add(1);
    else ipc_packets_sent_.fetch_add(1);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "u_robot/a2_driver";
    status.hardware_id = "unitree_a2";
    status.level = dry_run_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = dry_run_ ? "dry-run; requests suppressed" :
      (control_enabled_.load() ? "control enabled" : "control disabled");
    status.values.push_back(key_value("input_topic", input_topic_));
    status.values.push_back(key_value("control_enabled", control_enabled_.load() ? "true" : "false"));
    status.values.push_back(key_value("ipc_packets_sent", std::to_string(ipc_packets_sent_.load())));
    status.values.push_back(key_value("ipc_send_failures", std::to_string(ipc_send_failures_.load())));
    status.values.push_back(key_value("invalid_commands", std::to_string(invalid_command_count_.load())));
    status.values.push_back(key_value("timeout_stops", std::to_string(timeout_stop_count_.load())));
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  const bool dry_run_;
  const std::string input_topic_;
  const std::string socket_path_;
  const double publish_rate_hz_;
  CommandGuard guard_;

  std::atomic<bool> control_enabled_{false};
  std::atomic<std::uint64_t> ipc_packets_sent_{0};
  std::atomic<std::uint64_t> ipc_send_failures_{0};
  std::atomic<std::uint64_t> invalid_command_count_{0};
  std::atomic<std::uint64_t> timeout_stop_count_{0};

  std::mutex command_mutex_;
  VelocityCommand latest_command_;
  std::chrono::steady_clock::time_point last_command_time_{};
  bool have_command_{false};
  bool stale_stop_sent_{true};

  int ipc_socket_{-1};
  sockaddr_un sport_address_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace u_robot_a2_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<u_robot_a2_driver::A2DriverNode>());
  rclcpp::shutdown();
  return 0;
}
