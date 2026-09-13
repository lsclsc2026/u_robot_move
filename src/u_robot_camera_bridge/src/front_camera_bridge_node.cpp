#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <u_robot_camera_bridge/camera_packet.hpp>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::int64_t steady_nanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class FrontCameraBridgeNode final : public rclcpp::Node
{
public:
  FrontCameraBridgeNode() : Node("front_camera_bridge")
  {
    ipc_channel_ = declare_parameter<std::string>(
      "ipc_channel", "u_robot_a2_front_camera");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/camera/front/image/compressed");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
    const auto maximum_jpeg_bytes = declare_parameter<std::int64_t>(
      "maximum_jpeg_bytes",
      static_cast<std::int64_t>(u_robot_camera_bridge::kDefaultMaximumJpegBytes));
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 3.0);

    if (ipc_channel_.empty() || output_topic_.empty() || frame_id_.empty()) {
      throw std::invalid_argument("IPC channel, output topic, and frame ID must not be empty");
    }
    if (ipc_channel_.size() >= sizeof(sockaddr_un::sun_path) - 1U) {
      throw std::invalid_argument("IPC channel name is too long");
    }
    if (maximum_jpeg_bytes <= 0 ||
      maximum_jpeg_bytes > static_cast<std::int64_t>(64U * 1024U * 1024U))
    {
      throw std::invalid_argument("maximum_jpeg_bytes must be in (0, 64 MiB]");
    }
    if (!std::isfinite(stale_timeout_sec_) || stale_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("stale_timeout_sec must be finite and positive");
    }
    maximum_jpeg_bytes_ = static_cast<std::size_t>(maximum_jpeg_bytes);

    image_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      output_topic_, rclcpp::SensorDataQoS().keep_last(1));
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));

    create_ipc_listener();
    worker_ = std::thread([this]() { receive_loop(); });
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() { publish_diagnostics(); });

    RCLCPP_INFO(
      get_logger(), "Publishing read-only A2 front-camera JPEG from IPC %s to %s",
      ipc_channel_.c_str(), output_topic_.c_str());
  }

  ~FrontCameraBridgeNode() override
  {
    stop_.store(true);
    if (listener_ >= 0) {
      (void)::shutdown(listener_, SHUT_RDWR);
      ::close(listener_);
      listener_ = -1;
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void create_ipc_listener()
  {
    listener_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (listener_ < 0) {
      throw std::runtime_error("failed to create front-camera IPC socket");
    }

    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    address.sun_path[0] = '\0';
    std::memcpy(address.sun_path + 1, ipc_channel_.data(), ipc_channel_.size());
    const auto address_length = static_cast<socklen_t>(
      offsetof(sockaddr_un, sun_path) + 1U + ipc_channel_.size());
    if (::bind(
        listener_, reinterpret_cast<const sockaddr *>(&address), address_length) != 0)
    {
      const std::string message =
        "failed to bind front-camera IPC channel: errno=" + std::to_string(errno);
      ::close(listener_);
      listener_ = -1;
      throw std::runtime_error(message);
    }
    if (::listen(listener_, 1) != 0) {
      const std::string message = "failed to listen on front-camera IPC channel";
      ::close(listener_);
      listener_ = -1;
      throw std::runtime_error(message);
    }
  }

  bool receive_exact(const int socket, void * destination, const std::size_t size)
  {
    auto * cursor = static_cast<std::uint8_t *>(destination);
    std::size_t remaining = size;
    while (remaining > 0U && !stop_.load()) {
      pollfd descriptor{};
      descriptor.fd = socket;
      descriptor.events = POLLIN;
      const int result = ::poll(&descriptor, 1, 250);
      if (result == 0) {
        continue;
      }
      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        return false;
      }
      if ((descriptor.revents & POLLIN) == 0) {
        return false;
      }
      const auto received = ::recv(socket, cursor, remaining, 0);
      if (received <= 0) {
        if (received < 0 && errno == EINTR) {
          continue;
        }
        return false;
      }
      const auto received_size = static_cast<std::size_t>(received);
      cursor += received_size;
      remaining -= received_size;
    }
    return remaining == 0U;
  }

  void receive_loop()
  {
    while (!stop_.load()) {
      pollfd descriptor{};
      descriptor.fd = listener_;
      descriptor.events = POLLIN;
      const int poll_result = ::poll(&descriptor, 1, 250);
      if (poll_result <= 0) {
        if (poll_result < 0 && errno != EINTR && !stop_.load()) {
          ++ipc_errors_;
        }
        continue;
      }
      const int client = ::accept4(listener_, nullptr, nullptr, SOCK_CLOEXEC);
      if (client < 0) {
        if (errno != EINTR && !stop_.load()) {
          ++ipc_errors_;
        }
        continue;
      }
      ++connections_;
      receive_client(client);
      ::close(client);
    }
  }

  void receive_client(const int client)
  {
    while (!stop_.load()) {
      u_robot_camera_bridge::CameraPacketHeader header;
      if (!receive_exact(client, &header, sizeof(header))) {
        return;
      }
      if (!u_robot_camera_bridge::has_valid_camera_header(
          header, maximum_jpeg_bytes_))
      {
        ++invalid_frames_;
        return;
      }

      std::vector<std::uint8_t> jpeg(header.payload_size);
      if (!receive_exact(client, jpeg.data(), jpeg.size())) {
        ++ipc_errors_;
        return;
      }
      if (!u_robot_camera_bridge::looks_like_jpeg(jpeg.data(), jpeg.size())) {
        ++invalid_frames_;
        continue;
      }

      if (last_sequence_ != 0U && header.sequence > last_sequence_ + 1U) {
        dropped_frames_.fetch_add(header.sequence - last_sequence_ - 1U);
      }
      last_sequence_ = header.sequence;

      sensor_msgs::msg::CompressedImage message;
      message.header.stamp = now();
      message.header.frame_id = frame_id_;
      message.format = "jpeg";
      message.data = std::move(jpeg);
      last_payload_bytes_.store(message.data.size());
      image_publisher_->publish(std::move(message));
      ++published_frames_;
      last_receive_ns_.store(steady_nanoseconds());
    }
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "u_robot_camera_bridge/front_camera";
    status.hardware_id = "a2-pro/front-camera";

    const auto last_receive = last_receive_ns_.load();
    const double age = last_receive == 0 ? -1.0 :
      static_cast<double>(steady_nanoseconds() - last_receive) / 1.0e9;
    if (last_receive == 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for front-camera JPEG";
    } else if (age > stale_timeout_sec_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "front-camera JPEG is stale";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "front-camera JPEG healthy";
    }

    status.values.push_back(diagnostic_value("output_topic", output_topic_));
    status.values.push_back(diagnostic_value("frame_id", frame_id_));
    status.values.push_back(diagnostic_value(
      "published_frames", std::to_string(published_frames_.load())));
    status.values.push_back(diagnostic_value(
      "last_payload_bytes", std::to_string(last_payload_bytes_.load())));
    status.values.push_back(diagnostic_value(
      "dropped_frames", std::to_string(dropped_frames_.load())));
    status.values.push_back(diagnostic_value(
      "invalid_frames", std::to_string(invalid_frames_.load())));
    status.values.push_back(diagnostic_value(
      "ipc_errors", std::to_string(ipc_errors_.load())));
    status.values.push_back(diagnostic_value(
      "connections", std::to_string(connections_.load())));
    status.values.push_back(diagnostic_value("last_frame_age_sec", std::to_string(age)));
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(std::move(array));
  }

  std::string ipc_channel_;
  std::string output_topic_;
  std::string frame_id_;
  std::size_t maximum_jpeg_bytes_{u_robot_camera_bridge::kDefaultMaximumJpegBytes};
  double stale_timeout_sec_{3.0};
  int listener_{-1};
  std::atomic_bool stop_{false};
  std::thread worker_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::atomic<std::uint64_t> published_frames_{0U};
  std::atomic<std::uint64_t> dropped_frames_{0U};
  std::atomic<std::uint64_t> invalid_frames_{0U};
  std::atomic<std::uint64_t> ipc_errors_{0U};
  std::atomic<std::uint64_t> connections_{0U};
  std::atomic<std::size_t> last_payload_bytes_{0U};
  std::atomic<std::int64_t> last_receive_ns_{0};
  std::uint64_t last_sequence_{0U};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FrontCameraBridgeNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("front_camera_bridge"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
