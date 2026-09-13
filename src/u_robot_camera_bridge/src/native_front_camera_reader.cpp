#include <unitree/robot/b2/front_video/front_video_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <u_robot_camera_bridge/camera_packet.hpp>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
std::atomic_bool g_running{true};

void stop_handler(int)
{
  g_running.store(false);
}

struct Options
{
  std::string network_interface{"eth0"};
  std::string ipc_channel{"u_robot_a2_front_camera"};
  std::int32_t domain_id{0};
  double request_rate_hz{10.0};
};

Options parse_options(const int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      std::cout << "Usage: native_front_camera_reader [--network-interface IFACE] "
                   "[--domain-id ID] [--ipc-channel NAME] [--request-rate-hz HZ]\n";
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value after " + argument);
    }
    const std::string value(argv[++index]);
    if (argument == "--network-interface") {
      options.network_interface = value;
    } else if (argument == "--domain-id") {
      const long parsed = std::stol(value);
      if (parsed < 0 || parsed > 232) {
        throw std::invalid_argument("domain ID must be in [0, 232]");
      }
      options.domain_id = static_cast<std::int32_t>(parsed);
    } else if (argument == "--ipc-channel") {
      options.ipc_channel = value;
    } else if (argument == "--request-rate-hz") {
      options.request_rate_hz = std::stod(value);
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.network_interface.empty() || options.ipc_channel.empty()) {
    throw std::invalid_argument("interface and IPC channel must not be empty");
  }
  if (options.ipc_channel.size() >= sizeof(sockaddr_un::sun_path) - 1U) {
    throw std::invalid_argument("IPC channel name is too long");
  }
  if (!std::isfinite(options.request_rate_hz) || options.request_rate_hz <= 0.0 ||
    options.request_rate_hz > 30.0)
  {
    throw std::invalid_argument("request rate must be in (0, 30] Hz");
  }
  return options;
}

class AbstractStreamClient
{
public:
  explicit AbstractStreamClient(std::string channel) : channel_(std::move(channel)) {}

  ~AbstractStreamClient()
  {
    close_socket();
  }

  bool ensure_connected()
  {
    if (socket_ >= 0) {
      return true;
    }
    socket_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (socket_ < 0) {
      return false;
    }

    timeval send_timeout{};
    send_timeout.tv_sec = 0;
    send_timeout.tv_usec = 300000;
    (void)::setsockopt(
      socket_, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));

    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    address.sun_path[0] = '\0';
    std::memcpy(address.sun_path + 1, channel_.data(), channel_.size());
    const auto address_length = static_cast<socklen_t>(
      offsetof(sockaddr_un, sun_path) + 1U + channel_.size());
    if (::connect(
        socket_, reinterpret_cast<const sockaddr *>(&address), address_length) != 0)
    {
      close_socket();
      return false;
    }
    return true;
  }

  bool send_frame(
    const u_robot_camera_bridge::CameraPacketHeader & header,
    const std::vector<std::uint8_t> & jpeg)
  {
    if (!send_all(&header, sizeof(header)) || !send_all(jpeg.data(), jpeg.size())) {
      close_socket();
      return false;
    }
    return true;
  }

private:
  bool send_all(const void * source, const std::size_t size)
  {
    auto * cursor = static_cast<const std::uint8_t *>(source);
    std::size_t remaining = size;
    while (remaining > 0U && g_running.load()) {
      const auto sent = ::send(socket_, cursor, remaining, MSG_NOSIGNAL);
      if (sent > 0) {
        const auto sent_size = static_cast<std::size_t>(sent);
        cursor += sent_size;
        remaining -= sent_size;
        continue;
      }
      if (sent < 0 && errno == EINTR) {
        continue;
      }
      return false;
    }
    return remaining == 0U;
  }

  void close_socket()
  {
    if (socket_ >= 0) {
      ::close(socket_);
      socket_ = -1;
    }
  }

  std::string channel_;
  int socket_{-1};
};

std::uint64_t monotonic_nanoseconds()
{
  const auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  return static_cast<std::uint64_t>(value);
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parse_options(argc, argv);
    std::signal(SIGINT, stop_handler);
    std::signal(SIGTERM, stop_handler);
    std::signal(SIGPIPE, SIG_IGN);

    unitree::robot::ChannelFactory::Instance()->Init(
      options.domain_id, options.network_interface);
    unitree::robot::b2::FrontVideoClient camera;
    camera.SetTimeout(1.5F);
    camera.Init();

    AbstractStreamClient ipc(options.ipc_channel);
    std::uint64_t sequence{0U};
    std::uint64_t request_failures{0U};
    std::uint64_t invalid_images{0U};
    std::uint64_t ipc_failures{0U};
    auto last_report = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / options.request_rate_hz);

    std::cout << "[native_front_camera_reader] read-only front_videohub on "
              << options.network_interface << "/domain " << options.domain_id
              << " -> abstract stream IPC " << options.ipc_channel << " at up to "
              << options.request_rate_hz << " Hz" << std::endl;

    while (g_running.load()) {
      const auto cycle_start = std::chrono::steady_clock::now();
      if (!ipc.ensure_connected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        continue;
      }

      std::vector<std::uint8_t> jpeg;
      const std::int32_t result = camera.GetImageSample(jpeg);
      if (result != 0) {
        ++request_failures;
      } else if (
        jpeg.size() > u_robot_camera_bridge::kDefaultMaximumJpegBytes ||
        jpeg.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()) ||
        !u_robot_camera_bridge::looks_like_jpeg(jpeg.data(), jpeg.size()))
      {
        ++invalid_images;
      } else {
        u_robot_camera_bridge::CameraPacketHeader header;
        header.sequence = ++sequence;
        header.source_monotonic_ns = monotonic_nanoseconds();
        header.payload_size = static_cast<std::uint32_t>(jpeg.size());
        if (!ipc.send_frame(header, jpeg)) {
          ++ipc_failures;
        }
      }

      const auto now = std::chrono::steady_clock::now();
      if (now - last_report >= std::chrono::seconds(5)) {
        std::cout << "[native_front_camera_reader] frames=" << sequence
                  << " request_failures=" << request_failures
                  << " invalid_images=" << invalid_images
                  << " ipc_failures=" << ipc_failures << std::endl;
        last_report = now;
      }
      const auto elapsed = std::chrono::steady_clock::now() - cycle_start;
      const auto sleep_duration = period - elapsed;
      if (sleep_duration > std::chrono::duration<double>::zero()) {
        std::this_thread::sleep_for(sleep_duration);
      }
    }

    unitree::robot::ChannelFactory::Instance()->Release();
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "[native_front_camera_reader] fatal: " << exception.what() << std::endl;
    return 1;
  }
}
