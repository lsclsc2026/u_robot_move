#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <u_robot_state_bridge/native_joint_packet.hpp>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
volatile std::sig_atomic_t g_stop_requested{0};

void stop_handler(int)
{
  g_stop_requested = 1;
}

struct Options
{
  std::string source_topic{"rt/lf/lowstate"};
  std::string network_interface{"eth0"};
  std::string ipc_channel{"u_robot_a2_lowstate"};
  std::int32_t domain_id{0};
};

Options parse_options(const int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      std::cout << "Usage: native_low_state_reader [--source-topic TOPIC] "
                   "[--network-interface IFACE] [--domain-id ID] "
                   "[--ipc-channel NAME]\n";
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value after " + argument);
    }
    const std::string value(argv[++index]);
    if (argument == "--source-topic") {
      options.source_topic = value;
    } else if (argument == "--network-interface") {
      options.network_interface = value;
    } else if (argument == "--domain-id") {
      const long parsed = std::stol(value);
      if (parsed < 0 || parsed > 232) {
        throw std::invalid_argument("domain ID must be in [0, 232]");
      }
      options.domain_id = static_cast<std::int32_t>(parsed);
    } else if (argument == "--ipc-channel") {
      options.ipc_channel = value;
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.source_topic.empty() || options.network_interface.empty() ||
    options.ipc_channel.empty())
  {
    throw std::invalid_argument("topic, interface, and IPC channel must not be empty");
  }
  if (options.ipc_channel.size() >= sizeof(sockaddr_un::sun_path) - 1U) {
    throw std::invalid_argument("IPC channel name is too long");
  }
  return options;
}

class NativeLowStateForwarder
{
public:
  explicit NativeLowStateForwarder(std::string ipc_channel)
  : ipc_channel_(std::move(ipc_channel))
  {
    socket_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (socket_ < 0) {
      throw std::runtime_error("failed to create native-state IPC socket");
    }
    destination_.sun_family = AF_UNIX;
    destination_.sun_path[0] = '\0';
    std::memcpy(destination_.sun_path + 1, ipc_channel_.data(), ipc_channel_.size());
    destination_length_ = static_cast<socklen_t>(
      offsetof(sockaddr_un, sun_path) + 1U + ipc_channel_.size());
  }

  ~NativeLowStateForwarder()
  {
    if (socket_ >= 0) {
      ::close(socket_);
    }
  }

  void on_low_state(const unitree_hg::msg::dds_::LowState_ & source)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (!accepting_messages_) {
      return;
    }

    u_robot_state_bridge::NativeJointPacket packet;
    packet.source_tick = source.tick();
    packet.sequence = ++sequence_;

    for (std::size_t index = 0; index < u_robot_state_bridge::kA2LegJointCount; ++index) {
      const auto & motor = source.motor_state().at(index);
      if (!std::isfinite(motor.q()) || !std::isfinite(motor.dq()) ||
        !std::isfinite(motor.tau_est()))
      {
        ++invalid_count_;
        report_periodically("discarded non-finite native motor state");
        return;
      }
      packet.position[index] = static_cast<double>(motor.q());
      packet.velocity[index] = static_cast<double>(motor.dq());
      packet.effort[index] = static_cast<double>(motor.tau_est());
    }

    const auto sent = ::sendto(
      socket_, &packet, sizeof(packet), MSG_DONTWAIT,
      reinterpret_cast<const sockaddr *>(&destination_), destination_length_);
    if (sent != static_cast<ssize_t>(sizeof(packet))) {
      ++send_drop_count_;
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != ECONNREFUSED &&
        errno != ENOENT)
      {
        report_periodically("native-state IPC send failed");
      }
    }
  }

  void stop_accepting_messages()
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    accepting_messages_ = false;
  }

private:
  void report_periodically(const char * message)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_report_time_ >= std::chrono::seconds(2)) {
      std::cerr << "[native_low_state_reader] " << message
                << " (invalid=" << invalid_count_ << ", send_drops=" << send_drop_count_
                << ")\n";
      last_report_time_ = now;
    }
  }

  std::string ipc_channel_;
  int socket_{-1};
  sockaddr_un destination_{};
  socklen_t destination_length_{0};
  std::uint64_t sequence_{0U};
  std::uint64_t invalid_count_{0U};
  std::uint64_t send_drop_count_{0U};
  std::chrono::steady_clock::time_point last_report_time_{};
  std::mutex callback_mutex_;
  bool accepting_messages_{true};
};
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parse_options(argc, argv);
    std::signal(SIGINT, stop_handler);
    std::signal(SIGTERM, stop_handler);

    unitree::robot::ChannelFactory::Instance()->Init(
      options.domain_id, options.network_interface);
    NativeLowStateForwarder forwarder(options.ipc_channel);
    auto subscriber = std::make_shared<
      unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
      options.source_topic);
    subscriber->InitChannel(
      [&forwarder](const void * message) {
        forwarder.on_low_state(
          *static_cast<const unitree_hg::msg::dds_::LowState_ *>(message));
      },
      1);

    std::cout << "[native_low_state_reader] read-only " << options.source_topic
              << " on " << options.network_interface << "/domain " << options.domain_id
              << " -> abstract IPC " << options.ipc_channel << std::endl;
    while (g_stop_requested == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    forwarder.stop_accepting_messages();
    std::cout << "[native_low_state_reader] stopping subscription" << std::endl;
    subscriber->CloseChannel();
    std::cout << "[native_low_state_reader] subscription closed" << std::endl;
    subscriber.reset();
    std::cout << "[native_low_state_reader] subscriber released" << std::endl;

    // SDK2's process-global ChannelFactory::Release() intermittently corrupts
    // its DDS heap after a live LowState reader has been closed. This adapter is
    // deliberately isolated in its own read-only process, so the safe boundary
    // is to close the subscription above and let process exit reclaim the vendor
    // factory, DDS worker threads, and sockets. _Exit also bypasses the factory's
    // static destructor, which reaches the same faulty cleanup path.
    std::cout << "[native_low_state_reader] subscription stopped; exiting process"
              << std::endl;
    std::_Exit(EXIT_SUCCESS);
  } catch (const std::exception & exception) {
    std::cerr << "[native_low_state_reader] fatal: " << exception.what() << std::endl;
    return 1;
  }
}
