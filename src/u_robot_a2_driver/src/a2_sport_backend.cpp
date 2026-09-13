#include "u_robot_a2_driver/sport_command_packet.hpp"
#include "u_robot_a2_driver/sport_control_lock.hpp"
#include <unitree/robot/a2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
std::atomic_bool running{true};
void signal_handler(int) {running.store(false);}
}

int main(int argc, char ** argv) {
  std::string interface_name = "eth0";
  std::string socket_path = "/tmp/u_robot_a2_sport.sock";
  int domain = 0;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key{argv[i]};
    if (key == "--network-interface") interface_name = argv[i + 1];
    else if (key == "--socket-path") socket_path = argv[i + 1];
    else if (key == "--domain-id") domain = std::stoi(argv[i + 1]);
    else {std::cerr << "unknown option: " << key << '\n'; return 2;}
  }
  int fd = -1;
  bool owns_socket_path = false;
  std::unique_ptr<u_robot_a2_driver::SportControlLock> control_lock;
  try {
    control_lock = std::make_unique<u_robot_a2_driver::SportControlLock>();
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    ::unlink(socket_path.c_str());
    owns_socket_path = true;
    fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) throw std::runtime_error("socket creation failed");
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (socket_path.size() >= sizeof(address.sun_path)) throw std::runtime_error("socket path too long");
    std::memcpy(address.sun_path, socket_path.c_str(), socket_path.size() + 1);
    if (::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
      throw std::runtime_error("socket bind failed");

    unitree::robot::ChannelFactory::Instance()->Init(domain, interface_name);
    unitree::robot::a2::SportClient client;
    client.SetTimeout(1.0F);
    client.Init();
    std::map<std::string, std::string> state;
    const int state_result = client.GetState(state);
    if (state_result != 0) throw std::runtime_error("GetState failed: " + std::to_string(state_result));
    std::cout << "[a2_sport_backend] ready, fsm=" << state["fsm_name"] << std::endl;

    bool moving = false;
    auto last_command = std::chrono::steady_clock::now();
    while (running.load()) {
      pollfd pfd{fd, POLLIN, 0};
      const int ready = ::poll(&pfd, 1, 20);
      if (ready > 0 && (pfd.revents & POLLIN)) {
        u_robot_a2_driver::SportCommandPacket packet;
        const auto count = ::recv(fd, &packet, sizeof(packet), 0);
        if (count != static_cast<ssize_t>(sizeof(packet)) ||
          packet.magic != u_robot_a2_driver::kSportCommandMagic) continue;
        last_command = std::chrono::steady_clock::now();
        if (packet.type == u_robot_a2_driver::SportCommandType::move) {
          client.Move(static_cast<float>(packet.vx), static_cast<float>(packet.vy),
            static_cast<float>(packet.wz));
          moving = true;
        } else {
          client.StopMove();
          moving = false;
        }
      }
      if (moving && std::chrono::steady_clock::now() - last_command > std::chrono::milliseconds(300)) {
        client.StopMove();
        moving = false;
        std::cerr << "[a2_sport_backend] watchdog StopMove" << std::endl;
      }
    }
    client.StopMove();
    ::close(fd);
    ::unlink(socket_path.c_str());
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "[a2_sport_backend] fatal: " << error.what() << std::endl;
    if (fd >= 0) ::close(fd);
    if (owns_socket_path) ::unlink(socket_path.c_str());
    return 1;
  }
}
