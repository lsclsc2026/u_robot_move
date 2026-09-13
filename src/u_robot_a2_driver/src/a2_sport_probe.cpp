#include <unitree/robot/a2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
std::atomic_bool stop_requested{false};

void signal_handler(int)
{
  stop_requested.store(true);
}

double parse_number(const char * text, const char * name)
{
  std::size_t consumed = 0;
  const std::string value{text};
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument(std::string{name} + " must be a finite number");
  }
  return parsed;
}

void print_usage(const char * program)
{
  std::cerr
    << "Read-only: " << program << " <network-interface>\n"
    << "Motion:    " << program
    << " <network-interface> --move <vx> <seconds> --confirm-motion\n"
    << "Limits: |vx| <= 0.20 m/s and 0 < seconds <= 2.0\n";
}

class StopGuard
{
public:
  explicit StopGuard(unitree::robot::a2::SportClient & client) : client_(client) {}
  void arm() {armed_ = true;}
  void stop_now()
  {
    if (armed_) {
      const int result = client_.StopMove();
      std::cerr << "StopMove result: " << result << '\n';
      armed_ = false;
    }
  }
  ~StopGuard()
  {
    try {
      stop_now();
    } catch (...) {
      std::cerr << "StopMove threw during cleanup\n";
    }
  }

private:
  unitree::robot::a2::SportClient & client_;
  bool armed_{false};
};
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2 && argc != 6) {
    print_usage(argv[0]);
    return 2;
  }

  const std::string interface_name{argv[1]};
  const bool motion_requested = argc == 6;
  double vx = 0.0;
  double duration_seconds = 0.0;

  try {
    if (motion_requested) {
      if (std::string{argv[2]} != "--move" || std::string{argv[5]} != "--confirm-motion") {
        throw std::invalid_argument("motion requires --move <vx> <seconds> --confirm-motion");
      }
      vx = parse_number(argv[3], "vx");
      duration_seconds = parse_number(argv[4], "seconds");
      if (std::abs(vx) > 0.20 || duration_seconds <= 0.0 || duration_seconds > 2.0) {
        throw std::invalid_argument("motion is outside the probe safety limits");
      }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    unitree::robot::ChannelFactory::Instance()->Init(0, interface_name);
    unitree::robot::a2::SportClient client;
    client.SetTimeout(2.0F);
    client.Init();

    std::map<std::string, std::string> state;
    const int state_result = client.GetState(state);
    std::cout << "GetState result: " << state_result << '\n';
    for (const char * key : {"fsm_id", "fsm_name", "speed_level", "process_state",
      "auto_recovery_switch"})
    {
      std::cout << key << ": " << state[key] << '\n';
    }
    if (state_result != 0) {
      std::cerr << "GetState failed; refusing motion\n";
      return 1;
    }
    if (!motion_requested) {
      std::cout << "Read-only probe complete; no motion command was sent.\n";
      return 0;
    }

    StopGuard stop_guard{client};
    stop_guard.arm();
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(duration_seconds));
    std::cerr << "Sending Move(vx=" << vx << ", vy=0, wz=0) for at most "
              << duration_seconds << " seconds\n";
    const int move_result = client.Move(static_cast<float>(vx), 0.0F, 0.0F);
    std::cerr << "Move result: " << move_result << '\n';

    while (!stop_requested.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    stop_guard.stop_now();
    return move_result == 0 ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "Error: " << error.what() << '\n';
    print_usage(argv[0]);
    return 2;
  }
}
