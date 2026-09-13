#pragma once

#include <chrono>

namespace u_robot_a2_driver
{

struct VelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

class CommandGuard
{
public:
  CommandGuard(double max_vx, double max_vy, double max_wz, std::chrono::milliseconds timeout);

  [[nodiscard]] VelocityCommand clamp(const VelocityCommand & command) const;
  [[nodiscard]] bool is_finite(const VelocityCommand & command) const;
  [[nodiscard]] bool is_stale(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point last_command_time) const;

private:
  double max_vx_;
  double max_vy_;
  double max_wz_;
  std::chrono::milliseconds timeout_;
};

}  // namespace u_robot_a2_driver

