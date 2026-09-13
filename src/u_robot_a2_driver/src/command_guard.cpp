#include "u_robot_a2_driver/command_guard.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace u_robot_a2_driver
{

CommandGuard::CommandGuard(
  const double max_vx,
  const double max_vy,
  const double max_wz,
  const std::chrono::milliseconds timeout)
: max_vx_(max_vx), max_vy_(max_vy), max_wz_(max_wz), timeout_(timeout)
{
  if (max_vx_ <= 0.0 || max_vy_ <= 0.0 || max_wz_ <= 0.0 || timeout_.count() <= 0) {
    throw std::invalid_argument("velocity limits and command timeout must be positive");
  }
}

VelocityCommand CommandGuard::clamp(const VelocityCommand & command) const
{
  return VelocityCommand{
    std::clamp(command.vx, -max_vx_, max_vx_),
    std::clamp(command.vy, -max_vy_, max_vy_),
    std::clamp(command.wz, -max_wz_, max_wz_)};
}

bool CommandGuard::is_finite(const VelocityCommand & command) const
{
  return std::isfinite(command.vx) && std::isfinite(command.vy) && std::isfinite(command.wz);
}

bool CommandGuard::is_stale(
  const std::chrono::steady_clock::time_point now,
  const std::chrono::steady_clock::time_point last_command_time) const
{
  return now - last_command_time > timeout_;
}

}  // namespace u_robot_a2_driver

