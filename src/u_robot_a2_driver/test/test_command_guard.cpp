#include "u_robot_a2_driver/command_guard.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>

using u_robot_a2_driver::CommandGuard;
using u_robot_a2_driver::VelocityCommand;

TEST(CommandGuard, ClampsEveryAxis)
{
  const CommandGuard guard(0.5, 0.3, 0.6, std::chrono::milliseconds(250));
  const auto result = guard.clamp(VelocityCommand{1.0, -1.0, 2.0});
  EXPECT_DOUBLE_EQ(result.vx, 0.5);
  EXPECT_DOUBLE_EQ(result.vy, -0.3);
  EXPECT_DOUBLE_EQ(result.wz, 0.6);
}

TEST(CommandGuard, RejectsNonFiniteInput)
{
  const CommandGuard guard(0.5, 0.3, 0.6, std::chrono::milliseconds(250));
  EXPECT_FALSE(guard.is_finite(VelocityCommand{
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}));
  EXPECT_TRUE(guard.is_finite(VelocityCommand{0.1, -0.1, 0.2}));
}

TEST(CommandGuard, DetectsTimeout)
{
  const CommandGuard guard(0.5, 0.3, 0.6, std::chrono::milliseconds(250));
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(guard.is_stale(start + std::chrono::milliseconds(250), start));
  EXPECT_TRUE(guard.is_stale(start + std::chrono::milliseconds(251), start));
}

