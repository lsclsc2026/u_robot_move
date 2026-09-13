#include "u_robot_localization/localization_health.hpp"

#include <gtest/gtest.h>

#include <array>
#include <limits>

namespace u_robot_localization
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST(LocalizationHealth, ComputesConservativePlanarAndYawDeviation)
{
  std::array<double, 36> covariance{};
  covariance[0] = 0.04;
  covariance[7] = 0.09;
  covariance[35] = 0.16;

  const auto metrics = covariance_metrics(covariance);

  EXPECT_TRUE(metrics.valid);
  EXPECT_DOUBLE_EQ(metrics.position_stddev, 0.3);
  EXPECT_DOUBLE_EQ(metrics.yaw_stddev, 0.4);
}

TEST(LocalizationHealth, RejectsNonFiniteOrNegativeVariance)
{
  std::array<double, 36> covariance{};
  covariance[7] = -0.1;
  EXPECT_FALSE(covariance_metrics(covariance).valid);

  covariance[7] = 0.0;
  covariance[35] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(covariance_metrics(covariance).valid);
}

TEST(LocalizationHealth, MeasuresWrappedYawAndPlanarDistance)
{
  EXPECT_DOUBLE_EQ(planar_distance(1.0, 2.0, 4.0, 6.0), 5.0);
  EXPECT_NEAR(
    angular_distance(179.0 * kPi / 180.0, -179.0 * kPi / 180.0),
    2.0 * kPi / 180.0, 1.0e-12);
}

}  // namespace u_robot_localization
