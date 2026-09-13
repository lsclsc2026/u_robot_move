#include "u_robot_state_bridge/odometry_utils.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

using u_robot_state_bridge::Covariance;
using u_robot_state_bridge::covariance_is_zero;
using u_robot_state_bridge::set_covariance_diagonal;

TEST(OdometryUtils, DetectsZeroCovariance)
{
  Covariance covariance{};
  EXPECT_TRUE(covariance_is_zero(covariance));
  covariance[7] = 0.1;
  EXPECT_FALSE(covariance_is_zero(covariance));
}

TEST(OdometryUtils, PopulatesOnlyDiagonal)
{
  Covariance covariance{};
  set_covariance_diagonal(covariance, {1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
  EXPECT_DOUBLE_EQ(covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(covariance[7], 2.0);
  EXPECT_DOUBLE_EQ(covariance[14], 3.0);
  EXPECT_DOUBLE_EQ(covariance[21], 4.0);
  EXPECT_DOUBLE_EQ(covariance[28], 5.0);
  EXPECT_DOUBLE_EQ(covariance[35], 6.0);
  EXPECT_DOUBLE_EQ(covariance[1], 0.0);
}

TEST(OdometryUtils, RejectsWrongDiagonalSize)
{
  Covariance covariance{};
  EXPECT_THROW(set_covariance_diagonal(covariance, {1.0, 2.0}), std::invalid_argument);
}
