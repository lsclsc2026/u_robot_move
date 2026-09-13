#include "u_robot_localization/localization_health.hpp"

#include <algorithm>
#include <cmath>

namespace u_robot_localization
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

CovarianceMetrics covariance_metrics(const std::array<double, 36> & covariance)
{
  const double x_variance = covariance[0];
  const double y_variance = covariance[7];
  const double yaw_variance = covariance[35];
  const bool valid = std::isfinite(x_variance) && std::isfinite(y_variance) &&
    std::isfinite(yaw_variance) && x_variance >= 0.0 && y_variance >= 0.0 &&
    yaw_variance >= 0.0;
  if (!valid) {
    return {false, 0.0, 0.0};
  }
  return {
    true,
    std::sqrt(std::max(x_variance, y_variance)),
    std::sqrt(yaw_variance),
  };
}

double planar_distance(
  const double first_x, const double first_y, const double second_x,
  const double second_y)
{
  return std::hypot(second_x - first_x, second_y - first_y);
}

double angular_distance(const double first_yaw, const double second_yaw)
{
  return std::abs(std::remainder(second_yaw - first_yaw, 2.0 * kPi));
}

}  // namespace u_robot_localization
