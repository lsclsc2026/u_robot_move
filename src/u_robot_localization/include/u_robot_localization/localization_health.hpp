#pragma once

#include <array>

namespace u_robot_localization
{

struct CovarianceMetrics
{
  bool valid;
  double position_stddev;
  double yaw_stddev;
};

CovarianceMetrics covariance_metrics(const std::array<double, 36> & covariance);
double planar_distance(double first_x, double first_y, double second_x, double second_y);
double angular_distance(double first_yaw, double second_yaw);

}  // namespace u_robot_localization
