#include "u_robot_state_bridge/odometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace u_robot_state_bridge
{

bool covariance_is_zero(const Covariance & covariance)
{
  return std::all_of(covariance.begin(), covariance.end(), [](const double value) {
    return std::abs(value) < 1.0e-12;
  });
}

void set_covariance_diagonal(Covariance & covariance, const std::vector<double> & diagonal)
{
  if (diagonal.size() != 6U) {
    throw std::invalid_argument("covariance diagonal must contain six values");
  }
  if (std::any_of(diagonal.begin(), diagonal.end(), [](const double value) {
      return !std::isfinite(value) || value < 0.0;
    }))
  {
    throw std::invalid_argument("covariance values must be finite and non-negative");
  }

  covariance.fill(0.0);
  for (std::size_t index = 0; index < diagonal.size(); ++index) {
    covariance[index * 6U + index] = diagonal[index];
  }
}

}  // namespace u_robot_state_bridge

