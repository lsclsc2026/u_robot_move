#pragma once

#include <array>
#include <vector>

namespace u_robot_state_bridge
{

using Covariance = std::array<double, 36>;

[[nodiscard]] bool covariance_is_zero(const Covariance & covariance);
void set_covariance_diagonal(Covariance & covariance, const std::vector<double> & diagonal);

}  // namespace u_robot_state_bridge

