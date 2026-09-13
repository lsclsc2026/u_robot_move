#pragma once
#include <cstdint>
namespace u_robot_a2_driver {
constexpr std::uint32_t kSportCommandMagic = 0x41325350U;
enum class SportCommandType : std::uint32_t {stop = 0, move = 1};
struct SportCommandPacket {
  std::uint32_t magic{kSportCommandMagic};
  SportCommandType type{SportCommandType::stop};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};
}
