#ifndef U_ROBOT_STATE_BRIDGE__NATIVE_JOINT_PACKET_HPP_
#define U_ROBOT_STATE_BRIDGE__NATIVE_JOINT_PACKET_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace u_robot_state_bridge
{

constexpr std::uint32_t kNativeJointPacketMagic = 0x41324A53U;  // "A2JS"
constexpr std::uint16_t kNativeJointPacketVersion = 1U;
constexpr std::size_t kA2LegJointCount = 12U;

// Process-local wire format between the read-only SDK2 reader and the ROS
// adapter. Both processes run in the same container and architecture.
struct NativeJointPacket
{
  std::uint32_t magic{kNativeJointPacketMagic};
  std::uint16_t version{kNativeJointPacketVersion};
  std::uint16_t joint_count{static_cast<std::uint16_t>(kA2LegJointCount)};
  std::uint32_t source_tick{0U};
  std::uint32_t reserved{0U};
  std::uint64_t sequence{0U};
  std::array<double, kA2LegJointCount> position{};
  std::array<double, kA2LegJointCount> velocity{};
  std::array<double, kA2LegJointCount> effort{};
};

static_assert(std::is_standard_layout_v<NativeJointPacket>);
static_assert(std::is_trivially_copyable_v<NativeJointPacket>);

constexpr bool has_valid_native_joint_header(const NativeJointPacket & packet)
{
  return packet.magic == kNativeJointPacketMagic &&
         packet.version == kNativeJointPacketVersion &&
         packet.joint_count == static_cast<std::uint16_t>(kA2LegJointCount);
}

}  // namespace u_robot_state_bridge

#endif  // U_ROBOT_STATE_BRIDGE__NATIVE_JOINT_PACKET_HPP_
