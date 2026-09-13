#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace u_robot_camera_bridge
{
constexpr std::uint32_t kCameraPacketMagic{0x4132434AU};  // "A2CJ"
constexpr std::uint16_t kCameraPacketVersion{1U};
constexpr std::size_t kDefaultMaximumJpegBytes{4U * 1024U * 1024U};

struct CameraPacketHeader
{
  std::uint32_t magic{kCameraPacketMagic};
  std::uint16_t version{kCameraPacketVersion};
  std::uint16_t header_size{32U};
  std::uint64_t sequence{0U};
  std::uint64_t source_monotonic_ns{0U};
  std::uint32_t payload_size{0U};
  std::uint32_t reserved{0U};
};

inline bool has_valid_camera_header(
  const CameraPacketHeader & header, const std::size_t maximum_payload_size)
{
  return header.magic == kCameraPacketMagic &&
         header.version == kCameraPacketVersion &&
         header.header_size == sizeof(CameraPacketHeader) &&
         header.payload_size > 0U &&
         static_cast<std::size_t>(header.payload_size) <= maximum_payload_size;
}

inline bool looks_like_jpeg(const std::uint8_t * data, const std::size_t size)
{
  return data != nullptr && size >= 4U && data[0] == 0xFFU && data[1] == 0xD8U &&
         data[size - 2U] == 0xFFU && data[size - 1U] == 0xD9U;
}

static_assert(std::is_standard_layout_v<CameraPacketHeader>);
static_assert(std::is_trivially_copyable_v<CameraPacketHeader>);
static_assert(sizeof(CameraPacketHeader) == 32U);
}  // namespace u_robot_camera_bridge
