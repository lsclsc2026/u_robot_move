#include <gtest/gtest.h>

#include <u_robot_camera_bridge/camera_packet.hpp>

#include <array>

TEST(CameraPacket, AcceptsValidHeader)
{
  u_robot_camera_bridge::CameraPacketHeader header;
  header.payload_size = 1024U;
  EXPECT_TRUE(u_robot_camera_bridge::has_valid_camera_header(header, 2048U));
}

TEST(CameraPacket, RejectsInvalidOrOversizedHeader)
{
  u_robot_camera_bridge::CameraPacketHeader header;
  header.payload_size = 4096U;
  EXPECT_FALSE(u_robot_camera_bridge::has_valid_camera_header(header, 2048U));
  header.payload_size = 100U;
  header.magic = 0U;
  EXPECT_FALSE(u_robot_camera_bridge::has_valid_camera_header(header, 2048U));
}

TEST(CameraPacket, RecognizesCompleteJpeg)
{
  const std::array<std::uint8_t, 6U> jpeg{0xFFU, 0xD8U, 0x01U, 0x02U, 0xFFU, 0xD9U};
  EXPECT_TRUE(u_robot_camera_bridge::looks_like_jpeg(jpeg.data(), jpeg.size()));
  const std::array<std::uint8_t, 4U> invalid{0x00U, 0xD8U, 0xFFU, 0xD9U};
  EXPECT_FALSE(u_robot_camera_bridge::looks_like_jpeg(invalid.data(), invalid.size()));
}
