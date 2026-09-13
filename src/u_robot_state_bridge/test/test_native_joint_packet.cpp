#include <gtest/gtest.h>

#include <u_robot_state_bridge/native_joint_packet.hpp>

using u_robot_state_bridge::NativeJointPacket;
using u_robot_state_bridge::has_valid_native_joint_header;

TEST(NativeJointPacket, DefaultHeaderIsValid)
{
  EXPECT_TRUE(has_valid_native_joint_header(NativeJointPacket{}));
}

TEST(NativeJointPacket, RejectsProtocolMismatch)
{
  NativeJointPacket packet;
  packet.magic = 0U;
  EXPECT_FALSE(has_valid_native_joint_header(packet));

  packet = NativeJointPacket{};
  packet.version++;
  EXPECT_FALSE(has_valid_native_joint_header(packet));

  packet = NativeJointPacket{};
  packet.joint_count--;
  EXPECT_FALSE(has_valid_native_joint_header(packet));
}
