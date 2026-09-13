#!/usr/bin/env python3

"""Publish a deterministic A2 standing pose for URDF-only validation."""

from typing import Final

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES: Final = (
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
)

# A symmetric kinematic stance for the official 0.275 m + 0.275 m leg model.
# These are URDF joint angles, not values copied from A2 low-level motor state.
STAND_POSITIONS: Final = (
    0.0,
    0.45,
    -0.90,
    0.0,
    0.45,
    -0.90,
    0.0,
    0.45,
    -0.90,
    0.0,
    0.45,
    -0.90,
)


class A2StandJointPublisher(Node):
    def __init__(self) -> None:
        super().__init__("a2_stand_joint_publisher")
        publish_rate_hz = self.declare_parameter("publish_rate_hz", 20.0).value
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish)
        self.get_logger().info(
            f"Publishing isolated A2 URDF standing pose at {publish_rate_hz:.1f} Hz"
        )

    def _publish(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(STAND_POSITIONS)
        message.velocity = [0.0] * len(JOINT_NAMES)
        message.effort = [0.0] * len(JOINT_NAMES)
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = A2StandJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
