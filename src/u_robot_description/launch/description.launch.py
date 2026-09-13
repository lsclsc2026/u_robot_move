from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("u_robot_description"))
    robot_description = (package_share / "urdf" / "a2.urdf").read_text()
    publish_lidar_alias = LaunchConfiguration("publish_lidar_alias")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_lidar_alias",
                default_value="false",
                description=(
                    "Publish front_lidar_link -> hesai_lidar identity TF. The alias is "
                    "validated for the A2 fused points/points1/points2 stream."
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                condition=IfCondition(publish_lidar_alias),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="hesai_lidar_frame_alias",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "0",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "front_lidar_link",
                    "--child-frame-id", "hesai_lidar",
                ],
                output="screen",
            ),
        ]
    )
