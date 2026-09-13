from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_share = Path(get_package_share_directory("u_robot_description"))
    observability_share = Path(get_package_share_directory("u_robot_observability"))
    robot_description = (description_share / "urdf" / "a2_preview.urdf").read_text()

    domain_id = LaunchConfiguration("domain_id")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "domain_id",
                default_value="42",
                description="Isolated ROS domain; must differ from the live A2 domain.",
            ),
            DeclareLaunchArgument("foxglove_address", default_value="127.0.0.1"),
            DeclareLaunchArgument("foxglove_port", default_value="8766"),
            # The preview must never consume live A2 TF, odometry, or joint state.
            SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="a2_preview_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="u_robot_description",
                executable="a2_stand_joint_publisher.py",
                name="a2_stand_joint_publisher",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(observability_share / "launch" / "foxglove.launch.py")
                ),
                launch_arguments={
                    "address": foxglove_address,
                    "port": foxglove_port,
                }.items(),
            ),
        ]
    )
