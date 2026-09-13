from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    observability_share = Path(get_package_share_directory("u_robot_observability"))

    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription(
        [
            UnsetEnvironmentVariable("CYCLONEDDS_URI"),
            DeclareLaunchArgument(
                "map",
                description="Absolute path to the saved occupancy-grid YAML file.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument(
                "foxglove_address",
                default_value="127.0.0.1",
                description="Use loopback with VS Code or SSH port forwarding.",
            ),
            DeclareLaunchArgument("foxglove_port", default_value="9000"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "yaml_filename": ParameterValue(map_yaml, value_type=str),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_view",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["map_server"],
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(observability_share / "launch" / "foxglove.launch.py")
                ),
                condition=IfCondition(foxglove),
                launch_arguments={
                    "address": foxglove_address,
                    "port": foxglove_port,
                    "node_name": "saved_map_foxglove_bridge",
                }.items(),
            ),
        ]
    )
