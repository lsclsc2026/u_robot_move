from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("u_robot_bringup"))
    default_parameters = package_share / "config" / "a2_driver.yaml"
    state_bridge_parameters = package_share / "config" / "state_bridge.yaml"
    description_share = Path(get_package_share_directory("u_robot_description"))
    state_bridge_share = Path(get_package_share_directory("u_robot_state_bridge"))

    dry_run = LaunchConfiguration("dry_run")
    publish_lidar_alias = LaunchConfiguration("publish_lidar_alias")
    native_dds_domain_id = LaunchConfiguration("native_dds_domain_id")
    native_dds_interface = LaunchConfiguration("native_dds_interface")
    sport_backend = (
        Path(get_package_prefix("u_robot_a2_driver"))
        / "lib" / "u_robot_a2_driver" / "a2_sport_backend"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="Suppress every Unitree sport request when true.",
            ),
            DeclareLaunchArgument(
                "publish_lidar_alias",
                default_value="false",
                description="Publish the validated front_lidar_link -> hesai_lidar alias.",
            ),
            DeclareLaunchArgument("native_dds_domain_id", default_value="0"),
            DeclareLaunchArgument("native_dds_interface", default_value="eth0"),
            ExecuteProcess(
                cmd=[
                    str(sport_backend),
                    "--network-interface", native_dds_interface,
                    "--domain-id", native_dds_domain_id,
                    "--socket-path", "/tmp/u_robot_a2_sport.sock",
                ],
                condition=UnlessCondition(dry_run),
                output="screen",
                # A control-lock conflict must require an explicit restart.
                # Auto-respawn could silently enable navigation after teleop
                # releases its lock, which is not an operator mode switch.
                respawn=False,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(state_bridge_share / "launch" / "live_joint_state.launch.py")
                ),
                launch_arguments={
                    "parameters_file": str(state_bridge_parameters),
                    "native_dds_domain_id": native_dds_domain_id,
                    "native_dds_interface": native_dds_interface,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(description_share / "launch" / "description.launch.py")
                ),
                launch_arguments={
                    "publish_lidar_alias": publish_lidar_alias
                }.items(),
            ),
            Node(
                package="u_robot_state_bridge",
                executable="state_bridge_node",
                name="state_bridge",
                output="screen",
                parameters=[str(state_bridge_parameters)],
            ),
            Node(
                package="u_robot_a2_driver",
                executable="a2_driver_node",
                name="a2_driver",
                output="screen",
                parameters=[str(default_parameters), {"dry_run": dry_run}],
            ),
        ]
    )
