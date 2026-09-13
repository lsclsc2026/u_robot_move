from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("u_robot_perception"))
    default_parameters = package_share / "config" / "pointcloud_to_scan.yaml"
    default_filter_parameters = (
        package_share / "config" / "navigation_cloud_filter.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/unitree/slam_lidar/points",
                description="A2 fused navigation PointCloud2 topic.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_parameters),
                description="PointCloud2-to-LaserScan parameter profile.",
            ),
            DeclareLaunchArgument(
                "filter_params_file",
                default_value=str(default_filter_parameters),
                description="Raw-to-navigation PointCloud2 filter profile.",
            ),
            Node(
                package="u_robot_perception",
                executable="navigation_cloud_filter_node",
                name="navigation_cloud_filter",
                output="screen",
                parameters=[LaunchConfiguration("filter_params_file")],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
                remappings=[
                    ("cloud_in", LaunchConfiguration("pointcloud_topic")),
                    ("scan", "/scan"),
                ],
            ),
        ]
    )
