from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    localization_share = Path(get_package_share_directory("u_robot_localization"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))

    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    localization_params_file = LaunchConfiguration("localization_params_file")
    set_initial_pose = LaunchConfiguration("set_initial_pose")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")

    configured_yaml = RewrittenYaml(
        source_file=localization_params_file,
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "amcl.ros__parameters.set_initial_pose": set_initial_pose,
            "amcl.ros__parameters.initial_pose.x": initial_pose_x,
            "amcl.ros__parameters.initial_pose.y": initial_pose_y,
            "amcl.ros__parameters.initial_pose.yaw": initial_pose_yaw,
        },
        convert_types=True,
    )
    configured_params = ParameterFile(configured_yaml, allow_substs=True)

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Absolute path to the map YAML."),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Activate only the read-only map server and AMCL nodes.",
            ),
            DeclareLaunchArgument(
                "localization_params_file",
                default_value=str(localization_share / "config" / "localization.yaml"),
            ),
            DeclareLaunchArgument(
                "set_initial_pose",
                default_value="false",
                description=(
                    "Use the explicit map-frame initial pose below. Keep false "
                    "unless the robot is placed at a surveyed map pose."
                ),
            ),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(nav2_share / "launch" / "localization_launch.py")
                ),
                launch_arguments={
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": configured_yaml,
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="u_robot_localization",
                executable="initial_pose_adapter_node",
                name="initial_pose_adapter",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="u_robot_localization",
                executable="localization_monitor_node",
                name="localization_monitor",
                output="screen",
                parameters=[configured_params],
            ),
        ]
    )
