from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from u_robot_bringup.session_guard import acquire_operator_session


def generate_launch_description() -> LaunchDescription:
    nav_package = Path(get_package_share_directory("u_robot_navigation"))
    nav2_package = Path(get_package_share_directory("nav2_bringup"))
    localization_package = Path(get_package_share_directory("u_robot_localization"))
    perception_package = Path(get_package_share_directory("u_robot_perception"))
    bringup_package = Path(get_package_share_directory("u_robot_bringup"))
    camera_package = Path(get_package_share_directory("u_robot_camera_bridge"))
    observability_package = Path(get_package_share_directory("u_robot_observability"))
    patrol_package = Path(get_package_share_directory("u_robot_patrol"))

    params_file = str(nav_package / "config" / "nav2_params.yaml")
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")
    dry_run = LaunchConfiguration("dry_run")
    set_initial_pose = LaunchConfiguration("set_initial_pose")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    front_camera = LaunchConfiguration("front_camera")
    front_camera_rate_hz = LaunchConfiguration("front_camera_rate_hz")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")
    waypoints_file = LaunchConfiguration("waypoints_file")

    session_guard = OpaqueFunction(
        function=acquire_operator_session,
        kwargs={"session": "navigation"},
    )

    workload = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bringup_package / "launch" / "base.launch.py")
                ),
                launch_arguments={
                    "dry_run": dry_run,
                    "publish_lidar_alias": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(camera_package / "launch" / "front_camera.launch.py")
                ),
                condition=IfCondition(front_camera),
                launch_arguments={"publish_rate_hz": front_camera_rate_hz}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(perception_package / "launch" / "lidar_navigation.launch.py")
                ),
                launch_arguments={
                    "params_file": str(
                        perception_package / "config" / "pointcloud_to_mapping_scan.yaml"
                    )
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(localization_package / "launch" / "localization_core.launch.py")
                ),
                launch_arguments={
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "set_initial_pose": set_initial_pose,
                    "initial_pose_x": initial_pose_x,
                    "initial_pose_y": initial_pose_y,
                    "initial_pose_yaw": initial_pose_yaw,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(nav2_package / "launch" / "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="u_robot_navigation",
                executable="velocity_deadband_adapter_node",
                name="velocity_deadband_adapter",
                output="screen",
                parameters=[
                    str(nav_package / "config" / "velocity_deadband_adapter.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[
                    str(nav_package / "config" / "collision_monitor.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="u_robot_navigation",
                executable="operator_goal_adapter_node",
                name="operator_goal_adapter",
                output="screen",
                parameters=[
                    str(nav_package / "config" / "operator_goal_adapter.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="u_robot_patrol",
                executable="waypoint_patrol_node",
                name="waypoint_patrol",
                output="screen",
                parameters=[
                    str(patrol_package / "config" / "patrol.yaml"),
                    {
                        "use_sim_time": use_sim_time,
                        "map_yaml": ParameterValue(map_yaml, value_type=str),
                        "waypoint_file": ParameterValue(
                            waypoints_file, value_type=str
                        ),
                        "reverse_behavior_tree": str(
                            nav_package
                            / "behavior_trees"
                            / "navigate_reverse_with_local_clear_retry.xml"
                        ),
                    },
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_collision_monitor",
                        output="screen",
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"autostart": autostart},
                            {"node_names": ["collision_monitor"]},
                        ],
                    )
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(observability_package / "launch" / "foxglove.launch.py")
                ),
                condition=IfCondition(foxglove),
                launch_arguments={
                    "address": foxglove_address,
                    "port": foxglove_port,
                    "node_name": "navigation_foxglove_bridge",
                    "allow_initial_pose": "true",
                    "allow_navigation_goal": "true",
                    "allow_patrol": "true",
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            UnsetEnvironmentVariable("CYCLONEDDS_URI"),
            DeclareLaunchArgument("map", description="Absolute path to a qualified map YAML."),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Activate Nav2 lifecycle nodes; hardware control remains separately gated.",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="Suppress every Unitree sport request when true.",
            ),
            DeclareLaunchArgument("set_initial_pose", default_value="false"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("front_camera", default_value="true"),
            DeclareLaunchArgument("front_camera_rate_hz", default_value="15.0"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_address", default_value="127.0.0.1"),
            DeclareLaunchArgument("foxglove_port", default_value="9000"),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value="",
                description=(
                    "Optional patrol YAML. Empty derives a separate waypoint file "
                    "from the current map name."
                ),
            ),
            session_guard,
            workload,
        ]
    )
