from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from u_robot_bringup.session_guard import acquire_operator_session


def generate_launch_description() -> LaunchDescription:
    localization_share = Path(get_package_share_directory("u_robot_localization"))
    description_share = Path(get_package_share_directory("u_robot_description"))
    perception_share = Path(get_package_share_directory("u_robot_perception"))
    observability_share = Path(get_package_share_directory("u_robot_observability"))
    bringup_share = Path(get_package_share_directory("u_robot_bringup"))
    state_bridge_share = Path(get_package_share_directory("u_robot_state_bridge"))
    camera_bridge_share = Path(get_package_share_directory("u_robot_camera_bridge"))

    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    set_initial_pose = LaunchConfiguration("set_initial_pose")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")
    front_camera = LaunchConfiguration("front_camera")
    front_camera_rate_hz = LaunchConfiguration("front_camera_rate_hz")
    native_dds_domain_id = LaunchConfiguration("native_dds_domain_id")
    native_dds_interface = LaunchConfiguration("native_dds_interface")
    joint_source_topic = LaunchConfiguration("joint_source_topic")
    joint_ipc_channel = LaunchConfiguration("joint_ipc_channel")

    state_parameters = bringup_share / "config" / "state_bridge.yaml"
    session_guard = OpaqueFunction(
        function=acquire_operator_session,
        kwargs={"session": "static_localization"},
    )

    # This workload is intentionally read-only. It contains no A2 driver, Nav2
    # controller, velocity smoother, collision monitor, or cmd_vel publisher.
    workload = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(description_share / "launch" / "description.launch.py")
                ),
                launch_arguments={"publish_lidar_alias": "true"}.items(),
            ),
            Node(
                package="u_robot_state_bridge",
                executable="state_bridge_node",
                name="state_bridge",
                output="screen",
                parameters=[str(state_parameters)],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(state_bridge_share / "launch" / "live_joint_state.launch.py")
                ),
                launch_arguments={
                    "parameters_file": str(state_parameters),
                    "source_topic": joint_source_topic,
                    "ipc_channel": joint_ipc_channel,
                    "native_dds_domain_id": native_dds_domain_id,
                    "native_dds_interface": native_dds_interface,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(camera_bridge_share / "launch" / "front_camera.launch.py")
                ),
                condition=IfCondition(front_camera),
                launch_arguments={
                    "network_interface": native_dds_interface,
                    "publish_rate_hz": front_camera_rate_hz,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(perception_share / "launch" / "lidar_navigation.launch.py")
                ),
                launch_arguments={
                    "pointcloud_topic": pointcloud_topic,
                    "params_file": str(
                        perception_share / "config" / "pointcloud_to_mapping_scan.yaml"
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(localization_share / "launch" / "localization_core.launch.py")
                ),
                launch_arguments={
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "autostart": "true",
                    "set_initial_pose": set_initial_pose,
                    "initial_pose_x": initial_pose_x,
                    "initial_pose_y": initial_pose_y,
                    "initial_pose_yaw": initial_pose_yaw,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(observability_share / "launch" / "foxglove.launch.py")
                ),
                condition=IfCondition(foxglove),
                launch_arguments={
                    "address": foxglove_address,
                    "port": foxglove_port,
                    "node_name": "localization_foxglove_bridge",
                    "allow_initial_pose": "true",
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
                "pointcloud_topic",
                default_value="/unitree/slam_lidar/points",
            ),
            DeclareLaunchArgument("set_initial_pose", default_value="false"),
            DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("native_dds_domain_id", default_value="0"),
            DeclareLaunchArgument("native_dds_interface", default_value="eth0"),
            DeclareLaunchArgument("joint_source_topic", default_value="rt/lf/lowstate"),
            DeclareLaunchArgument(
                "joint_ipc_channel", default_value="u_robot_a2_lowstate"
            ),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_address", default_value="127.0.0.1"),
            DeclareLaunchArgument("foxglove_port", default_value="9000"),
            DeclareLaunchArgument("front_camera", default_value="true"),
            DeclareLaunchArgument("front_camera_rate_hz", default_value="15.0"),
            session_guard,
            workload,
        ]
    )
