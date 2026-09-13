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
from launch_ros.parameter_descriptions import ParameterValue
from u_robot_bringup.session_guard import acquire_operator_session


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("u_robot_bringup"))
    description_share = Path(get_package_share_directory("u_robot_description"))
    observability_share = Path(get_package_share_directory("u_robot_observability"))
    parameters = bringup_share / "config" / "state_bridge.yaml"
    state_bridge_share = Path(get_package_share_directory("u_robot_state_bridge"))
    camera_bridge_share = Path(get_package_share_directory("u_robot_camera_bridge"))

    joint_source_topic = LaunchConfiguration("joint_source_topic")
    joint_ipc_channel = LaunchConfiguration("joint_ipc_channel")
    native_dds_domain_id = LaunchConfiguration("native_dds_domain_id")
    native_dds_interface = LaunchConfiguration("native_dds_interface")
    odometry_source_topic = LaunchConfiguration("odometry_source_topic")
    publish_base_tf = LaunchConfiguration("publish_base_tf")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")
    front_camera = LaunchConfiguration("front_camera")
    front_camera_rate_hz = LaunchConfiguration("front_camera_rate_hz")

    session_guard = OpaqueFunction(
        function=acquire_operator_session,
        kwargs={"session": "live_visualization"},
    )

    workload = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(state_bridge_share / "launch" / "live_joint_state.launch.py")
                ),
                launch_arguments={
                    "parameters_file": str(parameters),
                    "source_topic": joint_source_topic,
                    "ipc_channel": joint_ipc_channel,
                    "native_dds_domain_id": native_dds_domain_id,
                    "native_dds_interface": native_dds_interface,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(description_share / "launch" / "description.launch.py")
                ),
                launch_arguments={"publish_lidar_alias": "false"}.items(),
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
            Node(
                package="u_robot_state_bridge",
                executable="state_bridge_node",
                name="state_bridge",
                output="screen",
                parameters=[
                    str(parameters),
                    {
                        "source_topic": ParameterValue(
                            odometry_source_topic, value_type=str
                        ),
                        "publish_tf": ParameterValue(publish_base_tf, value_type=bool),
                    },
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(observability_share / "launch" / "foxglove.launch.py")
                ),
                launch_arguments={
                    "address": foxglove_address,
                    "port": foxglove_port,
                    "node_name": "live_visualization_foxglove_bridge",
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            # The long-lived container currently inherits a host-only CycloneDDS
            # URI that does not exist inside the container. Automatic interface
            # discovery is validated on the A2 host and avoids a launch failure.
            UnsetEnvironmentVariable("CYCLONEDDS_URI"),
            DeclareLaunchArgument(
                "joint_source_topic", default_value="rt/lf/lowstate"
            ),
            DeclareLaunchArgument(
                "joint_ipc_channel", default_value="u_robot_a2_lowstate"
            ),
            DeclareLaunchArgument("native_dds_domain_id", default_value="0"),
            DeclareLaunchArgument("native_dds_interface", default_value="eth0"),
            DeclareLaunchArgument("odometry_source_topic", default_value="/dog_odom"),
            DeclareLaunchArgument(
                "publish_base_tf",
                default_value="true",
                description="Publish odom -> base_link from the read-only odometry bridge.",
            ),
            DeclareLaunchArgument("foxglove_address", default_value="127.0.0.1"),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="9000",
                description=(
                    "Standalone live-state endpoint. Do not run beside another "
                    "Foxglove launch using the same port."
                ),
            ),
            DeclareLaunchArgument(
                "front_camera",
                default_value="true",
                description="Publish the read-only A2 front-camera JPEG stream.",
            ),
            DeclareLaunchArgument(
                "front_camera_rate_hz", default_value="15.0"
            ),
            session_guard,
            workload,
        ]
    )
