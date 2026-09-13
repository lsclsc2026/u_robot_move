from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    native_reader = (
        Path(get_package_prefix("u_robot_state_bridge"))
        / "lib"
        / "u_robot_state_bridge"
        / "native_low_state_reader"
    )

    parameters_file = LaunchConfiguration("parameters_file")
    source_topic = LaunchConfiguration("source_topic")
    ipc_channel = LaunchConfiguration("ipc_channel")
    native_dds_domain_id = LaunchConfiguration("native_dds_domain_id")
    native_dds_interface = LaunchConfiguration("native_dds_interface")

    return LaunchDescription(
        [
            DeclareLaunchArgument("parameters_file"),
            DeclareLaunchArgument("source_topic", default_value="rt/lf/lowstate"),
            DeclareLaunchArgument(
                "ipc_channel", default_value="u_robot_a2_lowstate"
            ),
            DeclareLaunchArgument("native_dds_domain_id", default_value="0"),
            DeclareLaunchArgument("native_dds_interface", default_value="eth0"),
            ExecuteProcess(
                cmd=[
                    str(native_reader),
                    "--source-topic",
                    source_topic,
                    "--network-interface",
                    native_dds_interface,
                    "--domain-id",
                    native_dds_domain_id,
                    "--ipc-channel",
                    ipc_channel,
                ],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="u_robot_state_bridge",
                executable="joint_state_bridge_node",
                name="joint_state_bridge",
                output="screen",
                parameters=[
                    parameters_file,
                    {
                        "source_topic": ParameterValue(source_topic, value_type=str),
                        "ipc_channel": ParameterValue(ipc_channel, value_type=str),
                    },
                ],
            ),
        ]
    )
