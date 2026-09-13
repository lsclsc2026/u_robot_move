from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("u_robot_camera_bridge"))
    parameters_file = LaunchConfiguration("parameters_file")
    multicast_group = LaunchConfiguration("multicast_group")
    udp_port = LaunchConfiguration("udp_port")
    network_interface = LaunchConfiguration("network_interface")
    output_topic = LaunchConfiguration("output_topic")
    frame_id = LaunchConfiguration("frame_id")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")

    return LaunchDescription(
        [
            # The A2 login shell exports a host-side CycloneDDS XML path which is
            # intentionally not mounted into the development container.
            UnsetEnvironmentVariable("CYCLONEDDS_URI"),
            DeclareLaunchArgument(
                "parameters_file",
                default_value=str(package_share / "config" / "front_camera.yaml"),
            ),
            DeclareLaunchArgument(
                "multicast_group", default_value="230.1.1.1"
            ),
            DeclareLaunchArgument("udp_port", default_value="1720"),
            DeclareLaunchArgument(
                "network_interface", default_value="eth0"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/camera/front/image/compressed"
            ),
            DeclareLaunchArgument("frame_id", default_value="camera_link"),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="15.0",
                description="Maximum JPEG publications per second.",
            ),
            Node(
                package="u_robot_camera_bridge",
                executable="front_camera_bridge_node",
                name="front_camera_bridge",
                output="screen",
                parameters=[
                    parameters_file,
                    {
                        "multicast_group": ParameterValue(
                            multicast_group, value_type=str
                        ),
                        "udp_port": ParameterValue(udp_port, value_type=int),
                        "network_interface": ParameterValue(
                            network_interface, value_type=str
                        ),
                        "output_topic": ParameterValue(output_topic, value_type=str),
                        "frame_id": ParameterValue(frame_id, value_type=str),
                        "publish_rate_hz": ParameterValue(
                            publish_rate_hz, value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
