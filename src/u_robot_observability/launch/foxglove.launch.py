from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_bridge(context, *, package_share):
    def boolean_argument(name):
        text = LaunchConfiguration(name).perform(context)
        normalized = text.strip().lower()
        if normalized not in {
            "true", "false", "1", "0", "yes", "no", "on", "off"
        }:
            raise RuntimeError(f"{name} must be a boolean, got '{text}'")
        return normalized in {"true", "1", "yes", "on"}

    allow_initial_pose = boolean_argument("allow_initial_pose")
    allow_navigation_goal = boolean_argument("allow_navigation_goal")
    allow_patrol = boolean_argument("allow_patrol")

    parameters = [
        str(package_share / "config" / "foxglove_read_only.yaml"),
        {
            "address": ParameterValue(
                LaunchConfiguration("address"), value_type=str
            ),
            "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
        },
    ]
    client_topics = ["^/operator/audio/(speak|stop)$"]
    if allow_initial_pose:
        client_topics.append("^/operator/initialpose$")
    if allow_navigation_goal:
        client_topics.append("^/operator/goal_pose$")
        client_topics.append("^/move_base_simple/goal$")
        client_topics.append("^/operator/cancel_navigation$")
    if allow_patrol:
        client_topics.append("^/operator/waypoint$")
        client_topics.append("^/operator/patrol_command$")
    # Browser writes remain restricted to operator intents. Audio parameters and
    # its explicit output gate are exposed, but no velocity/hardware-control API is.
    parameters.append(
        {
            "client_topic_whitelist": client_topics,
            "service_whitelist": ["^/audio_bridge/(enable_output|enable_voice_service)$"],
            "param_whitelist": [
                r"^/audio_bridge\.(volume|loop_message|interval_sec|loop_enabled|repeat_count|speaker_id|minimum_speech_guard_sec)$"
            ],
            "capabilities": [
                "clientPublish",
                "parameters",
                "parametersSubscribe",
                "services",
                "connectionGraph",
                "assets",
            ],
        }
    )

    return [
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name=LaunchConfiguration("node_name"),
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("u_robot_observability"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "address",
                default_value="127.0.0.1",
                description="Bridge bind address; use SSH forwarding with loopback.",
            ),
            DeclareLaunchArgument("port", default_value="9000"),
            DeclareLaunchArgument("node_name", default_value="foxglove_bridge"),
            DeclareLaunchArgument(
                "allow_initial_pose",
                default_value="false",
                description=(
                    "Allow only /operator/initialpose publication from Foxglove. "
                    "The localization adapter adds safe covariance before AMCL."
                ),
            ),
            DeclareLaunchArgument(
                "allow_navigation_goal",
                default_value="false",
                description=(
                    "Allow guarded navigation-goal intent publication on the "
                    "operator topic and the standard Foxglove/RViz topic."
                ),
            ),
            DeclareLaunchArgument(
                "allow_patrol",
                default_value="false",
                description=(
                    "Allow only guarded waypoint and patrol-command intents "
                    "from Foxglove."
                ),
            ),
            OpaqueFunction(
                function=_launch_bridge,
                kwargs={"package_share": package_share},
            ),
        ]
    )
