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
    mapping_share = Path(get_package_share_directory("u_robot_mapping"))
    description_share = Path(get_package_share_directory("u_robot_description"))
    perception_share = Path(get_package_share_directory("u_robot_perception"))
    observability_share = Path(get_package_share_directory("u_robot_observability"))
    bringup_share = Path(get_package_share_directory("u_robot_bringup"))
    state_bridge_share = Path(get_package_share_directory("u_robot_state_bridge"))
    camera_bridge_share = Path(get_package_share_directory("u_robot_camera_bridge"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    mapping_pointcloud_topic = "/mapping/points_time_aligned"
    rviz = LaunchConfiguration("rviz")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")
    front_camera = LaunchConfiguration("front_camera")
    front_camera_rate_hz = LaunchConfiguration("front_camera_rate_hz")

    session_guard = OpaqueFunction(
        function=acquire_operator_session,
        kwargs={"session": "manual_mapping"},
    )

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
                parameters=[str(bringup_share / "config" / "state_bridge.yaml")],
            ),
            Node(
                package="u_robot_perception",
                executable="cloud_timestamp_normalizer_node",
                name="mapping_cloud_timestamp_normalizer",
                output="screen",
                parameters=[
                    {
                        "input_topic": pointcloud_topic,
                        "output_topic": mapping_pointcloud_topic,
                        "minimum_age_to_restamp": 0.20,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(state_bridge_share / "launch" / "live_joint_state.launch.py")
                ),
                launch_arguments={
                    "parameters_file": str(
                        bringup_share / "config" / "state_bridge.yaml"
                    )
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(camera_bridge_share / "launch" / "front_camera.launch.py")
                ),
                condition=IfCondition(front_camera),
                launch_arguments={
                    "publish_rate_hz": front_camera_rate_hz,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(perception_share / "launch" / "lidar_navigation.launch.py")
                ),
                launch_arguments={
                    "pointcloud_topic": mapping_pointcloud_topic,
                    "params_file": str(
                        perception_share / "config" / "pointcloud_to_mapping_scan.yaml"
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(mapping_share / "launch" / "mapping.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(observability_share / "launch" / "foxglove.launch.py")
                ),
                condition=IfCondition(foxglove),
                launch_arguments={
                    "address": foxglove_address,
                    "port": foxglove_port,
                    "node_name": "mapping_foxglove_bridge",
                }.items(),
            ),
            Node(
                condition=IfCondition(rviz),
                package="rviz2",
                executable="rviz2",
                name="mapping_rviz",
                output="screen",
                arguments=["-d", str(mapping_share / "rviz" / "mapping.rviz")],
            ),
        ]
    )

    return LaunchDescription(
        [
            # The A2 login environment may reference a host-side CycloneDDS XML
            # path that is not mounted into Docker. Let ROS discover the validated
            # host-network interface instead of failing every rclcpp node.
            UnsetEnvironmentVariable("CYCLONEDDS_URI"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/unitree/slam_lidar/points",
                description="Validated A2 fused front/rear navigation cloud.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Open the mapping RViz view when a graphical display is available.",
            ),
            DeclareLaunchArgument(
                "foxglove",
                default_value="true",
                description="Start the read-only Foxglove WebSocket bridge.",
            ),
            DeclareLaunchArgument(
                "foxglove_address",
                default_value="127.0.0.1",
                description="Use loopback with VS Code or SSH port forwarding.",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="9000",
                description=(
                    "Single operator Foxglove endpoint for the navigation and "
                    "live A2-Pro 3D panels."
                ),
            ),
            DeclareLaunchArgument(
                "front_camera",
                default_value="true",
                description="Publish the read-only A2 front-camera JPEG stream.",
            ),
            DeclareLaunchArgument(
                "front_camera_rate_hz",
                default_value="15.0",
                description="Maximum front-camera request rate.",
            ),
            session_guard,
            workload,
        ]
    )
