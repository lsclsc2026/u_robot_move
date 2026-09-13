from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def start(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)
    cmd = [str(Path(get_package_prefix('u_robot_teleop')) / 'lib/u_robot_teleop/teleop_receiver.py'),
           '--pc1-ip', value('pc1_ip'), '--config', value('config'), '--duration', value('duration')]
    for argument, flag in [('dry_run', '--real-control'), ('special_actions', '--special-actions')]:
        text = value(argument).lower()
        if text not in ('true', 'false'):
            raise ValueError(f'{argument} must be true or false')
        if text == ('false' if argument == 'dry_run' else 'true'):
            cmd.append(flag)
    return [ExecuteProcess(cmd=cmd, output='screen', sigterm_timeout='5', sigkill_timeout='3')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pc1_ip', description='Windows LAN IPv4 of BLE sender, required'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('special_actions', default_value='false'),
        DeclareLaunchArgument('duration', default_value='0'),
        DeclareLaunchArgument('config', default_value=str(Path(get_package_share_directory('u_robot_teleop')) / 'config/teleop.yaml')),
        OpaqueFunction(function=start),
    ])
