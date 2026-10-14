"""Start a disabled coordinator by default; physical profiles remain gated."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='disabled'),
        DeclareLaunchArgument('profile_file', default_value=''),
        Node(package='auto_mode_manager', executable='auto_mode_manager_node',
             name='auto_mode_manager', output='screen', parameters=[{
                 'mode': LaunchConfiguration('mode'), 'profile_file': LaunchConfiguration('profile_file')}]),
    ])
