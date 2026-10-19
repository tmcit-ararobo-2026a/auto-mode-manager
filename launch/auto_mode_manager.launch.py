"""Start the coordinator with ordinary runtime parameters."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('auto_mode_manager'), 'config', 'manager.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='auto_mode_manager', executable='auto_mode_manager_node',
             name='auto_mode_manager', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
