"""Launch the independent manager foundation without robot outputs."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Return a standalone manager launch description."""
    return LaunchDescription([
        Node(package='auto_mode_manager', executable='auto_mode_manager_node',
             name='auto_mode_manager', output='screen'),
    ])
