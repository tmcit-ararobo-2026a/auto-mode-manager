"""Start the communication-only loopback test stage; no production fixtures are included."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def test_nodes(context):
    """Validate the eligible stage and test domain before starting either node."""
    stage = LaunchConfiguration('stage').perform(context)
    domain = int(LaunchConfiguration('domain_id').perform(context))
    if stage != 'communication':
        raise ValueError('This foundation provides communication only; other stages require their completed tasks')
    if domain < 100 or domain > 232:
        raise ValueError('Test domain must be in [100,232]')
    command_port = int(LaunchConfiguration('command_port').perform(context))
    operation_port = int(LaunchConfiguration('operation_port').perform(context))
    return [
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain)),
        Node(package='robot_port', executable='udp_node', name='stm32_test_udp',
             output='screen', parameters=[{
                 'local_address': '127.0.0.1', 'peer_address': '127.0.0.2',
                 'command_port': command_port, 'operation_port': operation_port,
                 'test_mode': True, 'enable_transmission': True,
             }]),
        Node(package='auto_mode_manager', executable='mock_mainboard_fixture.py',
             name='stm32_test_mainboard', output='screen', parameters=[{
                 'test_mode': True, 'local_address': '127.0.0.2', 'peer_address': '127.0.0.1',
                 'command_port': command_port, 'operation_port': operation_port,
             }]),
    ]


def generate_launch_description():
    """Provide an isolated early communication stage with explicit task boundaries."""
    return LaunchDescription([
        DeclareLaunchArgument('stage', default_value='communication'),
        DeclareLaunchArgument('domain_id', default_value='230'),
        DeclareLaunchArgument('command_port', default_value='26574'),
        DeclareLaunchArgument('operation_port', default_value='39244'),
        OpaqueFunction(function=test_nodes),
    ])
