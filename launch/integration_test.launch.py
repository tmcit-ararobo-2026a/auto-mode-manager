"""Start the communication-only loopback test stage; no production fixtures are included."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def test_nodes(context):
    """Validate the eligible stage and test domain before starting either node."""
    stage = LaunchConfiguration('stage').perform(context)
    domain = int(LaunchConfiguration('domain_id').perform(context))
    if stage not in ('communication', 'fixed_navigation', 'tracking_shot'):
        raise ValueError('Eligible stages are communication, fixed_navigation and tracking_shot; hand unavailable')
    if domain < 100 or domain > 232:
        raise ValueError('Test domain must be in [100,232]')
    command_port = int(LaunchConfiguration('command_port').perform(context))
    operation_port = int(LaunchConfiguration('operation_port').perform(context))
    clock = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}
    if stage != 'communication':
        share = Path(get_package_share_directory('auto_mode_manager'))
        profile = str(share / 'config/simulation.json')
        nodes = [
            Node(package='robot_port', executable='udp_node', name='stm32_test_udp', output='screen', parameters=[{
                'local_address': '127.0.0.1', 'peer_address': '127.0.0.2', 'command_port': command_port,
                'operation_port': operation_port, 'test_mode': True, 'enable_transmission': True, **clock}]),
            Node(package='auto_mode_manager', executable='simulation_fixture.py', name='stm32_test_fixture',
                 output='screen', parameters=[{'profile_file': profile, 'test_mode': True,
                                              'command_port': command_port, 'operation_port': operation_port, **clock}]),
            Node(package='gn10_pointcloud_localization', executable='localization_status_monitor_node',
                 name='stm32_test_localization', output='screen', parameters=[{
                     'status_monitor.quality_profile': str(share / 'config/simulation_quality.json'), **clock}]),
            Node(package='auto_mode_manager', executable='auto_mode_manager_runtime.py',
                 name='stm32_test_manager', output='screen', parameters=[{'mode': 'simulation', 'profile_file': profile, **clock}]),
            Node(package='auto_mode_manager', executable='navigation_safety_monitor.py',
                 name='stm32_test_safety', output='screen', parameters=[{'profile_file': profile, **clock}]),
            Node(package='gn10_navigation', executable='path_planner_node', name='stm32_test_planner', output='screen', parameters=[clock]),
            Node(package='gn10_navigation', executable='path_follower_node', name='stm32_test_follower', output='screen',
                 parameters=[{'max_speed': 0.3, 'max_angular_speed': 0.5, 'goal_tolerance': 0.03,
                              'goal_heading_tolerance_rad': 0.05, 'enable_legacy_path_input': False, **clock}]),
            Node(package='shooter_calculator', executable='shooter_calculator_node', name='stm32_test_shooter',
                 output='screen', parameters=[{'calibration_file': str(share / 'config/simulation_shot.json'), **clock}]),
        ]
        handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
                    on_exit=[EmitEvent(event=Shutdown(reason='Simulation process exited'))])) for node in nodes]
        return [SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
                SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain))] + handlers + nodes
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('domain_id', default_value='230'),
        DeclareLaunchArgument('command_port', default_value='26574'),
        DeclareLaunchArgument('operation_port', default_value='39244'),
        OpaqueFunction(function=test_nodes),
    ])
