"""
Advanced launch file demonstration

Demonstrates:
- Launch arguments for configuration
- Conditional node launching
- Parameter passing (inline and from YAML)
- Namespaces for multi-instance systems
- Remapping topics

Used in: Module 3 - Launch Files Advanced
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate advanced launch description."""

    # Get package directory
    pkg_dir = get_package_share_directory('launch_example')

    # Declare launch arguments
    use_namespace_arg = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Launch nodes in a namespace'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='room1',
        description='Namespace for nodes'
    )

    enable_controller_arg = DeclareLaunchArgument(
        'enable_controller',
        default_value='true',
        description='Enable controller node'
    )

    use_config_file_arg = DeclareLaunchArgument(
        'use_config',
        default_value='false',
        description='Load parameters from YAML file'
    )

    target_temp_arg = DeclareLaunchArgument(
        'target_temp',
        default_value='22.0',
        description='Target temperature setpoint'
    )

    # Get launch configuration values
    use_namespace = LaunchConfiguration('use_namespace')
    namespace = LaunchConfiguration('namespace')
    enable_controller = LaunchConfiguration('enable_controller')
    use_config = LaunchConfiguration('use_config')
    target_temp = LaunchConfiguration('target_temp')

    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # Sensor node with inline parameters
    sensor_node = Node(
        package='launch_example',
        executable='sensor_node',
        name='temp_sensor',
        parameters=[{
            'sensor_name': 'indoor_sensor',
            'publish_rate': 5.0,
            'min_temp': 18.0,
            'max_temp': 28.0
        }],
        output='screen'
    )

    # Processor node (parameters from file if use_config=true)
    processor_node = Node(
        package='launch_example',
        executable='processor_node',
        name='data_processor',
        parameters=[config_file] if use_config else [{
            'window_size': 20,
            'warning_threshold': 27.0
        }],
        output='screen'
    )

    # Controller node (conditional launch)
    controller_node = Node(
        package='launch_example',
        executable='controller_node',
        name='hvac_controller',
        parameters=[{
            'target_temp': target_temp,
            'tolerance': 1.5,
            'max_power': 80.0
        }],
        condition=IfCondition(enable_controller),
        output='screen'
    )

    # Group nodes with optional namespace
    node_group = GroupAction(
        actions=[
            PushRosNamespace(namespace) if use_namespace else GroupAction([]),
            sensor_node,
            processor_node,
            controller_node
        ]
    )

    # Create and return launch description
    return LaunchDescription([
        # Arguments
        use_namespace_arg,
        namespace_arg,
        enable_controller_arg,
        use_config_file_arg,
        target_temp_arg,

        # Nodes
        node_group
    ])
