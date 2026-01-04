"""
Multi-robot/multi-room launch file demonstration

Demonstrates:
- Multiple instances of same nodes
- Namespace separation
- Independent parameter configuration
- Practical multi-robot pattern

Launches monitoring systems for 2 rooms:
- room1: Target 22°C
- room2: Target 24°C

Used in: Module 3 - Launch Files Advanced (Namespaces)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate multi-room monitoring system."""

    # Room 1 nodes
    room1_sensor = Node(
        package='launch_example',
        executable='sensor_node',
        name='temp_sensor',
        namespace='room1',
        parameters=[{
            'sensor_name': 'room1_sensor',
            'publish_rate': 5.0,
            'min_temp': 19.0,
            'max_temp': 25.0
        }],
        output='screen'
    )

    room1_processor = Node(
        package='launch_example',
        executable='processor_node',
        name='data_processor',
        namespace='room1',
        parameters=[{
            'window_size': 15,
            'warning_threshold': 26.0
        }],
        output='screen'
    )

    room1_controller = Node(
        package='launch_example',
        executable='controller_node',
        name='hvac_controller',
        namespace='room1',
        parameters=[{
            'target_temp': 22.0,
            'tolerance': 1.0,
            'max_power': 100.0
        }],
        output='screen'
    )

    # Room 2 nodes
    room2_sensor = Node(
        package='launch_example',
        executable='sensor_node',
        name='temp_sensor',
        namespace='room2',
        parameters=[{
            'sensor_name': 'room2_sensor',
            'publish_rate': 5.0,
            'min_temp': 20.0,
            'max_temp': 28.0
        }],
        output='screen'
    )

    room2_processor = Node(
        package='launch_example',
        executable='processor_node',
        name='data_processor',
        namespace='room2',
        parameters=[{
            'window_size': 15,
            'warning_threshold': 28.0
        }],
        output='screen'
    )

    room2_controller = Node(
        package='launch_example',
        executable='controller_node',
        name='hvac_controller',
        namespace='room2',
        parameters=[{
            'target_temp': 24.0,  # Different target than room1
            'tolerance': 1.5,
            'max_power': 80.0
        }],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        # Room 1
        room1_sensor,
        room1_processor,
        room1_controller,

        # Room 2
        room2_sensor,
        room2_processor,
        room2_controller
    ])
