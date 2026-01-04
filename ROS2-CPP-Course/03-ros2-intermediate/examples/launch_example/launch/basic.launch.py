"""
Basic launch file demonstration

Starts all three nodes with default parameters:
- sensor_node: Publishes simulated temperature data
- processor_node: Computes moving average
- controller_node: Controls HVAC based on temperature

Used in: Module 3 - Launch Files Basics
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple nodes."""

    # Sensor node
    sensor_node = Node(
        package='launch_example',
        executable='sensor_node',
        name='temp_sensor',
        output='screen'
    )

    # Processor node
    processor_node = Node(
        package='launch_example',
        executable='processor_node',
        name='data_processor',
        output='screen'
    )

    # Controller node
    controller_node = Node(
        package='launch_example',
        executable='controller_node',
        name='hvac_controller',
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        sensor_node,
        processor_node,
        controller_node
    ])
