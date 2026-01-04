"""
Separate launch file - Each node in OWN process

Demonstrates:
- Traditional separate process approach
- Comparison with composed version
- Performance difference (serialization overhead)

Each node runs in separate process -> messages serialized/copied!

Used in: Module 3 - Composition (Comparison)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch each component as separate node."""

    # Image producer node
    producer_node = Node(
        package='composition_example',
        executable='image_producer_node',
        name='image_producer',
        parameters=[{
            'width': 640,
            'height': 480,
            'frame_rate': 30.0
        }],
        output='screen'
    )

    # Image processor node
    processor_node = Node(
        package='composition_example',
        executable='image_processor_node',
        name='image_processor',
        output='screen'
    )

    return LaunchDescription([
        producer_node,
        processor_node
    ])
