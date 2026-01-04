"""
Composed launch file - All nodes in ONE process

Demonstrates:
- ComposableNodeContainer for intra-process communication
- Zero-copy message passing
- Performance optimization through composition

All nodes run in single process -> zero-copy enabled!

Used in: Module 3 - Composition
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch all components in a single container."""

    # Create container with composed nodes
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Image producer component
            ComposableNode(
                package='composition_example',
                plugin='composition_example::ImageProducerComponent',
                name='image_producer',
                parameters=[{
                    'width': 640,
                    'height': 480,
                    'frame_rate': 30.0
                }],
                extra_arguments=[{
                    'use_intra_process_comms': True  # Enable zero-copy
                }]
            ),
            # Image processor component
            ComposableNode(
                package='composition_example',
                plugin='composition_example::ImageProcessorComponent',
                name='image_processor',
                extra_arguments=[{
                    'use_intra_process_comms': True  # Enable zero-copy
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
