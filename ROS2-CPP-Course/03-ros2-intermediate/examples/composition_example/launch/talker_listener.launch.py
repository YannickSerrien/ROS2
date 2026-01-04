"""
Simple talker/listener composition example

Demonstrates:
- Basic composition with simple nodes
- Zero-copy for string messages
- Container management

Used in: Module 3 - Composition (Basic Example)
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch talker and listener in single container."""

    container = ComposableNodeContainer(
        name='talker_listener_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='composition_example',
                plugin='composition_example::TalkerComponent',
                name='talker',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='composition_example',
                plugin='composition_example::ListenerComponent',
                name='listener',
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
