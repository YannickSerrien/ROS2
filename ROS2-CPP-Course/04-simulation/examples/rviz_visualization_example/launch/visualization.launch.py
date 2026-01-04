"""
Launch RViz visualization examples
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('rviz_visualization_example')
    rviz_config = os.path.join(pkg_dir, 'config', 'visualization.rviz')

    return LaunchDescription([
        # Marker publisher
        Node(
            package='rviz_visualization_example',
            executable='marker_publisher',
            name='marker_publisher',
            output='screen'
        ),

        # Path visualizer
        Node(
            package='rviz_visualization_example',
            executable='path_visualizer',
            name='path_visualizer',
            output='screen'
        ),

        # Interactive marker server
        Node(
            package='rviz_visualization_example',
            executable='interactive_marker_server',
            name='interactive_marker_server',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
