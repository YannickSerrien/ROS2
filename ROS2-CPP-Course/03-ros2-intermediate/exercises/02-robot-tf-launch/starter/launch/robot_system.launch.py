"""
Robot system launch file - STARTER

TODO: Complete the launch file

Tasks:
1. Declare launch arguments (robot_radius, robot_speed, use_config)
2. Create static TF node
3. Create odom broadcaster node
4. Create sensor monitor node
5. Support config file loading
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for robot TF system."""

    # Get package directory
    pkg_dir = get_package_share_directory('robot_tf_launch')
    config_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')

    # TODO: Declare launch arguments
    # Hint: DeclareLaunchArgument('robot_radius', default_value='2.0', ...)

    # TODO: Get launch configuration values
    # Hint: robot_radius = LaunchConfiguration('robot_radius')

    # TODO: Create static TF node
    # Hint: Node(package='robot_tf_launch', executable='static_tf_node', ...)
    #       Use parameters=[config_file] if use_config is true

    # TODO: Create odom broadcaster node
    # Pass robot_radius and robot_speed as parameters

    # TODO: Create sensor monitor node

    # TODO: Return LaunchDescription with all nodes

    return LaunchDescription([
        # Add your nodes here
    ])
