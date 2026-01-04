"""
Launch file for temperature monitoring system

TODO: Complete the launch file to start all nodes with proper configuration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('temperature_monitor')

    # Configuration files
    sensors_config = os.path.join(pkg_dir, 'config', 'sensors.yaml')
    monitor_config = os.path.join(pkg_dir, 'config', 'monitor.yaml')

    return LaunchDescription([
        # TODO: Add launch arguments if needed (e.g., enable_logging, custom_config)

        # TODO: Launch sensor nodes (3 instances with different configs)
        # Hint: Use Node() with name='temp_sensor_<room>', parameters=[sensors_config]

        Node(
            package='temperature_monitor',
            executable='temp_sensor_node',
            name='temp_sensor_living_room',
            parameters=[sensors_config],
            output='screen'
        ),

        # TODO: Add bedroom sensor

        # TODO: Add kitchen sensor

        # TODO: Launch monitor node
        Node(
            package='temperature_monitor',
            executable='monitor_node',
            name='temp_monitor',
            parameters=[monitor_config],
            output='screen'
        ),

        # TODO: Launch alerter node

        # TODO: Launch dashboard node
    ])
