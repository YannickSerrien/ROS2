"""
Robot arm system launch file

Starts:
1. Arm controller (TF broadcasting and joint control)
2. Move arm action server (goal-based movement)

Configuration loaded from arm_params.yaml
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch robot arm control system."""

    # Get package directory
    pkg_dir = get_package_share_directory('robot_arm_project')
    config_file = os.path.join(pkg_dir, 'config', 'arm_params.yaml')

    # Arm controller node
    arm_controller = Node(
        package='robot_arm_project',
        executable='arm_controller',
        name='arm_controller',
        parameters=[config_file],
        output='screen'
    )

    # Move arm action server
    move_arm_server = Node(
        package='robot_arm_project',
        executable='move_arm_server',
        name='move_arm_server',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        arm_controller,
        move_arm_server
    ])
