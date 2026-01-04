"""
Display XACRO robot in RViz (processes XACRO at launch time)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_dir = get_package_share_directory('urdf_robot_example')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'config', 'urdf_config.rviz')

    # Process XACRO file to get robot description
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
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
