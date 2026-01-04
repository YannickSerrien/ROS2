"""
Complete robot simulation with Gazebo and RViz
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_dir = get_package_share_directory('sensor_robot_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths
    xacro_file = os.path.join(pkg_dir, 'urdf', 'sensor_robot.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'empty.world')
    rviz_config = os.path.join(pkg_dir, 'config', 'simulation.rviz')

    # Process XACRO
    robot_description = xacro.process_file(xacro_file).toxml()

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Spawn robot (delayed)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'sensor_robot',
                    '-topic', '/robot_description',
                    '-z', '0.5'
                ],
                output='screen'
            )
        ]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        rviz
    ])
