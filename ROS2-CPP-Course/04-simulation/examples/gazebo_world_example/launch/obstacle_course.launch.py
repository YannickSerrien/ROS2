"""
Launch Gazebo with obstacle course world
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():

    pkg_dir = get_package_share_directory('gazebo_world_example')
    world_file = os.path.join(pkg_dir, 'worlds', 'obstacle_course.world')
    models_dir = os.path.join(pkg_dir, 'models')

    # Set Gazebo model path to find custom models
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_dir
    )

    # Launch Gazebo with world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file],
        output='screen'
    )

    return LaunchDescription([
        set_model_path,
        gazebo
    ])
