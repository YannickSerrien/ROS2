# Lesson 10: Launch Files Advanced

## Learning Objectives

- Use launch arguments for flexible configurations
- Implement conditional node launching
- Include other launch files
- Apply namespaces for multi-robot systems
- Use launch events and event handlers
- Create reusable launch file templates

## Launch Arguments

Make launch files configurable:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot'
    )

    # Use arguments
    node = Node(
        package='my_package',
        executable='my_node',
        name=LaunchConfiguration('robot_name'),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        node
    ])
```

**Running**:
```bash
# Use defaults
ros2 launch my_package system.launch.py

# Override arguments
ros2 launch my_package system.launch.py use_sim_time:=true robot_name:=robot2
```

## Conditional Launching

Launch nodes based on conditions:

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Argument
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true'
    )

    # Launch only if enabled
    camera_node = Node(
        package='camera_driver',
        executable='camera_node',
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )

    # Launch only if NOT enabled
    dummy_camera = Node(
        package='simulators',
        executable='fake_camera',
        condition=UnlessCondition(LaunchConfiguration('enable_camera'))
    )

    return LaunchDescription([
        enable_camera_arg,
        camera_node,
        dummy_camera
    ])
```

## Including Launch Files

Compose systems from smaller launch files:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include another launch file
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sensor_package'),
                'launch',
                'sensors.launch.py'
            )
        ]),
        launch_arguments={'namespace': 'robot1'}.items()
    )

    return LaunchDescription([sensors_launch])
```

## Namespaces

Organize nodes for multi-robot systems:

```python
def generate_launch_description():
    # Robot 1
    robot1_node = Node(
        package='my_package',
        executable='robot_controller',
        namespace='robot1',  # Topics become /robot1/cmd_vel
        parameters=[{'robot_id': 1}]
    )

    # Robot 2
    robot2_node = Node(
        package='my_package',
        executable='robot_controller',
        namespace='robot2',  # Topics become /robot2/cmd_vel
        parameters=[{'robot_id': 2}]
    )

    return LaunchDescription([robot1_node, robot2_node])
```

## Remapping

Change topic/service names:

```python
Node(
    package='my_package',
    executable='my_node',
    remappings=[
        ('/input_topic', '/sensor/data'),
        ('/output_topic', '/processed/data'),
        ('service_name', 'new_service_name')
    ]
)
```

## Group Actions

Apply configuration to multiple nodes:

```python
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # Group with shared namespace
    robot1_group = GroupAction([
        PushRosNamespace('robot1'),
        Node(package='pkg1', executable='node1'),
        Node(package='pkg2', executable='node2'),
        # Both nodes are in /robot1 namespace
    ])

    return LaunchDescription([robot1_group])
```

## Events and Event Handlers

React to node lifecycle events:

```python
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    my_node = Node(
        package='my_package',
        executable='my_node',
        name='my_node'
    )

    # Log when node starts
    on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=my_node,
            on_start=[
                LogInfo(msg='Node has started!')
            ]
        )
    )

    # React when node exits
    on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=my_node,
            on_exit=[
                LogInfo(msg='Node has exited!')
            ]
        )
    )

    return LaunchDescription([my_node, on_start, on_exit])
```

## Complete Example: Flexible Robot Launch

```python
"""
Advanced launch file with arguments, conditionals, and includes
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot')

    # Arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Robot namespace'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera'
    )

    # Configuration
    robot_name = LaunchConfiguration('robot_name')
    use_sim = LaunchConfiguration('use_sim')
    enable_camera = LaunchConfiguration('enable_camera')

    # Include sensor launch file
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('sensor_bringup'),
                'launch',
                'sensors.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name,
            'use_sim_time': use_sim
        }.items()
    )

    # Robot nodes with namespace
    robot_group = GroupAction([
        PushRosNamespace(robot_name),

        # Controller
        Node(
            package='robot_control',
            executable='controller',
            parameters=[{
                'use_sim_time': use_sim
            }]
        ),

        # Camera (conditional)
        Node(
            package='camera_driver',
            executable='camera_node',
            condition=IfCondition(enable_camera),
            parameters=[{
                'use_sim_time': use_sim
            }]
        ),

        # State publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ... # URDF content
            }]
        )
    ])

    # Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_dir, 'rviz', 'robot.rviz'])],
        condition=IfCondition(use_sim)
    )

    return LaunchDescription([
        # Arguments
        robot_name_arg,
        use_sim_arg,
        enable_camera_arg,

        # Actions
        sensors_launch,
        robot_group,
        rviz_node
    ])
```

**Usage**:
```bash
# Real robot with camera
ros2 launch my_robot robot.launch.py

# Simulation without camera
ros2 launch my_robot robot.launch.py use_sim:=true enable_camera:=false

# Different robot name
ros2 launch my_robot robot.launch.py robot_name:=robot2
```

## Best Practices

1. **Always Provide Defaults**
   ```python
   DeclareLaunchArgument('arg_name', default_value='default')
   ```

2. **Add Descriptions**
   ```python
   DeclareLaunchArgument(
       'my_arg',
       default_value='value',
       description='What this argument does'
   )
   ```

3. **Use Package Relative Paths**
   ```python
   # Good
   get_package_share_directory('my_package')

   # Bad
   '/absolute/path/to/package'
   ```

4. **Organize with Groups**
   - Group related nodes
   - Apply namespace to groups
   - Conditional groups

5. **Document Arguments**
   ```python
   """
   Launch Arguments:
       robot_name: Robot namespace (default: robot1)
       use_sim: Use simulation time (default: false)
   """
   ```

## Summary

Advanced launch file features:

**Arguments**: Make launch files configurable
**Conditionals**: Launch nodes based on conditions
**Includes**: Compose from smaller launch files
**Namespaces**: Multi-robot support
**Events**: React to node lifecycle
**Groups**: Apply configuration to multiple nodes

## What's Next?

- **Lesson 11**: Composition Basics - Performance optimization

---

**Next Lesson**: [Lesson 11: Composition Basics](11-composition-basics.md)
