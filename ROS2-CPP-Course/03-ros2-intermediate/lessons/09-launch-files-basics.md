# Lesson 9: Launch Files Basics

## Learning Objectives

- Understand what launch files are and why they're essential
- Create Python launch files for ROS2
- Start multiple nodes with a single command
- Pass parameters to nodes from launch files
- Load parameters from YAML files
- Use basic launch actions (Node, ExecuteProcess)

## Introduction

Real robot systems have 10+ nodes. Starting them manually with `ros2 run` is impractical. **Launch files** automate system startup.

### Without Launch Files

```bash
# Terminal 1
ros2 run pkg1 node1
# Terminal 2
ros2 run pkg2 node2 --ros-args -p param:=value
# Terminal 3
ros2 run pkg3 node3
# ... 7 more terminals
```

### With Launch Files

```bash
# Single command!
ros2 launch my_package my_system.launch.py
```

## Basic Launch File Structure

**File**: `launch/my_first_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='my_talker'
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='my_listener'
        )
    ])
```

### File Location

```
my_package/
├── launch/
│   └── my_system.launch.py
├── package.xml
└── CMakeLists.txt
```

### CMakeLists.txt

```cmake
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

### Running

```bash
colcon build --packages-select my_package
source install/setup.bash
ros2 launch my_package my_system.launch.py
```

## Starting Multiple Nodes

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define nodes
    camera_node = Node(
        package='my_camera',
        executable='camera_driver',
        name='front_camera'
    )

    detector_node = Node(
        package='object_detection',
        executable='detector',
        name='object_detector'
    )

    tracker_node = Node(
        package='tracking',
        executable='tracker',
        name='object_tracker'
    )

    # Return all nodes
    return LaunchDescription([
        camera_node,
        detector_node,
        tracker_node
    ])
```

## Passing Parameters

### Inline Parameters

```python
Node(
    package='my_package',
    executable='my_node',
    name='configured_node',
    parameters=[{
        'max_speed': 2.5,
        'robot_name': 'RobotA',
        'enable_sensors': True
    }]
)
```

### From YAML File

**File**: `config/params.yaml`
```yaml
my_node:
  ros__parameters:
    max_speed: 2.5
    robot_name: "RobotA"
    enable_sensors: true
```

**Launch File**:
```python
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get path to config file
    config_file = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            parameters=[config_file]
        )
    ])
```

## Node Configuration Options

```python
Node(
    package='my_package',
    executable='my_node',
    name='custom_name',  # Override default node name
    namespace='robot1',  # Add namespace
    output='screen',  # Show output in terminal
    parameters=[{'param': 'value'}],
    remappings=[
        ('/old_topic', '/new_topic'),  # Remap topics
    ],
    arguments=['--ros-args', '--log-level', 'debug']  # Extra args
)
```

## Complete Example: Multi-Sensor System

```python
"""
Launch file for multi-sensor robot system
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('sensor_system')

    # Config files
    camera_config = os.path.join(pkg_dir, 'config', 'camera_params.yaml')
    lidar_config = os.path.join(pkg_dir, 'config', 'lidar_params.yaml')

    # Camera driver
    camera_node = Node(
        package='camera_driver',
        executable='camera_node',
        name='front_camera',
        parameters=[camera_config],
        output='screen'
    )

    # Lidar driver
    lidar_node = Node(
        package='lidar_driver',
        executable='lidar_node',
        name='laser_scanner',
        parameters=[lidar_config],
        output='screen'
    )

    # Sensor fusion
    fusion_node = Node(
        package='sensor_fusion',
        executable='fusion_node',
        name='sensor_fusion',
        parameters=[{
            'update_rate': 20.0,
            'fusion_method': 'kalman'
        }],
        output='screen'
    )

    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'config.rviz')]
    )

    return LaunchDescription([
        camera_node,
        lidar_node,
        fusion_node,
        rviz_node
    ])
```

## Logging Output

```python
# Show all output in terminal
Node(..., output='screen')

# Log to file
Node(..., output='log')

# Both terminal and log
Node(..., output='both')

# Suppress output
Node(..., output=None)
```

## Common Pitfalls

1. **Wrong File Extension**
   - Must be `.launch.py` (not `.py`)

2. **Forgot to Install Launch Directory**
   ```cmake
   # Add to CMakeLists.txt
   install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
   ```

3. **Wrong Function Name**
   ```python
   # Must be exactly this name
   def generate_launch_description():
   ```

4. **Incorrect Package Path**
   ```python
   # Good: Use ament_index
   get_package_share_directory('my_package')

   # Bad: Hardcoded paths
   '/home/user/ros2_ws/src/my_package'
   ```

## Summary

Launch files start multiple nodes with one command:

**Key Components**:
- Python file in `launch/` directory
- `generate_launch_description()` function
- Returns `LaunchDescription([...])`
- `Node()` actions for each node

**Benefits**:
- Start entire system with one command
- Configure parameters from files
- Reproducible system startup
- Easy to modify and share

## What's Next?

- **Lesson 10**: Advanced launch features (arguments, conditionals, includes)

---

**Next Lesson**: [Lesson 10: Launch Advanced](10-launch-advanced.md)
