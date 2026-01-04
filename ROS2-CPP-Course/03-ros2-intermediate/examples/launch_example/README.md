# Launch Files Example

Complete demonstration of ROS2 launch files with multi-node systems, parameters, namespaces, and configuration management.

## What's Included

This package demonstrates launch files through a simulated temperature monitoring and control system:

### Nodes

**1. Sensor Node** (`sensor_node`)
- Publishes simulated temperature data
- Configurable via parameters: sensor_name, publish_rate, min_temp, max_temp

**2. Processor Node** (`processor_node`)
- Subscribes to temperature data
- Computes moving average
- Publishes processed data
- Configurable: window_size, warning_threshold

**3. Controller Node** (`controller_node`)
- Subscribes to processed temperature
- Implements proportional control
- Publishes HVAC power commands
- Configurable: target_temp, tolerance, max_power

### Launch Files

**1. basic.launch.py** - Simple multi-node launch
**2. advanced.launch.py** - Arguments, conditionals, parameters
**3. multi_robot.launch.py** - Multiple instances with namespaces

## Building

```bash
# From workspace root
colcon build --packages-select launch_example
source install/setup.bash
```

## Running the Examples

### Example 1: Basic Launch (All Defaults)

```bash
ros2 launch launch_example basic.launch.py
```

**What happens**:
- Sensor publishes temperature data at 10 Hz
- Processor computes 10-sample moving average
- Controller maintains 22°C target temperature

**Monitoring**:
```bash
# List running nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor temperature
ros2 topic echo /temperature

# Monitor control commands
ros2 topic echo /hvac_power
```

### Example 2: Advanced Launch (With Arguments)

**Default run**:
```bash
ros2 launch launch_example advanced.launch.py
```

**Custom target temperature**:
```bash
ros2 launch launch_example advanced.launch.py target_temp:=25.0
```

**Disable controller**:
```bash
ros2 launch launch_example advanced.launch.py enable_controller:=false
```

**Use configuration file**:
```bash
ros2 launch launch_example advanced.launch.py use_config:=true
```

**With namespace**:
```bash
ros2 launch launch_example advanced.launch.py use_namespace:=true namespace:=building_a
```

Topics become:
- `/building_a/temperature`
- `/building_a/temperature_avg`
- `/building_a/hvac_power`

**Combine multiple arguments**:
```bash
ros2 launch launch_example advanced.launch.py \
  use_namespace:=true \
  namespace:=room1 \
  target_temp:=23.5 \
  enable_controller:=true
```

### Example 3: Multi-Room System

```bash
ros2 launch launch_example multi_robot.launch.py
```

**What happens**:
- Launches 6 nodes (3 per room)
- Room 1: Target 22°C
- Room 2: Target 24°C
- Each room operates independently

**Monitoring specific room**:
```bash
# Room 1 temperature
ros2 topic echo /room1/temperature

# Room 2 controller output
ros2 topic echo /room2/hvac_power

# List all nodes
ros2 node list
```

**Expected output**:
```
/room1/temp_sensor
/room1/data_processor
/room1/hvac_controller
/room2/temp_sensor
/room2/data_processor
/room2/hvac_controller
```

## Launch File Features Demonstrated

### 1. Basic Node Launching

```python
Node(
    package='launch_example',
    executable='sensor_node',
    name='temp_sensor',
    output='screen'  # Show output in terminal
)
```

### 2. Inline Parameters

```python
Node(
    package='launch_example',
    executable='sensor_node',
    parameters=[{
        'sensor_name': 'indoor_sensor',
        'publish_rate': 5.0,
        'min_temp': 18.0,
        'max_temp': 28.0
    }]
)
```

### 3. Parameters from YAML File

```python
import os
from ament_index_python.packages import get_package_share_directory

pkg_dir = get_package_share_directory('launch_example')
config_file = os.path.join(pkg_dir, 'config', 'params.yaml')

Node(
    package='launch_example',
    executable='processor_node',
    parameters=[config_file]
)
```

### 4. Launch Arguments

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Declare argument
target_temp_arg = DeclareLaunchArgument(
    'target_temp',
    default_value='22.0',
    description='Target temperature setpoint'
)

# Use argument
target_temp = LaunchConfiguration('target_temp')

Node(
    parameters=[{'target_temp': target_temp}]
)
```

### 5. Conditional Launching

```python
from launch.conditions import IfCondition

enable_controller = LaunchConfiguration('enable_controller')

Node(
    package='launch_example',
    executable='controller_node',
    condition=IfCondition(enable_controller)  # Only launch if true
)
```

### 6. Namespaces

```python
Node(
    package='launch_example',
    executable='sensor_node',
    namespace='room1'  # Topics become /room1/temperature
)
```

### 7. Groups and Namespace Pushing

```python
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

GroupAction([
    PushRosNamespace('room1'),
    sensor_node,
    processor_node,
    controller_node
])
```

## Configuration File Format

**config/params.yaml**:
```yaml
data_processor:
  ros__parameters:
    window_size: 25
    warning_threshold: 26.5

hvac_controller:
  ros__parameters:
    target_temp: 23.0
    tolerance: 2.0
    max_power: 90.0
```

**Note**: `ros__parameters` is required for ROS2 parameter loading.

## Command Line Tools

### Inspect running launch

```bash
# List nodes
ros2 node list

# Node information
ros2 node info /temp_sensor

# List parameters
ros2 param list /data_processor

# Get parameter value
ros2 param get /hvac_controller target_temp

# Set parameter at runtime
ros2 param set /hvac_controller target_temp 25.0
```

### Monitor system

```bash
# Topic list
ros2 topic list

# Topic info
ros2 topic info /temperature

# Echo topic
ros2 topic echo /temperature_avg

# Topic publication rate
ros2 topic hz /temperature
```

## System Architecture

```
sensor_node
    ↓ publishes
[/temperature] (Float64)
    ↓ subscribes
processor_node
    ↓ publishes
[/temperature_avg] (Float64)
    ↓ subscribes
controller_node
    ↓ publishes
[/hvac_power] (Float64)
```

## Common Patterns

### Pattern 1: Single System with Defaults

```python
def generate_launch_description():
    return LaunchDescription([
        Node(package='pkg', executable='node1'),
        Node(package='pkg', executable='node2')
    ])
```

### Pattern 2: Configurable System

```python
def generate_launch_description():
    target_arg = DeclareLaunchArgument('target', default_value='22.0')

    node = Node(
        package='pkg',
        executable='node',
        parameters=[{'target': LaunchConfiguration('target')}]
    )

    return LaunchDescription([target_arg, node])
```

### Pattern 3: Multi-Instance System

```python
def generate_launch_description():
    instance1 = Node(package='pkg', executable='node', namespace='instance1')
    instance2 = Node(package='pkg', executable='node', namespace='instance2')

    return LaunchDescription([instance1, instance2])
```

## Troubleshooting

**Nodes not starting**:
```bash
# Check if package is built
ros2 pkg list | grep launch_example

# Verify executables
ros2 pkg executables launch_example
```

**Parameters not loading**:
```bash
# Check YAML syntax
cat install/launch_example/share/launch_example/config/params.yaml

# Verify parameter names match node declarations
ros2 param list /node_name
```

**Namespace issues**:
```bash
# List all topics (should show namespace prefixes)
ros2 topic list

# Echo namespaced topic
ros2 topic echo /room1/temperature
```

**Launch file errors**:
- Check Python syntax (indentation, missing imports)
- Verify package names and executables
- Ensure paths are correct (use `get_package_share_directory`)

## Best Practices

1. **Always provide default values** for launch arguments
2. **Add descriptions** to all launch arguments
3. **Use relative paths** via `get_package_share_directory`
4. **Set output='screen'** to see node output during development
5. **Group related nodes** for organization
6. **Use namespaces** for multi-instance systems
7. **Document launch arguments** at top of file

## Related Lessons

- **Lesson 9**: Launch Files Basics
- **Lesson 10**: Launch Files Advanced

## Next Steps

- Add more conditional logic
- Include other launch files
- Add event handlers
- Integrate with TF2 and actions
- Use in mini-project robot arm
