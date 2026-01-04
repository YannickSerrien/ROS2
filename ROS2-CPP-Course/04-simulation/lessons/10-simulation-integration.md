# Lesson 10: Simulation Integration

## Learning Objectives

- Integrate all simulation components into complete system
- Create production-ready launch file architecture
- Combine RViz, Gazebo, robot control, and navigation
- Organize simulation packages properly
- Implement automated testing workflows
- Debug complex multi-component systems
- Prepare simulations for deployment

## Complete Simulation Stack

### System Architecture

```
┌─────────────────────────────────────────────────────┐
│                  RViz (Visualization)               │
│  - Robot model, sensors, maps, planning            │
└──────────────┬──────────────────────────────────────┘
               │ Topics (/scan, /camera, /odom, /map)
┌──────────────▼──────────────────────────────────────┐
│            Your Robot Nodes                         │
│  - Navigation, planning, control, perception       │
└──────────────┬──────────────────────────────────────┘
               │ /cmd_vel, sensor subscriptions
┌──────────────▼──────────────────────────────────────┐
│         Gazebo (Physics Simulation)                 │
│  - Robot with sensors and controllers              │
│  - World environment                               │
└─────────────────────────────────────────────────────┘
```

### Data Flow

```
Gazebo                  ROS2 Topics              Your Nodes           RViz
  │                                                  │                 │
  ├─ Sensors ──────► /scan, /camera ──────────────► │                 │
  ├─ Odometry ─────► /odom ───────────────────────► │                 │
  │                                                  │                 │
  │                                ┌─── /map ───────┼────────────────►│
  │                                │                 │                 │
  │                                ├─── /path ──────┼────────────────►│
  │                  ┌────────── cmd_vel ◄───────────┤                 │
  │◄─────────────────┘                               │                 │
  │                                                  │                 │
  └─ Robot motion                                    └─────────────────┘
```

## Package Organization

### Standard Simulation Package Structure

```
my_robot_simulation/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── robot_params.yaml
│   ├── rviz_config.rviz
│   └── nav2_params.yaml
├── launch/
│   ├── gazebo.launch.py
│   ├── spawn_robot.launch.py
│   ├── rviz.launch.py
│   ├── robot_sim.launch.py          # Combines Gazebo + robot
│   └── full_sim.launch.py           # Everything together
├── urdf/
│   ├── robot.urdf.xacro
│   ├── robot_gazebo.xacro           # Gazebo-specific (sensors, plugins)
│   ├── properties.xacro
│   └── materials.xacro
├── worlds/
│   ├── empty.world
│   ├── test_arena.world
│   └── obstacles.world
├── meshes/
│   ├── base.dae
│   └── wheel.stl
├── scripts/
│   └── test_navigation.py
└── README.md
```

### Separation of Concerns

**robot_description package**: URDF, meshes (hardware-agnostic)
**robot_gazebo package**: Simulation-specific (worlds, Gazebo plugins)
**robot_bringup package**: Launch files for real robot
**robot_simulation package**: Launch files for simulation

## Complete Launch File Architecture

### Modular Launch System

**1. Gazebo Launch** (`gazebo.launch.py`):

```python
"""
Launch Gazebo with specific world
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_simulation = get_package_share_directory('my_robot_simulation')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_simulation, 'worlds', 'test_arena.world'),
        description='Path to world file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gazebo
    ])
```

**2. Robot State Publisher** (`robot_state.launch.py`):

```python
"""
Launch robot_state_publisher with URDF
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_simulation = get_package_share_directory('my_robot_simulation')

    # URDF/XACRO
    urdf_file = os.path.join(pkg_simulation, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher
    ])
```

**3. Spawn Robot** (`spawn_robot.launch.py`):

```python
"""
Spawn robot in Gazebo
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Launch arguments for initial pose
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.5')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    # Spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', '/robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )

    return LaunchDescription([
        x_arg, y_arg, z_arg, yaw_arg,
        spawn_entity
    ])
```

**4. RViz Launch** (`rviz.launch.py`):

```python
"""
Launch RViz with config
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_simulation = get_package_share_directory('my_robot_simulation')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_simulation, 'config', 'robot_sim.rviz'),
        description='RViz config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        rviz_config_arg,
        use_sim_time_arg,
        rviz
    ])
```

**5. Complete Simulation** (`full_sim.launch.py`):

```python
"""
Complete simulation: Gazebo + Robot + RViz + Controllers
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_simulation = get_package_share_directory('my_robot_simulation')

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='test_arena',
        description='World name (without .world extension)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': [pkg_simulation, '/worlds/', LaunchConfiguration('world'), '.world']
        }.items()
    )

    # 2. Robot state publisher
    robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'robot_state.launch.py')
        )
    )

    # 3. Spawn robot (delayed to ensure Gazebo is ready)
    spawn_robot = TimerAction(
        period=3.0,  # Wait 3 seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_simulation, 'launch', 'spawn_robot.launch.py')
                )
            )
        ]
    )

    # 4. RViz (conditional)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'rviz.launch.py')
        ),
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        world_arg,
        rviz_arg,
        gazebo,
        robot_state,
        spawn_robot,
        rviz
    ])
```

**Usage**:

```bash
# Launch everything with defaults
ros2 launch my_robot_simulation full_sim.launch.py

# Custom world, no RViz
ros2 launch my_robot_simulation full_sim.launch.py \
  world:=obstacles rviz:=false

# Different initial position
ros2 launch my_robot_simulation full_sim.launch.py \
  x:=2.0 y:=3.0 yaw:=1.57
```

## RViz Configuration for Simulation

### Comprehensive RViz Setup

**Create config** (`config/robot_sim.rviz`):

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Global Options:
    Fixed Frame: odom
    Frame Rate: 30

  Displays:
    # Robot model
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Description Topic: /robot_description
      Enabled: true

    # TF frames
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
      Show Axes: true
      Marker Scale: 0.5

    # Lidar
    - Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Topic: /scan
      Size: 0.05
      Color: 255; 0; 0
      Enabled: true

    # Camera
    - Class: rviz_default_plugins/Image
      Name: Camera
      Topic: /camera/image_raw
      Enabled: true

    # Odometry
    - Class: rviz_default_plugins/Odometry
      Name: Odometry
      Topic: /odom
      Color: 0; 255; 0
      Enabled: true
      Keep: 100

    # Map (if using navigation)
    - Class: rviz_default_plugins/Map
      Name: Map
      Topic: /map
      Enabled: true

    # Path (if planning)
    - Class: rviz_default_plugins/Path
      Name: Path
      Topic: /plan
      Color: 255; 255; 0
      Enabled: true

Views:
  Current:
    Class: rviz_default_plugins/Orbit
    Distance: 10.0
    Focal Point:
      X: 0
      Y: 0
      Z: 0
    Pitch: 0.5
    Yaw: 0.5
```

## Testing and Debugging

### Automated Testing Script

```python
#!/usr/bin/env python3
"""
Test robot movement in simulation
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class SimulationTester(Node):

    def __init__(self):
        super().__init__('simulation_tester')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.current_pose = None
        self.test_results = []

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def wait_for_odom(self, timeout=5.0):
        """Wait for first odometry message"""
        start = time.time()
        while self.current_pose is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_pose is not None

    def move_forward(self, distance, speed=0.5):
        """Move forward specified distance"""
        if not self.wait_for_odom():
            self.get_logger().error("No odometry received!")
            return False

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y

        cmd = Twist()
        cmd.linear.x = speed

        rate = self.create_rate(10)  # 10 Hz

        while True:
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)

            dx = self.current_pose.position.x - start_x
            dy = self.current_pose.position.y - start_y
            traveled = (dx**2 + dy**2)**0.5

            if traveled >= distance:
                break

            rate.sleep()

        # Stop
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)

        self.get_logger().info(f"Moved {traveled:.2f}m forward")
        return True

    def rotate(self, angle, angular_speed=0.5):
        """Rotate specified angle in radians"""
        # Implementation similar to move_forward
        pass

    def run_tests(self):
        """Run complete test suite"""
        self.get_logger().info("Starting simulation tests...")

        # Test 1: Forward movement
        self.get_logger().info("Test 1: Moving forward 2m")
        success = self.move_forward(2.0, speed=0.5)
        self.test_results.append(("Forward movement", success))

        time.sleep(1)

        # Test 2: Rotation
        self.get_logger().info("Test 2: Rotating 90 degrees")
        success = self.rotate(1.57, angular_speed=0.5)
        self.test_results.append(("Rotation", success))

        # Report results
        self.get_logger().info("=" * 50)
        self.get_logger().info("TEST RESULTS:")
        for test_name, result in self.test_results:
            status = "PASS" if result else "FAIL"
            self.get_logger().info(f"  {test_name}: {status}")
        self.get_logger().info("=" * 50)

def main():
    rclpy.init()
    tester = SimulationTester()

    try:
        tester.run_tests()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run tests**:
```bash
# Terminal 1: Launch simulation
ros2 launch my_robot_simulation full_sim.launch.py

# Terminal 2: Run tests
ros2 run my_robot_simulation test_navigation.py
```

### Debugging Checklist

**Systematic debugging**:

```bash
# 1. Check Gazebo is running
ps aux | grep gazebo

# 2. List all ROS2 nodes
ros2 node list

# 3. Check topics
ros2 topic list

# 4. Verify robot_description published
ros2 topic echo /robot_description --once

# 5. Check TF tree
ros2 run tf2_tools view_frames

# 6. Monitor cmd_vel
ros2 topic echo /cmd_vel

# 7. Check sensor data
ros2 topic hz /scan
ros2 topic hz /camera/image_raw

# 8. Inspect Gazebo models
gz model --list

# 9. Check for errors in Gazebo
gazebo --verbose
```

## Integration with Navigation Stack

### Adding Nav2 to Simulation

**nav2_params.yaml**:
```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    # AMCL parameters...

controller_server:
  ros__parameters:
    use_sim_time: True
    # Controller parameters...

planner_server:
  ros__parameters:
    use_sim_time: True
    # Planner parameters...

# ... more Nav2 nodes
```

**Navigation launch** (`nav_sim.launch.py`):
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    pkg_simulation = get_package_share_directory('my_robot_simulation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # Full simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'full_sim.launch.py')
        )
    )

    # Nav2
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_simulation, 'config', 'nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        simulation,
        navigation
    ])
```

## Performance Optimization

### Simulation Speed Tips

**1. Reduce physics complexity**:
```xml
<!-- In world file -->
<physics>
  <max_step_size>0.01</max_step_size>  <!-- Larger = faster -->
  <real_time_update_rate>100</real_time_update_rate>  <!-- Lower = faster -->
</physics>
```

**2. Simplify collision meshes**:
```xml
<!-- Use boxes instead of complex meshes -->
<collision>
  <geometry><box size="0.5 0.5 0.5"/></geometry>
</collision>
```

**3. Reduce sensor update rates**:
```xml
<sensor>
  <update_rate>10</update_rate>  <!-- Lower = faster -->
</sensor>
```

**4. Disable visualizations**:
```xml
<sensor>
  <visualize>false</visualize>  <!-- No ray visualization in Gazebo -->
</sensor>
```

**5. Run headless**:
```bash
# Server only, no GUI
gzserver my_world.world
```

## Best Practices

**1. Modular launch files**:
- Separate Gazebo, robot, sensors, navigation
- Easy to test components individually
- Reusable across projects

**2. Parameterized launches**:
```python
# Use arguments for flexibility
DeclareLaunchArgument('world', default_value='test_arena')
DeclareLaunchArgument('x', default_value='0.0')
DeclareLaunchArgument('rviz', default_value='true')
```

**3. Use sim_time everywhere**:
```python
# All nodes must use simulation time
parameters=[{'use_sim_time': True}]
```

**4. Version control RViz configs**:
- Save RViz config in repo
- Load from launch file
- Document what each display shows

**5. Automated testing**:
- Write test scripts
- Verify robot can move, sense, navigate
- Run in CI/CD pipeline

**6. Documentation**:
```markdown
# README.md

## Launching Simulation

### Full simulation
ros2 launch my_robot_simulation full_sim.launch.py

### Arguments
- world: World file name (default: test_arena)
- x, y, z: Initial position
- rviz: Launch RViz (default: true)

## Testing
ros2 run my_robot_simulation test_navigation.py
```

## Summary

**Complete simulation integrates**:
- Gazebo (physics, sensors)
- URDF (robot description)
- Controllers (differential drive, Ackermann)
- RViz (visualization)
- Your robot nodes (navigation, perception)

**Launch file architecture**:
- Modular: Separate components
- Parameterized: Flexible configuration
- Hierarchical: Combine smaller launches

**Testing strategy**:
- Automated test scripts
- Systematic debugging
- Performance monitoring

**Production readiness**:
- Proper package organization
- Documentation
- Optimized performance
- Reproducible results

## What's Next?

**Next Module**: Hardware Integration - Transition from simulation to real robots

**You'll learn**:
- Sensor drivers and interfaces
- Motor control
- Hardware abstraction
- Sim-to-real transfer
- Safety systems

---

**Key Takeaway**: A well-integrated simulation is a complete testing environment - build it right to accelerate development and ensure quality!
