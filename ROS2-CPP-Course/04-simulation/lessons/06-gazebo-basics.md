# Lesson 6: Gazebo Basics

## Learning Objectives

- Understand Gazebo architecture and capabilities
- Start and navigate the Gazebo interface
- Load and manipulate models
- Configure basic physics parameters
- Understand simulation time vs real time
- Use Gazebo command-line tools
- Integrate Gazebo with ROS2

## What is Gazebo?

**Gazebo**: High-fidelity 3D robot simulator with physics engine, sensors, and ROS2 integration.

**Current versions**:
- **Gazebo Classic** (Gazebo 11): Legacy version, widely used
- **Ignition Gazebo** (now "Gazebo"): Modern rewrite, better performance

> **Note**: This course uses **Gazebo Classic** (still standard for ROS2 Humble). Concepts translate to new Gazebo.

**What Gazebo simulates**:
- ✅ **Physics**: Gravity, friction, collisions, forces
- ✅ **Sensors**: Lidar, cameras, IMU, GPS, depth sensors
- ✅ **Actuators**: Motors, servos with realistic dynamics
- ✅ **Environments**: Terrains, objects, lighting
- ✅ **Multiple robots**: Swarms, multi-robot systems

**What it doesn't do**:
- ❌ **Generate code**: You write robot controllers
- ❌ **Design robots**: Use CAD, export to URDF/SDF
- ❌ **Perfect reality**: Approximations for speed

## Gazebo Architecture

### Components

```
┌─────────────────────────────────────────┐
│     Gazebo GUI (Client)                 │
│  - 3D View, Model Editor, Scene Tools  │
└────────────┬────────────────────────────┘
             │ Network/IPC
┌────────────▼────────────────────────────┐
│     Gazebo Server (gzserver)            │
│  - Physics Engine (ODE, Bullet, DART)   │
│  - Sensor Simulation                    │
│  - Plugin System                        │
└────────────┬────────────────────────────┘
             │
┌────────────▼────────────────────────────┐
│     ROS2 Integration                    │
│  - gazebo_ros packages                  │
│  - Plugins (sensors, controllers)       │
└─────────────────────────────────────────┘
```

**gzserver**: Physics simulation (headless possible)
**gzclient**: GUI for visualization (optional)
**Plugins**: Connect simulation to ROS2

## Installing Gazebo

### With ROS2

```bash
# Gazebo Classic comes with ROS2 desktop install
sudo apt install ros-humble-desktop

# Or install separately
sudo apt install ros-humble-gazebo-ros-pkgs

# Verify installation
gazebo --version
```

### Standalone Gazebo

```bash
# If you need standalone Gazebo
sudo apt install gazebo11

# Gazebo ROS integration
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Starting Gazebo

### Basic Launch

```bash
# Start Gazebo with GUI (server + client)
gazebo

# Start with empty world (fast loading)
gazebo --verbose

# Headless (server only, no GUI)
gzserver

# Just GUI (connect to running server)
gzclient

# Pause on start
gazebo --pause

# Specific world file
gazebo worlds/cafe.world
```

### From ROS2

```bash
# Using gazebo_ros package
ros2 launch gazebo_ros gazebo.launch.py

# With specific world
ros2 launch gazebo_ros gazebo.launch.py world:=worlds/cafe.world

# Verbose output for debugging
ros2 launch gazebo_ros gazebo.launch.py verbose:=true
```

## Gazebo Interface

### Main Window Components

```
┌─────────────────────────────────────────────────────┐
│  File  Edit  View  Camera  Window  Help             │ Menu Bar
├──────┬──────────────────────────────────────┬───────┤
│      │                                      │       │
│ Left │        3D Scene View                 │ Right │
│Panel │     (Main viewport)                  │ Panel │
│      │                                      │       │
│Models│                                      │ World │
│Insert│                                      │ Stats │
│      │                                      │       │
└──────┴──────────────────────────────────────┴───────┘
│  Time: 00:05.32  Real time factor: 0.98             │ Bottom Bar
└─────────────────────────────────────────────────────┘
```

### Left Panel: Insert Models

**Purpose**: Add objects to world

- **Simple Shapes**: Box, sphere, cylinder
- **Model Database**: Pre-made models (online)
- **Local Models**: Your custom models

**Usage**:
1. Click model in left panel
2. Click in scene to place
3. Drag to position

### Right Panel

**World Tab**: Scene hierarchy
- Lists all models in simulation
- Expand to see links/joints
- Right-click for options (delete, rename)

**Scene Tab**: Visual settings
- Grid visibility
- Sky, lighting

**Layers Tab**: Organize scene elements

**Stats Tab**: Performance metrics
- Real-time factor
- FPS
- Physics step time

### 3D Scene Controls

**Mouse navigation**:
- **Left-click drag**: Rotate view
- **Middle-click drag** (or Shift+Left): Pan view
- **Scroll wheel**: Zoom
- **Right-click**: Context menu

**Toolbar**:
- **Select**: Choose models
- **Translate**: Move models (XYZ arrows)
- **Rotate**: Rotate models
- **Scale**: Resize models

**Camera views**:
- Top-down, front, side via Camera menu
- Orthographic vs perspective

### Bottom Status Bar

**Time display**:
- **Sim Time**: Simulated time elapsed
- **Real Time**: Wall clock time elapsed
- **Real Time Factor**: Sim time / Real time
  - `1.0` = Real-time (1 sim second = 1 real second)
  - `0.5` = Half speed (simulation too complex)
  - `2.0` = 2x speed (simulation simple/fast computer)

**Controls**:
- ▶️ Play simulation
- ⏸ Pause
- 🔄 Reset (world to initial state)
- 🔄 Reset time only

## World Files (.world)

### What Are World Files?

**World file**: XML (SDF format) describing entire simulation environment

**Contents**:
- Physics settings
- Lighting
- Models and their positions
- Plugins

### Basic World File

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    <!-- Physics engine settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Scene settings -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

  </world>
</sdf>
```

### Loading World Files

```bash
# Command line
gazebo my_world.world

# From ROS2 launch
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/my_world.world
```

### Common World Modifications

**Change gravity** (e.g., moon gravity):
```xml
<gravity>0 0 -1.62</gravity>  <!-- Moon: 1.62 m/s² -->
```

**Disable real-time constraint**:
```xml
<physics>
  <real_time_update_rate>0</real_time_update_rate>
  <!-- Run as fast as possible -->
</physics>
```

**Add models to world**:
```xml
<model name="my_box">
  <pose>1 2 0.5 0 0 0</pose>  <!-- x y z roll pitch yaw -->
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
      </material>
    </visual>
  </link>
</model>
```

## Spawning Models

### From Model Database

**Online model database**: Gazebo downloads models automatically

```bash
# In running Gazebo:
# Insert tab → Search "coke can" → Click to place
```

**Programmatically**:
```bash
# Spawn model from database
gz model --spawn-file=/path/to/model.sdf --model-name=my_model
```

### From URDF

**Spawn robot from URDF file**:

```bash
# Using gazebo_ros spawn_entity node
ros2 run gazebo_ros spawn_entity.py \
  -entity robot1 \
  -file /path/to/robot.urdf \
  -x 0 -y 0 -z 0.5
```

**In launch file**:
```python
from launch_ros.actions import Node

spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'my_robot',
        '-topic', '/robot_description',  # Or -file for direct file
        '-x', '0',
        '-y', '0',
        '-z', '0.5'
    ],
    output='screen'
)
```

## Physics Configuration

### Physics Engines

**Supported engines**:
- **ODE** (Open Dynamics Engine): Default, general purpose
- **Bullet**: Better collision detection
- **DART**: Advanced constraints
- **Simbody**: Biomechanics (rarely used)

**Choosing engine** (in world file):
```xml
<physics type="ode">  <!-- or "bullet", "dart" -->
  <!-- settings -->
</physics>
```

### Key Physics Parameters

**Update rate and step size**:
```xml
<physics>
  <!-- Physics calculated 1000 times per second -->
  <real_time_update_rate>1000</real_time_update_rate>

  <!-- Each physics step simulates 1ms -->
  <max_step_size>0.001</max_step_size>

  <!-- Try to run at real-time speed -->
  <real_time_factor>1.0</real_time_factor>
</physics>
```

**Trade-offs**:
- **Smaller step size**: More accurate, slower
- **Larger step size**: Faster, less stable (objects may penetrate)
- **Higher update rate**: Smoother, more CPU intensive

**Example values**:
```xml
<!-- Fast but less accurate -->
<max_step_size>0.01</max_step_size>
<real_time_update_rate>100</real_time_update_rate>

<!-- Accurate but slower -->
<max_step_size>0.0001</max_step_size>
<real_time_update_rate>10000</real_time_update_rate>
```

### Gravity

```xml
<!-- Earth gravity (default) -->
<gravity>0 0 -9.81</gravity>

<!-- Zero gravity (space) -->
<gravity>0 0 0</gravity>

<!-- Mars -->
<gravity>0 0 -3.71</gravity>
```

## Command-Line Tools

### Gazebo (gz) Command

**Service calls**:
```bash
# List running services
gz service --list

# Get world properties
gz world --info

# Pause simulation
gz world --pause=1

# Resume
gz world --pause=0

# Reset world
gz world --reset-all
```

**Topic inspection**:
```bash
# List topics
gz topic --list

# Echo topic data
gz topic --echo /gazebo/default/pose/info

# Get message info
gz topic --info /gazebo/default/world_stats
```

**Model commands**:
```bash
# Spawn model
gz model --spawn-file=model.sdf --model-name=my_model

# Delete model
gz model --model-name=my_model --delete

# Move model
gz model --model-name=my_box --pose "1,2,0.5,0,0,0"
```

## ROS2 Integration

### Gazebo ROS Packages

**Key packages**:
- `gazebo_ros`: Core integration (launch files, node)
- `gazebo_plugins`: ROS plugins for sensors, actuators
- `gazebo_msgs`: ROS2 message types for Gazebo

**Topics published by Gazebo**:
```bash
# Clock (simulation time)
/clock

# Model states
/gazebo/model_states

# Link states
/gazebo/link_states
```

### Services

```bash
# List Gazebo services
ros2 service list | grep gazebo

# Spawn entity
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity ...

# Delete entity
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'my_robot'}"

# Get model state
ros2 service call /gazebo/get_model_state gazebo_msgs/srv/GetModelState "{model_name: 'my_robot'}"

# Set model state (teleport)
ros2 service call /gazebo/set_model_state gazebo_msgs/srv/SetModelState ...
```

### Clock Topic

**Simulation time**:
```bash
# Gazebo publishes simulation time
ros2 topic echo /clock

# ROS2 nodes should use simulation time
ros2 run my_package my_node --ros-args --use-sim-time true
```

**In launch file**:
```python
Node(
    package='my_package',
    executable='my_node',
    parameters=[{'use_sim_time': True}]
)
```

## Creating Custom Worlds

### Directory Structure

```
my_robot_gazebo/
├── worlds/
│   ├── empty.world
│   └── my_test_world.world
├── models/
│   └── custom_obstacle/
│       ├── model.config
│       └── model.sdf
└── launch/
    └── gazebo.launch.py
```

### Example Custom World

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="test_world">

    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <include><uri>model://sun</uri></include>

    <!-- Ground -->
    <include><uri>model://ground_plane</uri></include>

    <!-- Add walls -->
    <model name="wall_1">
      <static>true</static>
      <pose>5 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 10 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.2 10 2</size></box></geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacles -->
    <model name="box_obstacle">
      <static>true</static>
      <pose>2 2 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
          <material>
            <ambient>1 0 0 1</ambient>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### Launch File for Custom World

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    pkg_dir = get_package_share_directory('my_robot_gazebo')
    world_file = os.path.join(pkg_dir, 'worlds', 'my_test_world.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
        )
    ])
```

## Performance Tips

**1. Simplify collision geometry**:
```xml
<!-- Complex visual, simple collision -->
<visual><geometry>
  <mesh filename="detailed_mesh.dae"/>
</geometry></visual>

<collision><geometry>
  <box size="1 1 1"/>  <!-- Simple approximation -->
</geometry></collision>
```

**2. Use static models when possible**:
```xml
<model name="building">
  <static>true</static>  <!-- No physics calculations -->
  <!-- ... -->
</model>
```

**3. Reduce physics update rate** (if acceptable):
```xml
<physics>
  <real_time_update_rate>100</real_time_update_rate>
  <max_step_size>0.01</max_step_size>
</physics>
```

**4. Disable shadows** (GPU intensive):
```xml
<scene>
  <shadows>false</shadows>
</scene>
```

**5. Run headless for automated testing**:
```bash
# Server only, no GUI rendering
gzserver my_world.world
```

## Troubleshooting

**Gazebo won't start**:
```bash
# Check for conflicting processes
killall -9 gzserver gzclient

# Clear cache
rm -rf ~/.gazebo/

# Reinstall
sudo apt install --reinstall gazebo11
```

**Poor performance**:
- Reduce physics update rate
- Simplify collision meshes
- Disable shadows
- Close unused programs

**Models falling through ground**:
- Check collision geometry exists
- Verify mass > 0
- Reduce physics step size

**Robot explodes on contact**:
- Physics step too large
- Inertia values incorrect
- Check for overlapping collision geometries

## Summary

**Gazebo provides**:
- High-fidelity physics simulation
- Sensor simulation (lidar, cameras, IMU)
- ROS2 integration via plugins
- Customizable environments

**Key components**:
- **gzserver**: Physics engine (can run headless)
- **gzclient**: GUI visualization
- **World files**: Environment descriptions (SDF)
- **Plugins**: Connect to ROS2

**Basic workflow**:
1. Create/load world file
2. Spawn robot (from URDF)
3. Add sensors via plugins
4. Run robot controllers
5. Visualize in Gazebo and RViz

**Important settings**:
- Physics update rate and step size
- Real-time factor
- Gravity
- use_sim_time for ROS2 nodes

## What's Next?

**Next Lesson**: World Building - Create complex environments

**You'll learn**:
- Building custom models
- Creating terrain
- Adding lighting and materials
- Organizing model libraries
- Importing CAD models

---

**Key Takeaway**: Gazebo brings your robot to life in a physics-based virtual world - master it to test safely before hardware!
