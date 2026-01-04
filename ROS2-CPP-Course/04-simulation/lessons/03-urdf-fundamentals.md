# Lesson 3: URDF Fundamentals

## Learning Objectives

- Understand URDF structure and purpose
- Define robot links and joints
- Configure visual and collision geometry
- Set mass and inertia properties
- Create complete robot descriptions
- Use robot_state_publisher
- Validate URDF files

## What is URDF?

**URDF (Unified Robot Description Format)**: XML-based format for describing robot structure, geometry, and properties.

**Purpose**: Standard way to represent robots in ROS2 for visualization, simulation, and control.

**Used by**:
- RViz (visualization)
- Gazebo (simulation)
- MoveIt (motion planning)
- Navigation stack
- Custom robot controllers

## URDF File Structure

### Basic Template

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Links (rigid bodies) -->
  <link name="base_link">
    <!-- Visual appearance -->
    <!-- Collision geometry -->
    <!-- Inertial properties -->
  </link>

  <link name="wheel_link">
    <!-- ... -->
  </link>

  <!-- Joints (connections between links) -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <!-- Position and orientation -->
    <!-- Axis of rotation/movement -->
    <!-- Limits and dynamics -->
  </joint>

</robot>
```

### Key Components

**1. Robot Tag**
```xml
<robot name="robot_name">
  <!-- All content here -->
</robot>
```

**2. Links** - Rigid bodies
```xml
<link name="link_name">
  <visual>     <!-- How it looks -->
  <collision>  <!-- Physics boundaries -->
  <inertial>   <!-- Mass properties -->
</link>
```

**3. Joints** - Connections between links
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

## Links in Detail

### Link Structure

A link represents a rigid body in the robot.

```xml
<link name="base_link">

  <!-- VISUAL: How the link appears in RViz/Gazebo -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- COLLISION: Shape used for physics collisions -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
  </collision>

  <!-- INERTIAL: Mass and inertia for physics -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="10.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0"
             iyy="0.1" iyz="0.0"
             izz="0.1"/>
  </inertial>

</link>
```

### Visual Element

**Purpose**: Defines appearance for visualization

**Components**:
- `<origin>`: Position and orientation relative to link frame
- `<geometry>`: Shape (box, cylinder, sphere, mesh)
- `<material>`: Color or texture

**Example shapes**:

```xml
<!-- Box -->
<geometry>
  <box size="length width height"/>
</geometry>

<!-- Cylinder -->
<geometry>
  <cylinder radius="0.1" length="0.5"/>
</geometry>

<!-- Sphere -->
<geometry>
  <sphere radius="0.1"/>
</geometry>

<!-- Mesh (STL, DAE file) -->
<geometry>
  <mesh filename="package://my_robot/meshes/base.stl" scale="1 1 1"/>
</geometry>
```

### Collision Element

**Purpose**: Defines shape for collision detection in simulation

**Best Practice**: Use simpler geometry than visual for performance

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Simplified shape -->
    <box size="0.5 0.3 0.2"/>
  </geometry>
</collision>
```

**Why different from visual?**
- **Visual**: Can be detailed mesh for appearance
- **Collision**: Should be simple shapes (box, cylinder) for fast physics

**Example**:
```xml
<visual>
  <geometry>
    <!-- Complex mesh with 10,000 triangles -->
    <mesh filename="package://my_robot/meshes/chassis_detailed.stl"/>
  </geometry>
</visual>

<collision>
  <geometry>
    <!-- Simple box approximation -->
    <box size="0.6 0.4 0.2"/>
  </geometry>
</collision>
```

### Inertial Properties

**Purpose**: Mass distribution for accurate physics simulation

**Required for**: Gazebo simulation

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- Center of mass -->
  <mass value="10.0"/>                <!-- kg -->
  <inertia ixx="0.1" ixy="0.0" ixz="0.0"
           iyy="0.1" iyz="0.0"
           izz="0.1"/>
</inertial>
```

**Inertia matrix**: Describes rotational resistance

**Calculating inertia**:

For common shapes:

```python
# Box (l=length, w=width, h=height, m=mass)
ixx = (m/12) * (h^2 + w^2)
iyy = (m/12) * (h^2 + l^2)
izz = (m/12) * (l^2 + w^2)

# Cylinder (r=radius, h=height, m=mass, z-axis)
ixx = iyy = (m/12) * (3*r^2 + h^2)
izz = (m/2) * r^2

# Sphere (r=radius, m=mass)
ixx = iyy = izz = (2/5) * m * r^2
```

**Tools**:
```bash
# Use MeshLab to calculate from 3D mesh
# Or online calculators for standard shapes
```

## Joints in Detail

### Joint Types

**1. Fixed** - No movement (rigid connection)
```xml
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.3 0 0.2" rpy="0 0 0"/>
</joint>
```

**2. Revolute** - Rotation with limits
```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Y-axis rotation -->
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
</joint>
```

**3. Continuous** - Rotation without limits (wheels)
```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0 0.15 0" rpy="-1.5708 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

**4. Prismatic** - Linear motion (slider)
```xml
<joint name="slider_joint" type="prismatic">
  <parent link="base"/>
  <child link="platform"/>
  <axis xyz="0 0 1"/>  <!-- Z-axis movement -->
  <limit lower="0" upper="0.5" effort="100" velocity="0.5"/>
</joint>
```

**5. Floating** - 6-DOF (rarely used)

**6. Planar** - 2D motion (rarely used)

### Joint Properties

**Origin**:
```xml
<origin xyz="x y z" rpy="roll pitch yaw"/>
```
- `xyz`: Position of child relative to parent (meters)
- `rpy`: Orientation as roll-pitch-yaw (radians)

**Axis**:
```xml
<axis xyz="0 0 1"/>
```
- Unit vector defining rotation/translation direction
- Common: `1 0 0` (X), `0 1 0` (Y), `0 0 1` (Z)

**Limits** (for revolute/prismatic):
```xml
<limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
```
- `lower`, `upper`: Joint range (rad or m)
- `effort`: Maximum force/torque (N or N⋅m)
- `velocity`: Maximum speed (rad/s or m/s)

**Dynamics** (optional):
```xml
<dynamics damping="0.1" friction="0.1"/>
```

## Materials and Colors

### Define Material

```xml
<material name="blue">
  <color rgba="0 0 1 1"/>  <!-- Red Green Blue Alpha (0-1) -->
</material>

<material name="red">
  <color rgba="1 0 0 1"/>
</material>
```

### Use Material

```xml
<visual>
  <geometry>
    <box size="0.5 0.3 0.2"/>
  </geometry>
  <material name="blue"/>
</visual>
```

### Texture (Gazebo)

```xml
<material name="textured">
  <texture filename="package://my_robot/textures/metal.png"/>
</material>
```

## Complete Robot Example

### Simple Mobile Robot

```xml
<?xml version="1.0"?>
<robot name="simple_bot">

  <!-- Base Link (main chassis) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.6 0.4 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.6 0.4 0.1"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="5.0"/>
      <inertia ixx="0.0833" ixy="0" ixz="0"
               iyy="0.1583" iyz="0"
               izz="0.225"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.00125" ixy="0" ixz="0"
               iyy="0.00125" iyz="0"
               izz="0.0025"/>
    </inertial>
  </link>

  <!-- Left Wheel Joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.225 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Right Wheel (similar to left) -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.00125" ixy="0" ixz="0"
               iyy="0.00125" iyz="0"
               izz="0.0025"/>
    </inertial>
  </link>

  <!-- Right Wheel Joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.225 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Caster Wheel (simple sphere) -->
  <link name="caster">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0"
               izz="0.0001"/>
    </inertial>
  </link>

  <!-- Caster Joint -->
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster"/>
    <origin xyz="-0.25 0 -0.05" rpy="0 0 0"/>
  </joint>

</robot>
```

## Using robot_state_publisher

### What It Does

**robot_state_publisher**: ROS2 node that:
1. Reads robot URDF
2. Listens to joint states
3. Publishes TF transforms for all links

### Launch with robot_state_publisher

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf',
        'robot.urdf'
    )

    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher
    ])
```

### Viewing in RViz

```bash
# Terminal 1: Launch robot state publisher
ros2 launch my_robot robot_state.launch.py

# Terminal 2: Start RViz
rviz2

# In RViz:
# 1. Set Fixed Frame: base_link
# 2. Add → RobotModel
# 3. Add → TF (to see coordinate frames)
```

## URDF Tools and Validation

### Check URDF Syntax

```bash
# Install check_urdf tool
sudo apt install liburdfdom-tools

# Validate URDF file
check_urdf robot.urdf
```

**Output (if valid)**:
```
robot name is: simple_bot
---------- Successfully Parsed XML ---------------
root Link: base_link has 3 child(ren)
    child(1):  left_wheel
    child(2):  right_wheel
    child(3):  caster
```

### Visualize URDF Tree

```bash
# Generate PDF of robot structure
urdf_to_graphiz robot.urdf
# Creates robot.pdf showing link/joint hierarchy
```

### View in RViz Directly

```bash
# Quick visualization without launch file
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat robot.urdf)"

# In another terminal
rviz2
```

## Common URDF Patterns

### Link with Multiple Visual Elements

```xml
<link name="robot_base">
  <!-- Main chassis -->
  <visual>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
    <material name="blue"/>
  </visual>

  <!-- Decoration (antenna) -->
  <visual>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.01" length="0.1"/>
    </geometry>
    <material name="red"/>
  </visual>

  <!-- Single collision for entire link -->
  <collision>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.05" ixy="0" ixz="0"
             iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

### Sensor Mounting

```xml
<!-- Lidar sensor -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
    <material name="black"/>
  </visual>

  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.001" ixy="0" ixz="0"
             iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Fixed to robot top -->
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
</joint>
```

## Best Practices

**1. Coordinate Frames**
```
Standard conventions:
- X: forward
- Y: left
- Z: up
- Origin at robot center or base
```

**2. Units**
```
URDF standard units:
- Length: meters
- Mass: kilograms
- Angle: radians
- Force: Newtons
- Torque: Newton-meters
```

**3. Link Naming**
```
Clear, descriptive names:
✅ base_link, left_wheel, camera_link
❌ link1, link2, part_a
```

**4. Collision Simplification**
```xml
<!-- Visual: detailed mesh -->
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/detailed_chassis.stl"/>
  </geometry>
</visual>

<!-- Collision: simple approximation -->
<collision>
  <geometry>
    <box size="0.6 0.4 0.2"/>
  </geometry>
</collision>
```

**5. Inertia Accuracy**
```
For Gazebo simulation:
- Calculate properly (don't use dummy values)
- Match mass distribution
- Use physics tools or calculators
```

**6. File Organization**
```
my_robot_description/
├── urdf/
│   └── robot.urdf
├── meshes/
│   ├── chassis.stl
│   └── wheel.stl
├── textures/
│   └── metal.png
└── launch/
    └── display.launch.py
```

## Common Pitfalls

**1. Missing parent/child in joints**
```xml
<!-- ❌ Wrong -->
<joint name="wheel_joint" type="continuous">
  <!-- Missing parent/child! -->
  <axis xyz="0 0 1"/>
</joint>

<!-- ✅ Correct -->
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <axis xyz="0 0 1"/>
</joint>
```

**2. Zero or missing mass**
```xml
<!-- ❌ Wrong: Will cause Gazebo errors -->
<inertial>
  <mass value="0"/>
  <!-- ... -->
</inertial>

<!-- ✅ Correct -->
<inertial>
  <mass value="0.5"/>
  <!-- ... -->
</inertial>
```

**3. Incorrect axis for joints**
```xml
<!-- For wheel rotating around Y-axis -->
<!-- ❌ Wrong -->
<axis xyz="0 0 1"/>  <!-- Z-axis -->

<!-- ✅ Correct -->
<axis xyz="0 1 0"/>  <!-- Y-axis -->
```

**4. Missing collision geometry**
```xml
<!-- ❌ Wrong: Robot will fall through world in Gazebo -->
<link name="base">
  <visual>...</visual>
  <!-- No collision! -->
  <inertial>...</inertial>
</link>

<!-- ✅ Correct -->
<link name="base">
  <visual>...</visual>
  <collision>...</collision>
  <inertial>...</inertial>
</link>
```

## Debugging URDF Issues

### RViz Not Showing Robot

**Check**:
```bash
# 1. Is robot_description published?
ros2 topic list | grep robot_description

# 2. Is robot_state_publisher running?
ros2 node list | grep robot_state_publisher

# 3. Check URDF validity
check_urdf robot.urdf

# 4. View TF tree
ros2 run tf2_tools view_frames
```

### Gazebo Crashes or Warnings

**Common issues**:
```
Error: "link has zero mass"
Solution: Add proper <inertial> with mass > 0

Error: "inertia matrix not positive definite"
Solution: Use proper inertia calculations

Warning: "No collision geometry"
Solution: Add <collision> to all non-fixed links
```

## Summary

**URDF describes**:
- Robot structure (links and joints)
- Visual appearance (how it looks)
- Collision geometry (for physics)
- Inertial properties (mass and inertia)

**Key components**:
- **Links**: Rigid bodies with visual/collision/inertial
- **Joints**: Connections (fixed, revolute, continuous, prismatic)
- **Materials**: Colors and textures
- **Origin**: Position and orientation (xyz, rpy)

**Workflow**:
1. Design robot structure (sketch link/joint hierarchy)
2. Write URDF file
3. Validate with `check_urdf`
4. Visualize with robot_state_publisher + RViz
5. Test in Gazebo (next lessons)

**Tools**:
- `robot_state_publisher`: Publishes TF from URDF
- `check_urdf`: Validates syntax
- `urdf_to_graphiz`: Visualizes structure
- RViz: View robot model

## What's Next?

**Next Lesson**: Visualization Markers - Custom shapes and annotations in RViz

**You'll learn**:
- Creating markers programmatically
- Interactive markers
- Path and trajectory visualization
- Debug overlays

---

**Key Takeaway**: URDF is the blueprint of your robot. Master it to create accurate simulations and enable advanced ROS2 features!
