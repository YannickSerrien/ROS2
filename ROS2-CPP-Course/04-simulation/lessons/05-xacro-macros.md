# Lesson 5: XACRO and Macros

## Learning Objectives

- Understand limitations of raw URDF
- Use XACRO to create parameterized robot descriptions
- Define and use macros for reusable components
- Use properties and math expressions
- Implement conditional inclusion
- Convert between XACRO and URDF
- Organize robot descriptions into modular files

## Why XACRO?

### Problems with Raw URDF

**URDF is verbose and repetitive**:

```xml
<!-- Four wheels means copying similar code 4 times! -->
<link name="front_left_wheel">
  <visual>
    <geometry><cylinder radius="0.1" length="0.05"/></geometry>
    <material name="black"><color rgba="0.2 0.2 0.2 1"/></material>
  </visual>
  <!-- More repetition... -->
</link>

<link name="front_right_wheel">
  <visual>
    <geometry><cylinder radius="0.1" length="0.05"/></geometry>
    <material name="black"><color rgba="0.2 0.2 0.2 1"/></material>
  </visual>
  <!-- More repetition... -->
</link>

<!-- Two more wheels with nearly identical code... -->
```

**Issues**:
- 🔁 **Repetitive**: Copy-paste similar components
- 🔧 **Hard to maintain**: Change wheel size? Edit 4 places!
- ❌ **Error-prone**: Easy to make typos in copies
- 📏 **No parameters**: Can't easily resize robot
- 🚫 **No math**: Manual calculation of positions

### XACRO Solution

**XACRO (XML Macros)**: Preprocessor for URDF that adds:
- **Variables** (properties)
- **Math expressions**
- **Macros** (reusable templates)
- **Conditional logic**
- **File includes**

**Same robot with XACRO**:
```xml
<!-- Define wheel once as macro -->
<xacro:macro name="wheel" params="prefix reflect">
  <link name="${prefix}_wheel">
    <visual>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel"/>
    <origin xyz="0 ${reflect * wheel_offset_y} 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</xacro:macro>

<!-- Use macro 4 times with different parameters -->
<xacro:wheel prefix="front_left" reflect="1"/>
<xacro:wheel prefix="front_right" reflect="-1"/>
<xacro:wheel prefix="rear_left" reflect="1"/>
<xacro:wheel prefix="rear_right" reflect="-1"/>
```

**Benefits**: Change wheel size once, affects all wheels!

## XACRO Basics

### File Structure

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Properties (variables) -->
  <xacro:property name="wheel_radius" value="0.1"/>

  <!-- Macros (templates) -->
  <xacro:macro name="wheel" params="prefix">
    <!-- Macro content -->
  </xacro:macro>

  <!-- Use macros -->
  <xacro:wheel prefix="left"/>

  <!-- Regular URDF content -->
  <link name="base_link">
    <!-- ... -->
  </link>

</robot>
```

**Key difference from URDF**:
- Namespace declaration: `xmlns:xacro="http://www.ros.org/wiki/xacro"`
- XACRO tags: `<xacro:property>`, `<xacro:macro>`, etc.

### Properties (Variables)

**Define constants**:
```xml
<!-- Simple values -->
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:property name="base_length" value="0.6"/>
<xacro:property name="base_width" value="0.4"/>

<!-- Colors -->
<xacro:property name="robot_color" value="0.2 0.2 0.8 1"/>
```

**Use properties**:
```xml
<!-- With ${} syntax -->
<geometry>
  <box size="${base_length} ${base_width} 0.1"/>
</geometry>

<geometry>
  <cylinder radius="${wheel_radius}" length="0.05"/>
</geometry>

<color rgba="${robot_color}"/>
```

### Math Expressions

**Calculations in ${} blocks**:
```xml
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:property name="wheel_separation" value="0.5"/>

<!-- Math operations -->
<origin xyz="0 ${wheel_separation / 2} ${-wheel_radius}"/>

<!-- Complex expressions -->
<xacro:property name="base_height" value="0.1"/>
<xacro:property name="sensor_height" value="${base_height + 0.15}"/>

<!-- Constants available -->
<xacro:property name="half_pi" value="${pi / 2}"/>
```

**Available operations**:
- Arithmetic: `+`, `-`, `*`, `/`
- Constants: `pi` (3.14159...)
- Parentheses for grouping: `${(a + b) * c}`

## Macros

### Basic Macro

**Define macro**:
```xml
<xacro:macro name="box_inertia" params="m x y z">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${m * (y*y + z*z) / 12}" ixy="0" ixz="0"
             iyy="${m * (x*x + z*z) / 12}" iyz="0"
             izz="${m * (x*x + y*y) / 12}"/>
  </inertial>
</xacro:macro>
```

**Use macro**:
```xml
<!-- Call with parameters -->
<link name="base_link">
  <xacro:box_inertia m="10" x="0.6" y="0.4" z="0.2"/>

  <visual>
    <geometry><box size="0.6 0.4 0.2"/></geometry>
  </visual>
</link>
```

### Macro with Block Parameters

**Pass XML blocks as parameters**:
```xml
<xacro:macro name="link_with_mesh" params="name *visual_origin *collision_geometry">
  <link name="${name}">
    <visual>
      <xacro:insert_block name="visual_origin"/>
      <geometry>
        <mesh filename="package://my_robot/meshes/${name}.stl"/>
      </geometry>
    </visual>

    <collision>
      <xacro:insert_block name="collision_geometry"/>
    </collision>
  </link>
</xacro:macro>

<!-- Use with block parameters (note *) -->
<xacro:link_with_mesh name="chassis">
  <visual_origin>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </visual_origin>
  <collision_geometry>
    <geometry><box size="0.6 0.4 0.2"/></geometry>
  </collision_geometry>
</xacro:link_with_mesh>
```

**Block parameters**: Prefix with `*` in params, use `<xacro:insert_block>` to include

### Complete Wheel Macro Example

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_with_wheels">

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="wheel_mass" value="0.5"/>
  <xacro:property name="base_to_wheel_y" value="0.225"/>

  <!-- Wheel macro -->
  <xacro:macro name="wheel" params="prefix reflect">

    <!-- Wheel link -->
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0.2 0.2 0.2 1"/>
        </material>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>

      <inertial>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <mass value="${wheel_mass}"/>
        <inertia ixx="${wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width) / 12}"
                 ixy="0" ixz="0"
                 iyy="${wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width) / 12}"
                 iyz="0"
                 izz="${wheel_mass * wheel_radius * wheel_radius / 2}"/>
      </inertial>
    </link>

    <!-- Wheel joint -->
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${reflect * base_to_wheel_y} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>

  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.6 0.4 0.1"/></geometry>
      <material name="blue"><color rgba="0 0 0.8 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.6 0.4 0.1"/></geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Use wheel macro for left and right wheels -->
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>

</robot>
```

**Result**: Two wheels with mirrored Y positions!

## File Includes

### Splitting Large Descriptions

**Main file** (`robot.urdf.xacro`):
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Include property files -->
  <xacro:include filename="$(find my_robot)/urdf/properties.xacro"/>

  <!-- Include macro libraries -->
  <xacro:include filename="$(find my_robot)/urdf/wheel_macro.xacro"/>
  <xacro:include filename="$(find my_robot)/urdf/sensor_macro.xacro"/>

  <!-- Robot structure -->
  <link name="base_link">
    <!-- ... -->
  </link>

  <!-- Use macros from included files -->
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>

  <xacro:lidar_sensor parent="base_link"/>

</robot>
```

**Properties file** (`properties.xacro`):
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Dimensions -->
  <xacro:property name="base_length" value="0.6"/>
  <xacro:property name="base_width" value="0.4"/>
  <xacro:property name="wheel_radius" value="0.1"/>

  <!-- Colors -->
  <xacro:property name="robot_blue" value="0.2 0.2 0.8 1"/>
  <xacro:property name="wheel_black" value="0.2 0.2 0.2 1"/>
</robot>
```

**Wheel macro file** (`wheel_macro.xacro`):
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="wheel" params="prefix reflect">
    <!-- Wheel definition -->
  </xacro:macro>
</robot>
```

**Benefit**: Organize complex robots into manageable modules!

## Conditional Inclusion

### Using xacro:if

**Conditional blocks**:
```xml
<!-- Property to enable/disable feature -->
<xacro:property name="use_gpu_lidar" value="true"/>

<!-- Include different sensor based on property -->
<xacro:if value="${use_gpu_lidar}">
  <xacro:include filename="$(find my_robot)/urdf/gpu_lidar.xacro"/>
</xacro:if>

<xacro:unless value="${use_gpu_lidar}">
  <xacro:include filename="$(find my_robot)/urdf/cpu_lidar.xacro"/>
</xacro:unless>
```

### Conditional Macro Content

```xml
<xacro:macro name="robot_base" params="use_mesh:=false">
  <link name="base_link">

    <!-- Use mesh if enabled, otherwise use simple box -->
    <xacro:if value="${use_mesh}">
      <visual>
        <geometry>
          <mesh filename="package://my_robot/meshes/base.stl"/>
        </geometry>
      </visual>
    </xacro:if>

    <xacro:unless value="${use_mesh}">
      <visual>
        <geometry>
          <box size="0.6 0.4 0.2"/>
        </geometry>
      </visual>
    </xacro:unless>

    <!-- Collision always uses simple geometry -->
    <collision>
      <geometry><box size="0.6 0.4 0.2"/></geometry>
    </collision>

  </link>
</xacro:macro>

<!-- Use macro -->
<xacro:robot_base use_mesh="true"/>
```

## Processing XACRO to URDF

### Command Line

**Convert XACRO to URDF**:
```bash
# Process XACRO file
xacro robot.urdf.xacro > robot.urdf

# With substitution arguments
xacro robot.urdf.xacro use_gpu:=true > robot.urdf

# Check syntax
xacro --check robot.urdf.xacro

# View processed output (debug)
xacro robot.urdf.xacro
```

### In Launch Files

**Process at launch time**:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get XACRO file path
    xacro_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf',
        'robot.urdf.xacro'
    )

    # Process XACRO to robot description
    robot_description = xacro.process_file(xacro_file).toxml()

    # robot_state_publisher with processed URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    return LaunchDescription([robot_state_publisher])
```

**With arguments**:
```python
# Process XACRO with mappings (arguments)
robot_description = xacro.process_file(
    xacro_file,
    mappings={'use_gpu': 'true', 'robot_name': 'robot1'}
).toxml()
```

## Advanced Patterns

### Parameterized Inertia Macros

```xml
<!-- Cylinder inertia (for wheels) -->
<xacro:macro name="cylinder_inertia" params="m r h">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${m * (3*r*r + h*h) / 12}" ixy="0" ixz="0"
             iyy="${m * (3*r*r + h*h) / 12}" iyz="0"
             izz="${m * r * r / 2}"/>
  </inertial>
</xacro:macro>

<!-- Box inertia -->
<xacro:macro name="box_inertia" params="m x y z">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${m * (y*y + z*z) / 12}" ixy="0" ixz="0"
             iyy="${m * (x*x + z*z) / 12}" iyz="0"
             izz="${m * (x*x + y*y) / 12}"/>
  </inertial>
</xacro:macro>

<!-- Sphere inertia -->
<xacro:macro name="sphere_inertia" params="m r">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${2 * m * r * r / 5}" ixy="0" ixz="0"
             iyy="${2 * m * r * r / 5}" iyz="0"
             izz="${2 * m * r * r / 5}"/>
  </inertial>
</xacro:macro>
```

**Usage**:
```xml
<link name="base_link">
  <xacro:box_inertia m="10" x="0.6" y="0.4" z="0.2"/>
  <!-- visual, collision -->
</link>

<link name="wheel">
  <xacro:cylinder_inertia m="0.5" r="0.1" h="0.05"/>
  <!-- visual, collision -->
</link>
```

### Default Parameter Values

```xml
<!-- Macro with default parameters -->
<xacro:macro name="sensor" params="parent xyz:='0 0 0' rpy:='0 0 0'">
  <joint name="sensor_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="sensor_link"/>
    <origin xyz="${xyz}" rpy="${rpy}"/>
  </joint>

  <link name="sensor_link">
    <!-- ... -->
  </link>
</xacro:macro>

<!-- Use with defaults -->
<xacro:sensor parent="base_link"/>

<!-- Override defaults -->
<xacro:sensor parent="base_link" xyz="0.3 0 0.2" rpy="0 0 1.57"/>
```

**Syntax**: `param:='default_value'`

### Gazebo-Specific Tags

```xml
<xacro:macro name="diff_drive_controller" params="robot_name">
  <gazebo>
    <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/${robot_name}</namespace>
      </ros>

      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>

      <wheel_separation>0.45</wheel_separation>
      <wheel_diameter>${2 * wheel_radius}</wheel_diameter>

      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>

      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
</xacro:macro>

<!-- Use in robot -->
<xacro:diff_drive_controller robot_name="my_robot"/>
```

## Real-World Robot Example

### Modular Robot Structure

```
my_robot_description/
├── urdf/
│   ├── robot.urdf.xacro           # Main file
│   ├── properties.xacro           # Dimensions, colors
│   ├── materials.xacro            # Material definitions
│   ├── base.xacro                 # Chassis
│   ├── wheels.xacro               # Wheel macros
│   ├── sensors.xacro              # Sensor macros
│   └── gazebo.xacro               # Gazebo plugins
└── meshes/
    ├── base.stl
    └── wheel.stl
```

**Main file** (`robot.urdf.xacro`):
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mobile_robot">

  <!-- Include all modules -->
  <xacro:include filename="$(find my_robot_description)/urdf/properties.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/materials.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/base.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/wheels.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/sensors.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/gazebo.xacro"/>

  <!-- Build robot -->
  <xacro:robot_base/>

  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>

  <xacro:lidar parent="base_link" xyz="0.2 0 0.15"/>
  <xacro:camera parent="base_link" xyz="0.3 0 0.2"/>

  <xacro:diff_drive_controller/>

</robot>
```

**Benefits**:
- **Modular**: Each file has single responsibility
- **Reusable**: Macros work across projects
- **Maintainable**: Changes in one place
- **Scalable**: Easy to add features

## Best Practices

**1. Use Descriptive Property Names**
```xml
<!-- ✅ Good -->
<xacro:property name="base_length" value="0.6"/>
<xacro:property name="front_wheel_offset_x" value="0.2"/>

<!-- ❌ Bad -->
<xacro:property name="l" value="0.6"/>
<xacro:property name="x1" value="0.2"/>
```

**2. Group Related Properties**
```xml
<!-- Dimensions -->
<xacro:property name="base_length" value="0.6"/>
<xacro:property name="base_width" value="0.4"/>
<xacro:property name="base_height" value="0.2"/>

<!-- Masses -->
<xacro:property name="base_mass" value="10.0"/>
<xacro:property name="wheel_mass" value="0.5"/>
```

**3. Calculate Derived Values**
```xml
<!-- Define fundamental values -->
<xacro:property name="wheel_radius" value="0.1"/>

<!-- Calculate dependent values -->
<xacro:property name="wheel_diameter" value="${2 * wheel_radius}"/>
<xacro:property name="base_ground_clearance" value="${wheel_radius + 0.02}"/>
```

**4. Use Meaningful Macro Parameters**
```xml
<!-- ✅ Good -->
<xacro:macro name="wheel" params="prefix reflect">

<!-- ❌ Bad -->
<xacro:macro name="wheel" params="p r">
```

**5. Document Macros**
```xml
<!--
  Wheel macro: Creates wheel link and joint
  Parameters:
    - prefix: Wheel name prefix (e.g., "left", "right")
    - reflect: 1 for left side, -1 for right side (mirrors Y position)
-->
<xacro:macro name="wheel" params="prefix reflect">
  <!-- ... -->
</xacro:macro>
```

## Common Pitfalls

**Missing namespace declaration**:
```xml
<!-- ❌ Wrong -->
<robot name="my_robot">

<!-- ✅ Correct -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
```

**Forgetting ${} in expressions**:
```xml
<!-- ❌ Wrong: Literal text "wheel_radius", not value -->
<cylinder radius="wheel_radius"/>

<!-- ✅ Correct -->
<cylinder radius="${wheel_radius}"/>
```

**Circular property dependencies**:
```xml
<!-- ❌ Wrong: a depends on b, b depends on a -->
<xacro:property name="a" value="${b + 1}"/>
<xacro:property name="b" value="${a + 1}"/>
```

**Incorrect file path in include**:
```xml
<!-- ❌ Wrong: Relative path won't work -->
<xacro:include filename="../urdf/macros.xacro"/>

<!-- ✅ Correct: Use package substitution -->
<xacro:include filename="$(find my_robot)/urdf/macros.xacro"/>
```

## Summary

**XACRO solves URDF problems**:
- ✅ Eliminates repetition with macros
- ✅ Enables parameterization with properties
- ✅ Supports calculations with math expressions
- ✅ Organizes code with file includes
- ✅ Allows conditional logic

**Key features**:
- **Properties**: `<xacro:property name="x" value="1.0"/>`
- **Math**: `${property * 2 + pi/2}`
- **Macros**: Reusable templates with parameters
- **Includes**: Split into modular files
- **Conditionals**: `<xacro:if>`, `<xacro:unless>`

**Workflow**:
1. Design robot with repeated elements
2. Extract common patterns into macros
3. Define properties for dimensions
4. Split into logical files
5. Process with `xacro` command or in launch file

**Best practices**:
- Use descriptive names
- Calculate derived values
- Document macros
- Organize into modules
- Test processed URDF

## What's Next?

**Next Lesson**: Gazebo Basics - Physics simulation fundamentals

**You'll learn**:
- Starting Gazebo
- World files and environments
- Spawning robots
- Basic physics configuration
- Gazebo GUI and controls

---

**Key Takeaway**: XACRO transforms URDF from verbose XML into maintainable, parameterized robot descriptions!
