# URDF Robot Example

Demonstrates URDF and XACRO robot descriptions with a simple mobile robot.

## What This Demonstrates

- **Raw URDF**: Basic robot description without macros
- **XACRO**: Modular robot description with properties and macros
- **robot_state_publisher**: Publishing robot state to TF
- **joint_state_publisher_gui**: Interactive joint control

## Package Contents

### URDF Files

**simple_robot.urdf**
- Raw URDF file (no XACRO)
- Mobile robot with 2 wheels + caster
- Shows verbose URDF structure

**robot.urdf.xacro**
- Main XACRO file using properties and macros
- Demonstrates modular organization
- Same robot as simple_robot.urdf but with XACRO benefits

**properties.xacro**
- Dimensions, masses, colors
- Shows parameter organization

**macros.xacro**
- Inertia calculation macros
- Wheel macro (reusable component)
- Demonstrates DRY principle

## Quick Start

### Display Simple URDF

```bash
# Build
colcon build --packages-select urdf_robot_example
source install/setup.bash

# Launch
ros2 launch urdf_robot_example display_simple.launch.py
```

**Result**: Robot appears in RViz, joint_state_publisher_gui allows wheel rotation

### Display XACRO Robot

```bash
ros2 launch urdf_robot_example display_xacro.launch.py
```

**Result**: Same robot, but built from XACRO (processed at launch time)

## Comparing URDF vs XACRO

**URDF (simple_robot.urdf)**:
- ✅ Simple, explicit
- ❌ Repetitive (left/right wheels nearly identical)
- ❌ Hard to change dimensions (edit multiple places)

**XACRO (robot.urdf.xacro)**:
- ✅ DRY - wheel defined once, used twice
- ✅ Parameterized - change `wheel_radius` in one place
- ✅ Modular - organized into properties/macros files
- ✅ Calculated values - `${base_length/2}` for caster position

## Key Concepts

**Links**: Rigid bodies (base_link, wheels, caster)
**Joints**: Connections between links (continuous for wheels, fixed for caster)
**Inertia**: Mass distribution for physics simulation
**XACRO Macros**: Reusable templates (wheel macro)
**XACRO Properties**: Variables for dimensions/colors

## Related Lessons

- Module 4, Lesson 3: URDF Fundamentals
- Module 4, Lesson 5: XACRO and Macros

## Manual Testing

```bash
# Check URDF is valid
check_urdf simple_robot.urdf

# Convert XACRO to URDF
xacro robot.urdf.xacro > robot_generated.urdf

# View TF tree
ros2 run tf2_tools view_frames
```
