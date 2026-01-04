# Lesson 2: RViz Basics

## Learning Objectives

- Understand RViz interface and components
- Configure displays for robot visualization
- Visualize TF frames and transforms
- Display sensor data (laser scans, point clouds)
- Save and load RViz configurations
- Use RViz for debugging

## What is RViz?

**RViz (ROS Visualization)**: 3D visualization tool for displaying robot state, sensor data, and environment information in real-time.

**Key Purpose**: Visual debugging and monitoring of ROS2 systems

**Not a simulator**: RViz only displays data - it doesn't generate it or run physics.

## RViz Interface

### Main Components

```
┌─────────────────────────────────────────────┐
│  Menu Bar (File, View, Panels, Help)       │
├──────────┬──────────────────────┬───────────┤
│          │                      │  Tool     │
│ Displays │  3D View             │  Properties│
│  Panel   │  (Main viewport)     │  Panel    │
│          │                      │           │
│  - Add   │                      │           │
│  - Robot │                      │           │
│  - TF    │                      │           │
│  - Laser │                      │           │
└──────────┴──────────────────────┴───────────┘
│  Status Bar (FPS, Time, Messages)          │
└─────────────────────────────────────────────┘
```

### Key Panels

**1. Displays Panel (Left)**
- Add/remove visualizations
- Configure display properties
- Enable/disable displays
- Organize with topics

**2. 3D View (Center)**
- Main visualization window
- Camera controls (orbit, pan, zoom)
- Multiple viewpoints
- Interactive elements

**3. Tool Properties (Right)**
- Current tool settings
- Selection information
- Camera configuration

## Starting RViz

### Basic Launch

```bash
# Start RViz with default config
ros2 run rviz2 rviz2

# Start with specific config file
ros2 run rviz2 rviz2 -d /path/to/config.rviz

# With custom display config
rviz2 -d my_robot.rviz
```

### From Launch File

```python
from launch_ros.actions import Node

rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', '/path/to/config.rviz']
)
```

## Essential Displays

### 1. RobotModel Display

**Purpose**: Visualize robot structure from URDF

**Configuration**:
1. Click **Add** button in Displays panel
2. Select **RobotModel**
3. Set **Description Topic**: `/robot_description`

**What it shows**:
- Robot links (visual geometry)
- Current joint positions
- Robot coordinate frames

**Requirements**:
- `robot_state_publisher` node running
- URDF loaded on `/robot_description` topic

**Example**:
```bash
# Terminal 1: Publish robot description
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro my_robot.urdf.xacro)"

# Terminal 2: Launch RViz
rviz2
# Add RobotModel display
```

### 2. TF Display

**Purpose**: Visualize coordinate frame transformations

**Configuration**:
1. Add **TF** display
2. Enable **Show Names** to see frame labels
3. Enable **Show Axes** to see coordinate axes
4. Adjust **Marker Scale** for visibility

**What it shows**:
- All active coordinate frames
- Transform relationships (parent→child)
- Frame orientations (XYZ axes)

**Useful for**:
- Debugging TF trees
- Verifying frame relationships
- Checking transform updates

**Color Convention**:
- **Red**: X-axis
- **Green**: Y-axis
- **Blue**: Z-axis

### 3. LaserScan Display

**Purpose**: Visualize 2D laser/lidar data

**Configuration**:
1. Add **LaserScan** display
2. Set **Topic**: `/scan`
3. Choose **Color Transformer**: Intensity, FlatColor, etc.
4. Set **Size**: Point size in meters

**What it shows**:
- Laser measurement points
- Distance to obstacles
- Scan field of view

**Example**:
```bash
# Terminal 1: Run lidar node (real or simulated)
ros2 run lidar_driver lidar_node

# Terminal 2: RViz
rviz2
# Add LaserScan, set topic to /scan
```

### 4. PointCloud2 Display

**Purpose**: Visualize 3D point cloud data

**Configuration**:
1. Add **PointCloud2** display
2. Set **Topic**: `/camera/depth/points`
3. Choose **Style**: Points, Flat Squares, Spheres
4. Set **Color Transformer**: RGB8, Intensity, AxisColor

**Use cases**:
- Depth camera visualization
- 3D mapping (SLAM)
- Object recognition

### 5. Image Display

**Purpose**: Show camera images

**Configuration**:
1. Add **Image** display
2. Set **Image Topic**: `/camera/image_raw`
3. Set **Transport Hint**: raw, compressed

**What it shows**:
- Camera feed
- Processed images
- Overlays and annotations

### 6. Marker Display

**Purpose**: Custom visualization shapes

**Configuration**:
1. Add **Marker** or **MarkerArray**
2. Set **Marker Topic**: `/visualization_marker`

**What it shows**:
- Custom shapes (arrows, spheres, cubes)
- Path trajectories
- Goal positions
- Debug information

## Camera Controls

### Mouse Controls

**Orbit** (default):
- **Left-click + drag**: Rotate view
- **Middle-click + drag**: Pan view
- **Scroll wheel**: Zoom in/out
- **Right-click**: Context menu

**FPS Mode**:
- **W/A/S/D**: Move camera
- **Mouse**: Look around
- **Q/E**: Up/down

### Camera Views

**Top-down**:
1. Tools → Views Panel
2. Type: TopDownOrtho
3. Good for 2D navigation

**Orbit** (default):
1. Type: Orbit
2. Good for 3D visualization

**Fixed Frame**:
- Set in Global Options
- All data transformed to this frame
- Usually: `map`, `odom`, or `base_link`

## Configuration Files

### Saving Configuration

```
File → Save Config As...
  → my_robot.rviz
```

**What's saved**:
- All display settings
- Camera position
- Fixed frame
- Panel layout

### Loading Configuration

```bash
# Command line
rviz2 -d my_robot.rviz

# Or: File → Open Config
```

### Config File Format

```yaml
Panels:
  - Class: rviz_common/Displays

Visualization Manager:
  Global Options:
    Fixed Frame: map
    Frame Rate: 30

  Displays:
    - Class: rviz_default_plugins/RobotModel
      Description Topic: /robot_description

    - Class: rviz_default_plugins/TF
      Show Names: true
      Show Axes: true

    - Class: rviz_default_plugins/LaserScan
      Topic: /scan
      Size: 0.05
      Color: 255; 255; 0  # Yellow
```

## Practical Example: Visualizing Robot

### Step 1: Prepare Robot Description

```bash
# Create simple URDF
cat > simple_robot.urdf <<EOF
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>
</robot>
EOF
```

### Step 2: Publish Robot State

```bash
# Publish robot description and TF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat simple_robot.urdf)"
```

### Step 3: Launch RViz

```bash
rviz2
```

### Step 4: Configure Displays

1. **Set Fixed Frame**: `base_link` (Global Options)
2. **Add RobotModel**:
   - Description Topic: `/robot_description`
3. **Add TF**:
   - Show Names: ✓
   - Show Axes: ✓

**Result**: Blue box appears representing robot!

## Common Display Settings

### Grid Display

**Purpose**: Reference grid for scale

```yaml
Add → Grid
  Cell Size: 1.0
  Cell Count: 10
  Color: 160; 160; 160
  Plane: XY (for ground)
```

### Axes Display

**Purpose**: Show coordinate system origin

```yaml
Add → Axes
  Length: 1.0
  Radius: 0.1
```

## Debugging with RViz

### Common Issues

**1. "No transform from [X] to [Y]"**
```
Problem: TF tree incomplete
Solution:
  - Check TF with: ros2 run tf2_tools view_frames
  - Verify all transforms being published
  - Check frame names match exactly
```

**2. Robot Model Gray/Invisible**
```
Problem: No robot_description topic
Solution:
  - Check: ros2 topic list | grep robot_description
  - Verify robot_state_publisher running
```

**3. LaserScan Not Showing**
```
Problem: Wrong frame or no data
Solution:
  - Check topic: ros2 topic echo /scan
  - Verify Fixed Frame can transform to laser frame
  - Check laser data publishing
```

**4. Display Shows "No messages received"**
```
Problem: Topic not publishing or wrong topic name
Solution:
  - List topics: ros2 topic list
  - Check data: ros2 topic echo <topic_name>
  - Verify topic name in display settings
```

## Best Practices

**1. Fixed Frame Selection**
```
Use:
  - "map" for global/world visualization
  - "odom" for local odometry
  - "base_link" for robot-centric view
```

**2. Performance**
```
If RViz is slow:
  - Reduce point cloud size
  - Decrease display update rate
  - Disable unused displays
  - Lower quality settings
```

**3. Organization**
```
Group related displays:
  - Create groups in Displays panel
  - Name displays clearly
  - Disable when not needed
```

**4. Config Management**
```
Save configs for:
  - Different robots
  - Different tasks (navigation, manipulation)
  - Different debug scenarios
```

## RViz in Launch Files

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'robot.rviz')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([rviz_node])
```

## Summary

**RViz is essential for**:
- Visualizing robot structure (URDF)
- Displaying sensor data (lasers, cameras, point clouds)
- Debugging transforms (TF tree)
- Monitoring robot state
- Visual feedback during development

**Key displays**:
- **RobotModel**: Shows robot from URDF
- **TF**: Visualizes coordinate frames
- **LaserScan**: 2D lidar data
- **PointCloud2**: 3D sensor data
- **Image**: Camera feeds
- **Marker**: Custom visualizations

**Workflow**:
1. Start RViz
2. Add displays for your data
3. Configure topics and settings
4. Save configuration
5. Use in launch files

## What's Next?

**Next Lesson**: URDF Fundamentals - Learn to describe robots in XML

**You'll learn**:
- URDF XML structure
- Links and joints
- Visual and collision geometry
- Creating robot descriptions

---

**Key Takeaway**: RViz is your visual debugging window into the robot's world. Master it to understand what your robot sees and does!
