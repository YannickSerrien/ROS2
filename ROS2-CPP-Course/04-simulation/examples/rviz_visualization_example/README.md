# RViz Visualization Example

Demonstrates RViz visualization capabilities including markers, paths, and interactive markers.

## What This Demonstrates

- **Marker types**: Arrow, cube, sphere, cylinder, line strip, text
- **Path visualization**: nav_msgs/Path and animated trajectories
- **Interactive markers**: Draggable goal setting

## Nodes

### marker_publisher
Publishes various marker types to `/visualization_marker_array`.

**Run**:
```bash
ros2 run rviz_visualization_example marker_publisher
```

### path_visualizer
Publishes circular path and animated spiral trajectory.

**Topics**:
- `/planned_path` (nav_msgs/Path)
- `/trajectory_marker` (visualization_msgs/Marker)

**Run**:
```bash
ros2 run rviz_visualization_example path_visualizer
```

### interactive_marker_server
Creates draggable goal marker, publishes goals to `/goal_pose`.

**Run**:
```bash
ros2 run rviz_visualization_example interactive_marker_server
```

## Quick Start

### Launch Everything

```bash
# Build
colcon build --packages-select rviz_visualization_example
source install/setup.bash

# Launch all nodes + RViz
ros2 launch rviz_visualization_example visualization.launch.py
```

### Manual Setup

```bash
# Terminal 1: Marker publisher
ros2 run rviz_visualization_example marker_publisher

# Terminal 2: Path visualizer
ros2 run rviz_visualization_example path_visualizer

# Terminal 3: Interactive markers
ros2 run rviz_visualization_example interactive_marker_server

# Terminal 4: RViz
rviz2 -d ./config/visualization.rviz
```

**In RViz**:
- Set Fixed Frame: `world`
- All displays should auto-load from config
- Drag the green arrow to set goals

## Related Lessons

- Module 4, Lesson 4: Visualization Markers

## Key Concepts

**Markers**: Custom 3D shapes for visualization
**MarkerArray**: Publish multiple markers efficiently
**Interactive Markers**: User-draggable markers for control
**Paths**: Trajectory visualization with nav_msgs/Path
