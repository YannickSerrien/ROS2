# Exercise 2: Robot Transform Tree with Launch Configuration

## Objective

Build a complete robot transform tree system with static and dynamic transforms, configured through launch files with parameters.

## Learning Goals

- Publish static transforms for fixed robot components
- Broadcast dynamic transforms for moving parts
- Query and use transforms between frames
- Create launch files with parameters and arguments
- Configure multi-node systems

## Background

Real robots have complex coordinate frame relationships:
- **Static frames**: Sensor mounts, fixed links
- **Dynamic frames**: Moving base, rotating joints

Launch files make it easy to configure and start all TF publishers together.

## Task Description

Build a mobile robot with sensor suite:

### Robot Structure
```
map (world frame)
 └─ odom (odometry frame)
     └─ base_link (robot base - dynamic)
         ├─ camera_link (front camera - static)
         ├─ lidar_link (lidar scanner - static)
         └─ imu_link (IMU sensor - static)
```

### Required Nodes

1. **Static TF Publisher** (`static_tf_node`):
   - Publish base_link → camera_link
   - Publish base_link → lidar_link
   - Publish base_link → imu_link
   - Configure positions via parameters

2. **Dynamic TF Broadcaster** (`odom_broadcaster`):
   - Publish odom → base_link (robot movement)
   - Simulate robot moving in circle
   - Configurable speed and radius

3. **TF Listener** (`sensor_monitor`):
   - Query transform from map to camera_link
   - Transform a point from camera frame to map frame
   - Display results periodically

4. **Launch File** (`robot_system.launch.py`):
   - Start all three nodes
   - Load sensor positions from YAML config
   - Support launch arguments for robot configuration

## Requirements

### Static TF Publisher

**Parameters**:
- `camera_x`, `camera_y`, `camera_z`: Camera position relative to base
- `camera_roll`, `camera_pitch`, `camera_yaw`: Camera orientation
- `lidar_x`, `lidar_y`, `lidar_z`: Lidar position
- `imu_x`, `imu_y`, `imu_z`: IMU position

**Behavior**:
- Publish all static transforms on startup
- Use tf2_ros::StaticTransformBroadcaster
- Convert Euler angles to quaternions

### Dynamic TF Broadcaster

**Parameters**:
- `radius`: Circle radius (meters)
- `angular_velocity`: Rotation speed (rad/s)
- `publish_rate`: TF update rate (Hz)

**Behavior**:
- Simulate robot moving in circle
- Publish odom → base_link at specified rate
- Also publish map → odom (static for this exercise)

### TF Listener

**Behavior**:
- Query transform from map to camera_link every 2 seconds
- Create a point in camera frame (e.g., 5m ahead)
- Transform point to map frame
- Log both positions

### Launch File

**Launch Arguments**:
- `robot_radius`: Circle radius (default: 2.0)
- `robot_speed`: Angular velocity (default: 0.5)
- `use_config`: Load sensor positions from YAML (default: false)

**Features**:
- Start all three nodes
- Pass parameters to nodes
- Support config file loading
- Set appropriate output modes

### Config File (`config/robot_params.yaml`)

```yaml
static_tf_node:
  ros__parameters:
    camera_x: 0.3
    camera_y: 0.0
    camera_z: 0.4
    camera_pitch: -0.349  # -20 degrees

    lidar_x: 0.2
    lidar_y: 0.0
    lidar_z: 0.15

    imu_x: 0.0
    imu_y: 0.0
    imu_z: 0.05
```

## Starter Code

The starter package includes:
- Package structure with dependencies
- Node skeletons with TODO comments
- Launch file template
- Config file template

## Testing

### Test 1: Basic System Launch

```bash
ros2 launch robot_tf_launch robot_system.launch.py
```

**Expected**:
- All three nodes start
- TF tree published
- Sensor monitor displays transformed points

### Test 2: Visualize TF Tree

**Terminal 1**: Launch system
```bash
ros2 launch robot_tf_launch robot_system.launch.py
```

**Terminal 2**: View frames
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

**Expected**: Complete tree from map → odom → base_link → sensors

### Test 3: Custom Configuration

```bash
ros2 launch robot_tf_launch robot_system.launch.py robot_radius:=5.0 robot_speed:=1.0
```

**Expected**: Larger, faster circular motion

### Test 4: Config File

```bash
ros2 launch robot_tf_launch robot_system.launch.py use_config:=true
```

**Expected**: Sensor positions loaded from YAML

### Test 5: Monitor Transforms

```bash
# Terminal 1: Launch system
ros2 launch robot_tf_launch robot_system.launch.py

# Terminal 2: Echo specific transform
ros2 run tf2_ros tf2_echo map camera_link

# Terminal 3: Monitor all frames
ros2 run tf2_ros tf2_monitor
```

## Hints

<details>
<summary>Hint 1: Static Transform Publisher</summary>

```cpp
tf2_ros::StaticTransformBroadcaster broadcaster(this);

geometry_msgs::msg::TransformStamped transform;
transform.header.stamp = this->now();
transform.header.frame_id = "base_link";
transform.child_frame_id = "camera_link";

// Set translation and rotation
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);
// ... assign to transform

broadcaster.sendTransform(transform);
```
</details>

<details>
<summary>Hint 2: Circular Motion</summary>

```cpp
double time = 0.0;
time += dt;

x = radius * cos(angular_velocity * time);
y = radius * sin(angular_velocity * time);
theta = angular_velocity * time + M_PI/2;
```
</details>

<details>
<summary>Hint 3: Transform Lookup</summary>

```cpp
tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

auto transform = tf_buffer_->lookupTransform(
    "map", "camera_link", tf2::TimePointZero);
```
</details>

<details>
<summary>Hint 4: Launch File Arguments</summary>

```python
robot_radius_arg = DeclareLaunchArgument(
    'robot_radius',
    default_value='2.0',
    description='Circle radius in meters'
)

Node(
    parameters=[{'radius': LaunchConfiguration('robot_radius')}]
)
```
</details>

## Bonus Challenges

1. **Joint State**: Add rotating sensor (e.g., spinning lidar)
2. **Multiple Robots**: Namespaces for two robots
3. **RViz Integration**: Add RViz launch with TF visualization
4. **GPS Frame**: Add earth/GPS coordinate frame

## Success Criteria

- [ ] Static transforms published correctly
- [ ] Dynamic transform updates at specified rate
- [ ] TF listener queries work
- [ ] Point transformation is correct
- [ ] Launch file starts all nodes
- [ ] Launch arguments work
- [ ] Config file loading works
- [ ] TF tree visualizes correctly

## Common Issues

**Transform Not Found**:
- Check frame names match exactly
- Ensure all TF publishers are running
- Use `ros2 topic echo /tf_static` to verify

**Lookup Fails**:
- Wait for transforms to be available
- Use timeout in lookupTransform
- Check buffer has been initialized

**Launch File Error**:
- Check Python syntax (indentation)
- Verify package/executable names
- Use `ros2 pkg executables robot_tf_launch` to check

## Related Lessons

- Lesson 5: TF2 Basics
- Lesson 6: Broadcasting Transformations
- Lesson 7: Listening to Transformations
- Lesson 9: Launch Files Basics
- Lesson 10: Launch Files Advanced

## Next Steps

- Complete Exercise 3 (Composition)
- Integrate TF2 into mini-project
- Combine with actions for coordinate-aware tasks
