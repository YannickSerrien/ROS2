# TF2 Example

Complete demonstration of ROS2 TF2 (Transform) system with broadcasters, listeners, and complete robot transform trees.

## What's Included

This package demonstrates TF2 coordinate transformation framework through four complete examples:

### 1. Static Transform Publisher
- **Node**: `static_tf_publisher`
- Publishes fixed transforms for robot components
- Demonstrates static broadcaster usage
- Shows quaternion construction from Euler angles

### 2. Dynamic Transform Broadcaster
- **Node**: `dynamic_tf_broadcaster`
- Publishes moving transforms (simulated robot odometry)
- Demonstrates proper time stamping
- Shows transform update rates (50 Hz)

### 3. Transform Listener
- **Node**: `tf_listener`
- Queries transforms between frames
- Transforms points between coordinate frames
- Demonstrates exception handling
- Shows transform availability checking

### 4. Complete Robot TF Tree
- **Node**: `robot_tf_tree`
- Full robot transform hierarchy
- Mix of static and dynamic transforms
- Realistic robot configuration

## Building

```bash
# From workspace root
colcon build --packages-select tf2_example
source install/setup.bash
```

## Running the Examples

### Example 1: Static Transforms Only

**Terminal 1** - Publish static transforms:
```bash
ros2 run tf2_example static_tf_publisher
```

**Terminal 2** - View transform tree:
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

**Terminal 3** - Echo a specific transform:
```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

**Expected Output**:
- Static transforms for `base_link -> camera_link` and `base_link -> laser_link`
- Transform values remain constant

### Example 2: Dynamic Transforms

**Terminal 1** - Start dynamic broadcaster:
```bash
ros2 run tf2_example dynamic_tf_broadcaster
```

**Terminal 2** - Monitor transform:
```bash
ros2 run tf2_ros tf2_echo odom base_link
```

**Expected Output**:
- Continuously updating transform as robot "moves"
- Position changes over time

### Example 3: Transform Listener

**Terminal 1** - Start complete robot TF tree:
```bash
ros2 run tf2_example robot_tf_tree
```

**Terminal 2** - Start listener:
```bash
ros2 run tf2_example tf_listener
```

**Expected Output**:
- Listener queries transforms every second
- Shows transform from odom to camera_link
- Demonstrates point transformation
- Lists all available frames

### Example 4: Complete Robot (Recommended)

**Terminal 1** - Start complete robot TF tree:
```bash
ros2 run tf2_example robot_tf_tree
```

This single node publishes:
- `map -> odom` (static)
- `odom -> base_link` (dynamic, 50 Hz)
- `base_link -> camera_link` (static)
- `base_link -> laser_link` (static)
- `base_link -> imu_link` (static)

**Terminal 2** - Visualize in RViz:
```bash
rviz2
```

In RViz:
1. Set Fixed Frame to `map`
2. Add -> TF display
3. See complete transform tree animating

**Terminal 3** - Query any transform:
```bash
# Camera position in map frame
ros2 run tf2_ros tf2_echo map camera_link

# Laser position in odom frame
ros2 run tf2_ros tf2_echo odom laser_link
```

## TF2 Command Line Tools

### List all frames:
```bash
ros2 run tf2_ros tf2_monitor
```

**Output**:
```
Frame: base_link published by: /robot_tf_tree
Average rate: 50.0 Hz
```

### View transform tree:
```bash
ros2 run tf2_tools view_frames
```

Generates `frames.pdf` showing complete TF tree diagram.

### Echo specific transform:
```bash
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>
```

Example:
```bash
ros2 run tf2_ros tf2_echo map camera_link
```

### Check TF topic:
```bash
# Dynamic transforms
ros2 topic echo /tf

# Static transforms
ros2 topic echo /tf_static
```

## Transform Tree Hierarchy

```
map (world/global frame)
 └─ odom (odometry frame - drift accumulates)
     └─ base_link (robot base)
         ├─ camera_link (camera sensor)
         ├─ laser_link (laser scanner)
         └─ imu_link (IMU sensor)
```

**Frame Conventions**:
- `map`: Global, static map frame
- `odom`: Odometry frame (smooth, local, drifts over time)
- `base_link`: Robot's main coordinate frame
- Sensor frames: Child frames of `base_link`

## Key Concepts Demonstrated

### Static vs Dynamic Transforms

**Static** (published once, on /tf_static):
```cpp
tf_static_broadcaster_->sendTransform(transform);
```
- Fixed robot components (sensors mounted on chassis)
- Never changes
- Examples: base_link → camera_link

**Dynamic** (published repeatedly, on /tf):
```cpp
// Publish at 50 Hz
timer_ = create_wall_timer(20ms, [this]() {
    transform.header.stamp = this->now();  // CRITICAL: update timestamp
    tf_broadcaster_->sendTransform(transform);
});
```
- Moving parts (robot position, joints)
- Updated regularly
- Examples: odom → base_link

### Time Stamping

**Always use current time for dynamic transforms**:
```cpp
transform.header.stamp = this->now();  // NOT rclcpp::Time(0)!
```

**Query latest transform**:
```cpp
tf_buffer_->lookupTransform("target", "source", tf2::TimePointZero);
```

### Quaternion Conversion

**Euler angles (roll, pitch, yaw) → Quaternion**:
```cpp
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);  // In radians
transform.transform.rotation.x = q.x();
transform.transform.rotation.y = q.y();
transform.transform.rotation.z = q.z();
transform.transform.rotation.w = q.w();
```

### Transform Lookup

**Basic lookup**:
```cpp
try {
    auto transform = tf_buffer_->lookupTransform(
        "target_frame",
        "source_frame",
        tf2::TimePointZero  // Latest
    );
} catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger, "TF error: %s", ex.what());
}
```

**With timeout**:
```cpp
auto transform = tf_buffer_->lookupTransform(
    "target", "source",
    tf2::TimePointZero,
    tf2::durationFromSec(1.0)  // Wait up to 1 second
);
```

### Point Transformation

**Transform point between frames**:
```cpp
geometry_msgs::msg::PointStamped point_in_camera;
point_in_camera.header.frame_id = "camera_link";
point_in_camera.point.x = 5.0;

auto point_in_map = tf_buffer_->transform(point_in_camera, "map");
```

## Update Rates

**Recommended rates**:
- Static transforms: Once (on startup)
- Slow odometry: 10-20 Hz
- Standard odometry: 30-50 Hz
- Fast motion/joints: 50-100 Hz
- Very fast (control loop): 100-200 Hz

**This example uses 50 Hz** for odom → base_link.

## Common Patterns

### Creating TF Buffer and Listener

```cpp
// In constructor
tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
```

### Check Transform Availability

```cpp
bool can_transform = tf_buffer_->canTransform(
    "target_frame",
    "source_frame",
    tf2::TimePointZero,
    tf2::durationFromSec(0.5)  // Timeout
);
```

### List All Frames

```cpp
std::string all_frames = tf_buffer_->allFramesAsString();
RCLCPP_INFO(logger, "Frames:\n%s", all_frames.c_str());
```

## Troubleshooting

**Error: "Frame does not exist"**
```bash
# Check if frame is being published
ros2 topic echo /tf --no-arr | grep frame_id
ros2 topic echo /tf_static --no-arr | grep frame_id
```

**Error: "Extrapolation into the future"**
- Requested time > latest transform time
- Solution: Use `tf2::TimePointZero` for latest

**Error: "Extrapolation into the past"**
- Requested time < oldest buffered time
- Solution: Increase buffer size or use more recent time

**Transform not updating**
```bash
# Check publication rate
ros2 run tf2_ros tf2_monitor
```

## Visualization in RViz

1. Start RViz: `rviz2`
2. Set **Fixed Frame** to `map`
3. Add → **TF** display
4. Configure TF display:
   - Show Names: On
   - Show Axes: On
   - Show Arrows: On
   - Frame Timeout: 15 seconds

You'll see the complete transform tree with moving robot.

## Related Lessons

- **Lesson 5**: TF2 Basics
- **Lesson 6**: Broadcasting Transformations
- **Lesson 7**: Listening to Transformations
- **Lesson 8**: TF2 Advanced Topics

## Next Steps

- Modify robot sensor positions
- Add more sensor frames (e.g., GPS, ultrasonic)
- Change robot motion pattern
- Combine with actions for goal-based movement
- Use in mini-project robot arm for forward kinematics
