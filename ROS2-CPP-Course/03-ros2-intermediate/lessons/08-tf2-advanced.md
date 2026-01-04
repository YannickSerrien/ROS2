# Lesson 8: TF2 Advanced Topics

## Learning Objectives

- Query transforms at specific past times ("time travel")
- Understand and use transform timeouts
- Debug TF problems effectively
- Optimize TF performance for real-time systems
- Handle complex frame tree scenarios

## Time Travel in TF2

TF2 maintains transform history (default: 10 seconds). You can query past transforms:

```cpp
// Get transform from 2 seconds ago
rclcpp::Time two_seconds_ago = this->now() - rclcpp::Duration::from_seconds(2.0);

try {
    auto old_transform = tf_buffer_->lookupTransform(
        "base_link",
        "camera_link",
        two_seconds_ago,
        tf2::durationFromSec(0.5));  // Timeout
} catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(logger, "Requested time not in buffer: %s", ex.what());
}
```

**Use Case**: Processing sensor data that arrived late.

## Waiting for Transforms

```cpp
// Wait for transform to become available
bool wait_for_transform(const std::string & target, const std::string & source)
{
    try {
        // Block until transform exists (or timeout)
        auto transform = tf_buffer_->lookupTransform(
            target, source,
            tf2::TimePointZero,
            tf2::durationFromSec(5.0));  // Wait up to 5 seconds
        return true;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(logger, "Transform not available: %s", ex.what());
        return false;
    }
}
```

## Frame Existence Check

```cpp
bool frame_exists(const std::string & frame_name)
{
    return tf_buffer_->_frameExists(frame_name);
}

// Get all frames
std::string all_frames = tf_buffer_->allFramesAsString();
RCLCPP_INFO(logger, "Available frames:\n%s", all_frames.c_str());
```

## Advanced Transformations

### Transform with Velocity

```cpp
#include <geometry_msgs/msg/twist_stamped.hpp>

// Transform velocity (linear + angular)
geometry_msgs::msg::TwistStamped vel_in_camera;
vel_in_camera.header.frame_id = "camera_link";
vel_in_camera.header.stamp = this->now();
vel_in_camera.twist.linear.x = 1.0;

auto vel_in_base = tf_buffer_->transform(vel_in_camera, "base_link");
```

### Chain of Transforms

TF2 automatically handles chains:

```
camera_link → base_link → odom → map
```

```cpp
// Automatically traverses chain
auto transform = tf_buffer_->lookupTransform("map", "camera_link", time);
// TF2 internally computes: camera→base, base→odom, odom→map
```

## Debugging TF Issues

### Common Problems and Solutions

**Problem: "Frame X does not exist"**
```bash
# Check if frame is being published
ros2 topic echo /tf --no-arr | grep frame_id
ros2 topic echo /tf_static --no-arr | grep frame_id
```

**Problem: "Extrapolation into the future"**
- Cause: Requested time > latest transform time
- Solution: Use `tf2::TimePointZero` for latest

**Problem: "Lookup would require extrapolation into the past"**
- Cause: Requested time < oldest buffered transform
- Solution: Increase buffer size or request more recent time

### Diagnostic Tools

```bash
# View entire TF tree
ros2 run tf2_tools view_frames
# Creates frames.pdf

# Monitor specific transform
ros2 run tf2_ros tf2_echo parent_frame child_frame

# Check TF publication rates
ros2 run tf2_ros tf2_monitor

# List all frames
ros2 run tf2_ros tf2_echo --help
```

### TF Monitor Output

```bash
$ ros2 run tf2_ros tf2_monitor

Frame: base_link published by: robot_state_publisher
Average rate: 50.0 Hz  # Good!
Most recent transform: 0.020 seconds old  # Fresh

Frame: camera_link published by: static_transform_publisher
Average rate: N/A (static)
Most recent transform: 0.000 seconds old
```

## Performance Optimization

### Buffer Size

```cpp
// Default: 10 second history
tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

// Custom: 30 second history (uses more memory)
auto cache_time = tf2::durationFromSec(30.0);
tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), cache_time);
```

### Publication Rate Guidelines

```cpp
// Static transforms: Once is enough
static_broadcaster_->sendTransform(transform);

// Slow-changing (odom): 10-20 Hz
timer_ = create_wall_timer(50ms, publish_callback);

// Fast joints/sensors: 50-100 Hz
timer_ = create_wall_timer(10ms, publish_callback);

// Very fast (real-time control): 200+ Hz
timer_ = create_wall_timer(5ms, publish_callback);
```

## Complex Scenarios

### Multiple Robots

```
map
├─ robot1/odom
│  └─ robot1/base_link
│     └─ robot1/camera_link
└─ robot2/odom
   └─ robot2/base_link
      └─ robot2/camera_link
```

Use namespaces to avoid conflicts.

### Dynamic Reconfiguration

```cpp
// Handling frames that appear/disappear
if (!tf_buffer_->canTransform("base_link", "sensor_link", time)) {
    RCLCPP_WARN(logger, "Sensor not available yet");
    return;
}

auto transform = tf_buffer_->lookupTransform("base_link", "sensor_link", time);
```

## Summary

**Advanced Techniques**:
- Time travel: Query past transforms
- Waiting: Block until transform available
- Debugging: Use tf2_tools for visualization
- Optimization: Tune buffer size and rates

**Best Practices**:
- Static: Publish once
- Dynamic: 10-100 Hz depending on speed
- Always handle exceptions
- Monitor with tf2_monitor

## What's Next?

You've mastered TF2! Next topic: **Launch Files** for system orchestration.

- **Lesson 9**: Launch Files Basics
- **Lesson 10**: Launch Advanced Features

---

**Next Lesson**: [Lesson 9: Launch Files Basics](09-launch-files-basics.md)
