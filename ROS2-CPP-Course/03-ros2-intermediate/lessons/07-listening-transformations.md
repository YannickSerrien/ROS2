# Lesson 7: Listening to Transformations

## Learning Objectives

By the end of this lesson, you will be able to:
- Query transforms between any two frames
- Transform points and poses between frames
- Handle transform exceptions and errors
- Use TF buffer and listeners correctly
- Implement time-synchronized transforms
- Build applications that use TF for sensor data processing

## Introduction

Broadcasting transforms tells ROS2 where frames are. **Listening** to transforms lets you use that information:
- "Where is this object (in camera frame) relative to the robot base?"
- "Transform this laser scan from sensor frame to map frame"
- "What's the position of the gripper in world coordinates?"

## TF2 Buffer and Listener

### Setup

```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TFListenerNode : public rclcpp::Node
{
public:
    TFListenerNode() : Node("tf_listener")
    {
        // Create TF buffer (stores transform history)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        // Create TF listener (subscribes to /tf and /tf_static)
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "TF Listener ready");
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
```

**Important**: Create the buffer BEFORE the listener!

## Looking Up Transforms

### Basic Transform Query

```cpp
void lookup_transform()
{
    std::string target_frame = "base_link";
    std::string source_frame = "camera_link";

    try {
        // Get latest transform
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_->lookupTransform(
                target_frame,   // To this frame
                source_frame,   // From this frame
                tf2::TimePointZero);  // Latest available

        RCLCPP_INFO(this->get_logger(),
                   "Transform from %s to %s:",
                   source_frame.c_str(), target_frame.c_str());
        RCLCPP_INFO(this->get_logger(),
                   "  Translation: (%.2f, %.2f, %.2f)",
                   transform_stamped.transform.translation.x,
                   transform_stamped.transform.translation.y,
                   transform_stamped.transform.translation.z);

    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(),
                    "Could not transform: %s", ex.what());
    }
}
```

### Transform at Specific Time

```cpp
void lookup_transform_at_time(rclcpp::Time when)
{
    try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(
                "base_link",
                "camera_link",
                when,  // Specific time
                tf2::durationFromSec(1.0));  // Timeout

    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    }
}
```

## Transforming Points

### Point Transformation

```cpp
void transform_point()
{
    // Create point in source frame
    geometry_msgs::msg::PointStamped point_in_camera;
    point_in_camera.header.frame_id = "camera_link";
    point_in_camera.header.stamp = this->now();
    point_in_camera.point.x = 1.0;
    point_in_camera.point.y = 0.5;
    point_in_camera.point.z = 0.2;

    try {
        // Transform to target frame
        geometry_msgs::msg::PointStamped point_in_base =
            tf_buffer_->transform(point_in_camera, "base_link");

        RCLCPP_INFO(this->get_logger(),
                   "Point in camera: (%.2f, %.2f, %.2f)",
                   point_in_camera.point.x,
                   point_in_camera.point.y,
                   point_in_camera.point.z);
        RCLCPP_INFO(this->get_logger(),
                   "Point in base: (%.2f, %.2f, %.2f)",
                   point_in_base.point.x,
                   point_in_base.point.y,
                   point_in_base.point.z);

    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    }
}
```

### Pose Transformation

```cpp
void transform_pose()
{
    // Create pose in source frame
    geometry_msgs::msg::PoseStamped pose_in_map;
    pose_in_map.header.frame_id = "map";
    pose_in_map.header.stamp = this->now();
    pose_in_map.pose.position.x = 5.0;
    pose_in_map.pose.position.y = 3.0;
    pose_in_map.pose.position.z = 0.0;
    pose_in_map.pose.orientation.w = 1.0;  // Identity rotation

    try {
        // Transform to robot frame
        geometry_msgs::msg::PoseStamped pose_in_robot =
            tf_buffer_->transform(pose_in_map, "base_link");

        RCLCPP_INFO(this->get_logger(),
                   "Object in robot frame: (%.2f, %.2f)",
                   pose_in_robot.pose.position.x,
                   pose_in_robot.pose.position.y);

    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    }
}
```

## Complete Example: Object Detector

```cpp
/**
 * @file object_localizer.cpp
 * @brief Transforms detected objects from camera to robot frame
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>

class ObjectLocalizer : public rclcpp::Node
{
public:
    ObjectLocalizer() : Node("object_localizer")
    {
        // TF setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to detections (simulated as points for this example)
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/object_detection",
            10,
            std::bind(&ObjectLocalizer::detection_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Object localizer started");
    }

private:
    void detection_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                   "Detected object at (%.2f, %.2f, %.2f) in %s frame",
                   msg->point.x, msg->point.y, msg->point.z,
                   msg->header.frame_id.c_str());

        try {
            // Transform to base_link
            auto point_in_base = tf_buffer_->transform(*msg, "base_link");

            RCLCPP_INFO(this->get_logger(),
                       "Object in robot frame: (%.2f, %.2f, %.2f)",
                       point_in_base.point.x,
                       point_in_base.point.y,
                       point_in_base.point.z);

            // Could also transform to map frame
            auto point_in_map = tf_buffer_->transform(*msg, "map");

            RCLCPP_INFO(this->get_logger(),
                       "Object in map frame: (%.2f, %.2f, %.2f)",
                       point_in_map.point.x,
                       point_in_map.point.y,
                       point_in_map.point.z);

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                       "Could not transform detection: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectLocalizer>());
    rclcpp::shutdown();
    return 0;
}
```

## Handling Time

### Why Timestamps Matter

For moving frames, the transform depends on **when** you ask:

```cpp
// Robot at t=0: base_link at (0, 0)
// Robot at t=5: base_link at (2, 1)

// Transform point detected at t=0
point.header.stamp = rclcpp::Time(0);  // When point was observed
auto transformed = tf_buffer->transform(point, "map");
// Uses transform from t=0
```

### Using Sensor Timestamps

```cpp
void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Use the timestamp from when image was captured
    geometry_msgs::msg::PointStamped detected_point;
    detected_point.header.stamp = msg->header.stamp;  // Important!
    detected_point.header.frame_id = "camera_link";
    // ... set point coordinates from image processing

    try {
        auto point_in_base = tf_buffer_->transform(detected_point, "base_link");
    } catch (const tf2::TransformException & ex) {
        // Handle error
    }
}
```

## Common Exceptions

### Transform Not Available

```cpp
try {
    auto transform = tf_buffer_->lookupTransform("target", "source", tf2::TimePointZero);
} catch (const tf2::LookupException & ex) {
    // Frames don't exist or aren't connected
    RCLCPP_ERROR(logger, "Lookup failed: %s", ex.what());
} catch (const tf2::ConnectivityException & ex) {
    // Frames exist but no path between them
    RCLCPP_ERROR(logger, "Not connected: %s", ex.what());
} catch (const tf2::ExtrapolationException & ex) {
    // Requested time not in buffer
    RCLCPP_ERROR(logger, "Time out of range: %s", ex.what());
}
```

## Common Pitfalls

### 1. Not Using Sensor Timestamps

```cpp
// BAD: Uses current time, not when data was captured
point.header.stamp = this->now();

// GOOD: Uses sensor's timestamp
point.header.stamp = sensor_msg->header.stamp;
```

### 2. Swapped Frame Order

```cpp
// Returns transform FROM source TO target
lookupTransform(target, source, time);

// Common mistake: backwards
lookupTransform(source, target, time);  // Wrong direction!
```

### 3. Not Handling Exceptions

```cpp
// BAD: Can crash
auto transform = tf_buffer->lookupTransform("a", "b", time);

// GOOD: Always use try-catch
try {
    auto transform = tf_buffer->lookupTransform("a", "b", time);
} catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger, "Error: %s", ex.what());
}
```

## Best Practices

1. **Always Use try-catch**
   ```cpp
   try {
       auto result = tf_buffer_->transform(input, target_frame);
   } catch (const tf2::TransformException & ex) {
       // Handle error
   }
   ```

2. **Preserve Timestamps**
   ```cpp
   // Keep original timestamp from sensor
   output.header.stamp = input.header.stamp;
   ```

3. **Check Frame IDs**
   ```cpp
   if (msg->header.frame_id.empty()) {
       RCLCPP_WARN(logger, "Message has no frame_id!");
       return;
   }
   ```

4. **Use Timeouts**
   ```cpp
   // Wait up to 1 second for transform
   tf_buffer_->lookupTransform(
       target, source, time,
       tf2::durationFromSec(1.0));
   ```

## Summary

Listening to transforms enables coordinate conversions:

**Key Operations**:
- **lookupTransform()**: Get transform between frames
- **transform()**: Transform point/pose to different frame

**Critical Points**:
- Use sensor timestamps, not current time
- Always use try-catch for TF operations
- Buffer stores transform history (~10 seconds)
- Create buffer before listener

**Common Pattern**:
```cpp
// Setup (constructor)
tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

// Use (callback)
try {
    auto transformed = tf_buffer_->transform(input, "target_frame");
} catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger, "%s", ex.what());
}
```

## What's Next?

- **Lesson 8**: TF2 Advanced - Time travel, complex queries, performance tuning
- **Lesson 9**: Launch Files - Orchestrate TF publishers and listeners

## Further Reading

- [TF2 Listener Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)
- [TF2 API Documentation](https://docs.ros2.org/latest/api/tf2_ros/)

---

**Exercise**: Create a node that:
1. Subscribes to `/camera/objects` (PointStamped messages)
2. Transforms each point to `map` frame
3. Publishes transformed points to `/map/objects`

**Next Lesson**: [Lesson 8: TF2 Advanced](08-tf2-advanced.md)
