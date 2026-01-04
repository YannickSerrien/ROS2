# Lesson 6: Broadcasting Transformations

## Learning Objectives

By the end of this lesson, you will be able to:
- Broadcast static transforms using C++ and command-line tools
- Publish dynamic transforms that change over time
- Use TransformBroadcaster for regular transform updates
- Convert between Euler angles and quaternions
- Understand parent-child frame relationships
- Build a complete TF tree for a robot

## Introduction

In Lesson 5, you learned **what** TF2 is. Now you'll learn **how** to publish transforms—the foundation of any TF-enabled robot system.

Broadcasting transforms means telling ROS2: "Frame B is at position X, Y, Z relative to Frame A, rotated by quaternion Q."

## Static vs Dynamic Transforms

### Static Transforms

**Definition**: Relationships that never change (e.g., camera mounted on robot base).

**When to Use**:
- Sensor positions on robot
- Fixed link-to-link relationships
- Calibrated frame offsets

### Dynamic Transforms

**Definition**: Relationships that change (e.g., robot moving, joint rotating).

**When to Use**:
- Robot position in world
- Rotating joints
- Moving parts (gripper opening/closing)

## Static Transform Publisher

### Command-Line Tool

The easiest way to publish static transforms:

```bash
ros2 run tf2_ros static_transform_publisher \
    x y z yaw pitch roll \
    parent_frame child_frame

# Example: Camera 0.5m forward, 1m up from base
ros2 run tf2_ros static_transform_publisher \
    0.5 0 1.0 0 0 0 \
    base_link camera_link
```

**Parameters**:
- `x y z`: Translation in meters
- `yaw pitch roll`: Rotation in radians (or use quaternion)
- `parent_frame`: Reference frame
- `child_frame`: Frame being defined

### In C++ Code

```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class StaticFramePublisher : public rclcpp::Node
{
public:
    StaticFramePublisher() : Node("static_tf_publisher")
    {
        // Create static broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transform
        publish_static_transform();
    }

private:
    void publish_static_transform()
    {
        geometry_msgs::msg::TransformStamped transform;

        // Header
        transform.header.stamp = this->now();
        transform.header.frame_id = "base_link";  // Parent
        transform.child_frame_id = "camera_link";  // Child

        // Translation
        transform.transform.translation.x = 0.5;  // 0.5m forward
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 1.0;  // 1m up

        // Rotation (identity = no rotation)
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        // Broadcast
        tf_static_broadcaster_->sendTransform(transform);

        RCLCPP_INFO(this->get_logger(), "Published static transform: base_link → camera_link");
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};
```

## Dynamic Transform Publisher

For transforms that change over time:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class DynamicFramePublisher : public rclcpp::Node
{
public:
    DynamicFramePublisher() : Node("dynamic_tf_publisher"), angle_(0.0)
    {
        // Create dynamic broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Publish at 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DynamicFramePublisher::publish_transform, this));
    }

private:
    void publish_transform()
    {
        geometry_msgs::msg::TransformStamped transform;

        // Header with current time
        transform.header.stamp = this->now();
        transform.header.frame_id = "world";
        transform.child_frame_id = "robot_base";

        // Position (moving in circle)
        transform.transform.translation.x = std::cos(angle_);
        transform.transform.translation.y = std::sin(angle_);
        transform.transform.translation.z = 0.0;

        // Rotation (using tf2 helper)
        tf2::Quaternion q;
        q.setRPY(0, 0, angle_);  // Roll, Pitch, Yaw
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        // Broadcast
        tf_broadcaster_->sendTransform(transform);

        // Update angle for next iteration
        angle_ += 0.1;
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    double angle_;
};
```

## Euler Angles ↔ Quaternions

### What are Quaternions?

Quaternions represent rotations with 4 numbers (x, y, z, w). They avoid gimbal lock and interpolate smoothly.

**Don't worry about the math**—use helper functions!

### Converting from Euler Angles (Roll, Pitch, Yaw)

```cpp
#include <tf2/LinearMath/Quaternion.h>

// Create quaternion from Euler angles (radians)
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);

// Use in transform
transform.transform.rotation.x = q.x();
transform.transform.rotation.y = q.y();
transform.transform.rotation.z = q.z();
transform.transform.rotation.w = q.w();
```

### Converting to Euler Angles

```cpp
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// Extract from transform
tf2::Quaternion q(
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w
);

// Convert to Euler
tf2::Matrix3x3 m(q);
double roll, pitch, yaw;
m.getRPY(roll, pitch, yaw);
```

## Complete Example: Robot with Sensors

```cpp
/**
 * @file robot_tf_publisher.cpp
 * @brief Publishes TF tree for robot with multiple sensors
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class RobotTFPublisher : public rclcpp::Node
{
public:
    RobotTFPublisher() : Node("robot_tf_publisher"), x_(0.0), y_(0.0), theta_(0.0)
    {
        // Broadcasters
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms once
        publish_static_transforms();

        // Publish dynamic transform at 20 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RobotTFPublisher::publish_dynamic_transforms, this));

        RCLCPP_INFO(this->get_logger(), "Robot TF publisher started");
    }

private:
    void publish_static_transforms()
    {
        // Camera on top of robot
        geometry_msgs::msg::TransformStamped camera_tf;
        camera_tf.header.stamp = this->now();
        camera_tf.header.frame_id = "base_link";
        camera_tf.child_frame_id = "camera_link";
        camera_tf.transform.translation.x = 0.1;
        camera_tf.transform.translation.y = 0.0;
        camera_tf.transform.translation.z = 0.5;

        tf2::Quaternion q_camera;
        q_camera.setRPY(0, 0, 0);  // No rotation
        camera_tf.transform.rotation.x = q_camera.x();
        camera_tf.transform.rotation.y = q_camera.y();
        camera_tf.transform.rotation.z = q_camera.z();
        camera_tf.transform.rotation.w = q_camera.w();

        // Lidar in front
        geometry_msgs::msg::TransformStamped lidar_tf;
        lidar_tf.header.stamp = this->now();
        lidar_tf.header.frame_id = "base_link";
        lidar_tf.child_frame_id = "laser_link";
        lidar_tf.transform.translation.x = 0.3;
        lidar_tf.transform.translation.y = 0.0;
        lidar_tf.transform.translation.z = 0.2;

        tf2::Quaternion q_lidar;
        q_lidar.setRPY(0, 0, 0);
        lidar_tf.transform.rotation.x = q_lidar.x();
        lidar_tf.transform.rotation.y = q_lidar.y();
        lidar_tf.transform.rotation.z = q_lidar.z();
        lidar_tf.transform.rotation.w = q_lidar.w();

        // Broadcast static transforms
        static_broadcaster_->sendTransform({camera_tf, lidar_tf});
    }

    void publish_dynamic_transforms()
    {
        // odom → base_link (robot moving)
        geometry_msgs::msg::TransformStamped odom_to_base;
        odom_to_base.header.stamp = this->now();
        odom_to_base.header.frame_id = "odom";
        odom_to_base.child_frame_id = "base_link";

        // Simulate robot motion
        x_ += 0.01 * std::cos(theta_);
        y_ += 0.01 * std::sin(theta_);
        theta_ += 0.01;

        odom_to_base.transform.translation.x = x_;
        odom_to_base.transform.translation.y = y_;
        odom_to_base.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_to_base.transform.rotation.x = q.x();
        odom_to_base.transform.rotation.y = q.y();
        odom_to_base.transform.rotation.z = q.z();
        odom_to_base.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(odom_to_base);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_, y_, theta_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotTFPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

## Best Practices

### 1. Publish at Appropriate Rates

```cpp
// Static: Publish once
static_broadcaster->sendTransform(transform);

// Slow-moving (odom): 10-20 Hz
timer_ = create_wall_timer(50ms, callback);

// Fast-moving (joints): 50-100 Hz
timer_ = create_wall_timer(10ms, callback);
```

### 2. Always Timestamp Correctly

```cpp
// GOOD: Current time
transform.header.stamp = this->now();

// BAD: Zero time (causes lookup failures)
transform.header.stamp = rclcpp::Time(0);

// GOOD for static: Can use now()
```

### 3. Follow Naming Conventions

```cpp
// Standard frame names (REP-105)
"map"           // World-fixed frame
"odom"          // Odometry frame
"base_link"     // Robot base center
"base_footprint"  // Projection on ground

// Sensor frames
"camera_link"
"laser_link"
"imu_link"
```

### 4. Maintain Proper Tree Structure

```
map
└─ odom
   └─ base_link
      ├─ camera_link
      ├─ laser_link
      └─ arm_base
          └─ arm_link1
```

Each frame has ONE parent (tree, not graph).

## Common Pitfalls

### 1. Reversed Parent-Child

```cpp
// BAD: Backwards relationship
transform.header.frame_id = "camera_link";
transform.child_frame_id = "base_link";
// This says base_link is attached to camera!

// GOOD: Camera attached to base
transform.header.frame_id = "base_link";
transform.child_frame_id = "camera_link";
```

### 2. Creating TF Cycles

```cpp
// BAD: Creates cycle
// A → B and B → A (impossible!)

// GOOD: Tree structure
// A → B → C
```

### 3. Not Publishing Regularly

```cpp
// BAD: Only publish once for dynamic transform
tf_broadcaster->sendTransform(transform);
// TF will timeout!

// GOOD: Publish continuously
timer_ = create_wall_timer(100ms, [this]() {
    tf_broadcaster->sendTransform(transform);
});
```

## Visualizing in RViz

```bash
# Terminal 1: Run your TF publisher
ros2 run my_package robot_tf_publisher

# Terminal 2: Launch RViz
rviz2

# In RViz:
# 1. Set Fixed Frame to "odom" or "base_link"
# 2. Add → TF
# 3. See your frame tree!
```

## Debugging Commands

```bash
# View TF tree (creates PDF)
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo odom base_link

# Monitor TF for problems
ros2 run tf2_ros tf2_monitor

# Check if transform exists
ros2 run tf2_ros tf2_echo parent_frame child_frame
```

## Summary

Broadcasting transforms publishes frame relationships:

**Key Concepts**:
- **Static**: Fixed relationships (use `StaticTransformBroadcaster`)
- **Dynamic**: Changing relationships (use `TransformBroadcaster`)
- **Rate**: Static once, dynamic 10-100 Hz
- **Quaternions**: Use `tf2::Quaternion` and `setRPY()` helper

**Structure**:
```cpp
transform.header.frame_id = "parent";
transform.child_frame_id = "child";
transform.transform.translation = {x, y, z};
transform.transform.rotation = quaternion;
broadcaster->sendTransform(transform);
```

## What's Next?

You can now publish transforms! Next, you'll learn to **use** them:

- **Lesson 7**: Listening to Transformations - Query and transform points/poses
- **Lesson 8**: TF2 Advanced - Time travel, complex queries, debugging

## Further Reading

- [TF2 Broadcaster Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html)
- [REP-105: Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
- [Quaternion Basics](https://www.ros.org/reps/rep-0103.html)

---

**Exercise**: Create a TF publisher for a robot arm with 2 joints. Publish:
- Static: base_link → joint1_link (fixed)
- Dynamic: joint1_link → joint2_link (rotating)
- Static: joint2_link → end_effector_link (fixed)

**Next Lesson**: [Lesson 7: Listening to Transformations](07-listening-transformations.md)
