# Lesson 9: Robot Spawning and Control

## Learning Objectives

- Spawn robots programmatically in Gazebo
- Use launch files for robot deployment
- Configure differential drive controllers
- Implement Ackermann steering
- Control robots via ROS2 topics
- Manage multiple robot instances
- Apply forces and set velocities programmatically
- Debug spawning issues

## Spawning Robots in Gazebo

### Manual Spawning (Command Line)

**Spawn from URDF file**:
```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity my_robot \
  -file /path/to/robot.urdf \
  -x 0 -y 0 -z 0.5
```

**Spawn from topic** (robot_description):
```bash
# Terminal 1: Publish robot description
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat robot.urdf)"

# Terminal 2: Spawn from topic
ros2 run gazebo_ros spawn_entity.py \
  -entity my_robot \
  -topic /robot_description \
  -x 0 -y 0 -z 0.5
```

**Parameters**:
- `-entity`: Name in Gazebo (must be unique)
- `-file` or `-topic`: Source of URDF/SDF
- `-x`, `-y`, `-z`: Initial position
- `-R`, `-P`, `-Y`: Roll, pitch, yaw (radians)
- `-robot_namespace`: ROS namespace

### Spawning from Launch Files

**Complete robot launch**:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Package paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_my_robot = get_package_share_directory('my_robot_description')

    # URDF/XACRO file
    xacro_file = os.path.join(pkg_my_robot, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # World file
    world_file = os.path.join(pkg_my_robot, 'worlds', 'test_world.world')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

**Launch**:
```bash
ros2 launch my_robot_description spawn_robot.launch.py
```

## Differential Drive Controller

### What is Differential Drive?

**Differential drive**: Two independently driven wheels (left and right)
- Forward/backward: Both wheels same direction, same speed
- Turn: Wheels different speeds
- Rotate in place: Wheels opposite directions

**Common in**: Mobile robots, vacuum cleaners, wheelchairs

### Gazebo Diff Drive Plugin

**In URDF** (inside `<gazebo>` tag):

```xml
<gazebo>
  <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">

    <!-- ROS configuration -->
    <ros>
      <namespace>/robot</namespace>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>

    <!-- Wheel joints -->
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>

    <!-- Kinematics -->
    <wheel_separation>0.45</wheel_separation>  <!-- Distance between wheels (m) -->
    <wheel_diameter>0.2</wheel_diameter>       <!-- Wheel diameter (m) -->

    <!-- Limits -->
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>1.0</max_wheel_acceleration>

    <!-- Output -->
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>true</publish_wheel_tf>

    <!-- Frames -->
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>

    <!-- Odometry source -->
    <odometry_source>world</odometry_source>  <!-- 'world' (perfect) or 'encoder' (realistic) -->

    <!-- Update rate -->
    <update_rate>50</update_rate>

  </plugin>
</gazebo>
```

### Controlling Diff Drive Robot

**Command topic**: `/robot/cmd_vel` (geometry_msgs/Twist)

**Twist message structure**:
```cpp
geometry_msgs::msg::Twist cmd_vel;

// Linear velocity (m/s)
cmd_vel.linear.x = 0.5;   // Forward/backward
cmd_vel.linear.y = 0.0;   // Left/right (not used for diff drive)
cmd_vel.linear.z = 0.0;   // Up/down (not used)

// Angular velocity (rad/s)
cmd_vel.angular.x = 0.0;  // Roll (not used)
cmd_vel.angular.y = 0.0;  // Pitch (not used)
cmd_vel.angular.z = 0.5;  // Yaw (turning)
```

**Example C++ node**:

```cpp
/**
 * @file robot_teleop.cpp
 * @brief Simple keyboard teleoperation for differential drive robot
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>

using namespace std::chrono_literals;

class RobotTeleop : public rclcpp::Node
{
public:
    RobotTeleop() : Node("robot_teleop")
    {
        // Publisher for cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/robot/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Robot Teleop Started");
        RCLCPP_INFO(this->get_logger(), "Commands:");
        RCLCPP_INFO(this->get_logger(), "  w: Forward");
        RCLCPP_INFO(this->get_logger(), "  s: Backward");
        RCLCPP_INFO(this->get_logger(), "  a: Turn left");
        RCLCPP_INFO(this->get_logger(), "  d: Turn right");
        RCLCPP_INFO(this->get_logger(), "  x: Stop");
    }

    void send_command(double linear, double angular)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear;
        cmd_vel.angular.z = angular;

        cmd_vel_pub_->publish(cmd_vel);

        RCLCPP_INFO(this->get_logger(),
                   "Sent: linear=%.2f, angular=%.2f", linear, angular);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotTeleop>();

    // Example movements
    std::cout << "Moving forward for 2 seconds..." << std::endl;
    node->send_command(0.5, 0.0);  // Forward
    rclcpp::sleep_for(std::chrono::seconds(2));

    std::cout << "Turning left..." << std::endl;
    node->send_command(0.2, 0.5);  // Forward + turn left
    rclcpp::sleep_for(std::chrono::seconds(2));

    std::cout << "Stopping..." << std::endl;
    node->send_command(0.0, 0.0);  // Stop

    rclcpp::shutdown();
    return 0;
}
```

**Movement patterns**:

```cpp
// Forward
cmd_vel.linear.x = 0.5;
cmd_vel.angular.z = 0.0;

// Backward
cmd_vel.linear.x = -0.5;
cmd_vel.angular.z = 0.0;

// Rotate in place (left)
cmd_vel.linear.x = 0.0;
cmd_vel.angular.z = 0.5;

// Arc turn (forward + turn left)
cmd_vel.linear.x = 0.5;
cmd_vel.angular.z = 0.3;

// Stop
cmd_vel.linear.x = 0.0;
cmd_vel.angular.z = 0.0;
```

### Odometry Output

**Topic**: `/robot/odom` (nav_msgs/Odometry)

**Subscribing to odometry**:

```cpp
#include "nav_msgs/msg/odometry.hpp"

class OdometryMonitor : public rclcpp::Node
{
public:
    OdometryMonitor() : Node("odometry_monitor")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot/odom", 10,
            std::bind(&OdometryMonitor::odom_callback, this,
                     std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        double vx = msg->twist.twist.linear.x;
        double vz = msg->twist.twist.angular.z;

        RCLCPP_INFO(this->get_logger(),
                   "Position: (%.2f, %.2f) | Velocity: linear=%.2f, angular=%.2f",
                   x, y, vx, vz);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
```

## Ackermann Steering

### What is Ackermann Steering?

**Ackermann**: Car-like steering with front wheels turning
- Front wheels steer
- Rear wheels drive (or all-wheel drive)

**Common in**: Cars, trucks, autonomous vehicles

### Gazebo Ackermann Plugin

```xml
<gazebo>
  <plugin name="ackermann_drive" filename="libgazebo_ros_ackermann_drive.so">

    <ros>
      <namespace>/car</namespace>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>

    <!-- Wheel joints -->
    <front_left_joint>front_left_wheel_joint</front_left_joint>
    <front_right_joint>front_right_wheel_joint</front_right_joint>
    <rear_left_joint>rear_left_wheel_joint</rear_left_joint>
    <rear_right_joint>rear_right_wheel_joint</rear_right_joint>

    <!-- Steering joints -->
    <front_left_steering_joint>front_left_steering_joint</front_left_steering_joint>
    <front_right_steering_joint>front_right_steering_joint</front_right_steering_joint>

    <!-- Geometry -->
    <wheel_separation>1.2</wheel_separation>
    <wheelbase>1.5</wheelbase>  <!-- Front to rear axle distance -->
    <wheel_diameter>0.6</wheel_diameter>

    <!-- Limits -->
    <max_speed>10.0</max_speed>  <!-- m/s -->
    <max_steering_angle>0.6</max_steering_angle>  <!-- radians (~35 degrees) -->

    <!-- Update -->
    <update_rate>50</update_rate>

    <!-- Output -->
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>

  </plugin>
</gazebo>
```

**Control**: Same Twist message, but:
- `linear.x` = speed
- `angular.z` = steering angle (or turning rate depending on plugin)

## Multiple Robots

### Namespacing for Multi-Robot

**Launch file for two robots**:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    # Robot 1
    robot1_group = GroupAction([
        PushRosNamespace('robot1'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot1',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.5'
            ]
        )
    ])

    # Robot 2
    robot2_group = GroupAction([
        PushRosNamespace('robot2'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot2',  # Different entity name!
                '-topic', 'robot_description',
                '-x', '2', '-y', '0', '-z', '0.5'  # Different position!
            ]
        )
    ])

    return LaunchDescription([
        gazebo,
        robot1_group,
        robot2_group
    ])
```

**Result**:
- Robot 1 topics: `/robot1/cmd_vel`, `/robot1/odom`, `/robot1/scan`, etc.
- Robot 2 topics: `/robot2/cmd_vel`, `/robot2/odom`, `/robot2/scan`, etc.

**Controlling multiple robots**:

```cpp
// Publish to robot1
auto pub1 = node->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

// Publish to robot2
auto pub2 = node->create_publisher<geometry_msgs::msg::Twist>("/robot2/cmd_vel", 10);

// Move both
geometry_msgs::msg::Twist cmd;
cmd.linear.x = 0.5;

pub1->publish(cmd);
pub2->publish(cmd);
```

## Direct Robot Manipulation

### Applying Forces

**Use case**: Simulating wind, collisions, external disturbances

```cpp
#include "gazebo_msgs/srv/apply_body_wrench.hpp"

class ForceApplier : public rclcpp::Node
{
public:
    ForceApplier() : Node("force_applier")
    {
        // Service client
        client_ = this->create_client<gazebo_msgs::srv::ApplyBodyWrench>(
            "/gazebo/apply_body_wrench");

        // Wait for service
        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }
    }

    void apply_force(double fx, double fy, double fz, double duration)
    {
        auto request = std::make_shared<gazebo_msgs::srv::ApplyBodyWrench::Request>();

        request->body_name = "my_robot::base_link";
        request->reference_frame = "world";

        // Force vector
        request->wrench.force.x = fx;
        request->wrench.force.y = fy;
        request->wrench.force.z = fz;

        // Torque (if needed)
        request->wrench.torque.x = 0.0;
        request->wrench.torque.y = 0.0;
        request->wrench.torque.z = 0.0;

        // Duration
        request->duration = rclcpp::Duration::from_seconds(duration);

        // Call service
        auto future = client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Applied force: (%.1f, %.1f, %.1f)",
                   fx, fy, fz);
    }

private:
    rclcpp::Client<gazebo_msgs::srv::ApplyBodyWrench>::SharedPtr client_;
};
```

### Setting Model State (Teleportation)

**Use case**: Reset robot, test scenarios at specific positions

```cpp
#include "gazebo_msgs/srv/set_entity_state.hpp"

void teleport_robot(double x, double y, double z)
{
    auto client = node->create_client<gazebo_msgs::srv::SetEntityState>(
        "/gazebo/set_entity_state");

    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

    request->state.name = "my_robot";
    request->state.pose.position.x = x;
    request->state.pose.position.y = y;
    request->state.pose.position.z = z;

    request->state.pose.orientation.w = 1.0;  // Identity quaternion

    // Zero velocity
    request->state.twist.linear.x = 0.0;
    request->state.twist.angular.z = 0.0;

    auto future = client->async_send_request(request);
    // Robot instantly teleports to (x, y, z)
}
```

## Debugging Spawning Issues

### Common Problems

**Robot spawns but falls through ground**:
```
Problem: Missing collision geometry or inertia
Solution:
- Add <collision> to all links
- Set mass > 0
- Calculate proper inertia
```

**Robot explodes on spawn**:
```
Problem: Overlapping collision geometries, bad inertia
Solution:
- Check collision meshes don't overlap
- Verify inertia tensor is positive definite
- Reduce initial height (spawn closer to ground)
```

**Robot doesn't move with cmd_vel**:
```
Problem: Plugin not loaded, wrong joint names
Solution:
- Check Gazebo verbose output: gazebo --verbose
- Verify joint names in plugin match URDF
- Check topic: ros2 topic echo /robot/cmd_vel
- Verify wheels can rotate (not fixed joints)
```

**No odometry published**:
```
Problem: publish_odom=false, or wrong frame names
Solution:
- Set <publish_odom>true</publish_odom>
- Check frame names match TF tree
- Verify update_rate > 0
```

**Multiple robots have same topics**:
```
Problem: Missing namespace
Solution:
- Use <namespace> in plugin
- Use PushRosNamespace in launch file
- Ensure entity names are unique
```

### Debugging Commands

```bash
# List all entities in Gazebo
gz model --list

# Get model info
gz model --model-name=my_robot --info

# Check topics
ros2 topic list

# Monitor cmd_vel
ros2 topic echo /robot/cmd_vel

# Check TF tree
ros2 run tf2_tools view_frames

# Gazebo verbose output
gazebo --verbose

# Check loaded plugins
gz plugin --list
```

## Best Practices

**1. Test robot statically first**:
```bash
# Spawn without controllers
ros2 run gazebo_ros spawn_entity.py -entity test_robot -file robot.urdf

# Verify appearance, check for explosions
# Then add controllers
```

**2. Use realistic physics parameters**:
```xml
<!-- Match real robot specs -->
<wheel_separation>0.45</wheel_separation>  <!-- Measure on real robot -->
<max_wheel_torque>20</max_wheel_torque>    <!-- From motor specs -->
```

**3. Set use_sim_time**:
```python
# In all nodes
parameters=[{'use_sim_time': True}]
```

**4. Organize launch files**:
```python
# Separate concerns
gazebo.launch.py          # Just Gazebo
spawn_robot.launch.py     # Robot spawning
full_sim.launch.py        # Everything together
```

**5. Name entities uniquely**:
```python
# For multiple robots
arguments=['-entity', f'robot_{robot_id}']
```

## Summary

**Robot spawning**:
- Use `spawn_entity.py` from command line or launch files
- Provide URDF via file or topic
- Set initial position and orientation

**Differential drive**:
- Two wheels controlled independently
- Control via `geometry_msgs/Twist` on `cmd_vel`
- Publishes odometry and TF

**Ackermann steering**:
- Car-like steering
- Front wheels steer, rear wheels drive
- Similar control interface to diff drive

**Multi-robot**:
- Use unique entity names
- Namespace topics with PushRosNamespace
- Spawn at different positions

**Direct control**:
- Apply forces with services
- Teleport with SetEntityState
- Useful for testing and resets

## What's Next?

**Next Lesson**: Simulation Integration - Complete workflow from URDF to navigation

**You'll learn**:
- End-to-end simulation setup
- Integrating all components
- Navigation stack with Gazebo
- Testing and debugging workflows
- Production-ready simulation launch files

---

**Key Takeaway**: Gazebo plugins bring your robot to life - master spawning and control to create realistic simulations!
