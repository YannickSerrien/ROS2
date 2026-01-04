# Sensor Robot Simulation

Complete mobile robot simulation with lidar, camera, IMU, and differential drive in Gazebo.

## What This Demonstrates

- **Complete robot**: Mobile base with sensors
- **Gazebo sensors**: Lidar, camera, IMU plugins
- **Differential drive**: Robot control with cmd_vel
- **Odometry**: Published odom and TF
- **Integration**: Gazebo + RViz + robot_state_publisher

## Robot Features

### Sensors

**Lidar** (`/scan`):
- 360° coverage
- 10 Hz update rate
- 10m range
- sensor_msgs/LaserScan

**Camera** (`/camera/image_raw`):
- 640x480 resolution
- 30 FPS
- 90° FOV
- sensor_msgs/Image

**IMU** (`/imu/data`):
- 100 Hz update rate
- Orientation, angular velocity, linear acceleration
- sensor_msgs/Imu

### Control

**Differential Drive**:
- Subscribe to `/cmd_vel` (geometry_msgs/Twist)
- Publishes `/odom` (nav_msgs/Odometry)
- Publishes odom → base_link TF

## Quick Start

```bash
# Build
colcon build --packages-select sensor_robot_simulation
source install/setup.bash

# Launch simulation (Gazebo + RViz)
ros2 launch sensor_robot_simulation simulation.launch.py
```

**Result**:
- Gazebo opens with robot
- RViz shows robot model, lidar scan, camera image, odometry

## Manual Control

```bash
# Terminal 1: Launch simulation
ros2 launch sensor_robot_simulation simulation.launch.py

# Terminal 2: Control robot
# Forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Verify Sensors

```bash
# Check topics
ros2 topic list

# Echo lidar data
ros2 topic echo /scan

# Echo camera info
ros2 topic echo /camera/camera_info

# Echo IMU data
ros2 topic echo /imu/data

# Echo odometry
ros2 topic echo /odom

# Check TF tree
ros2 run tf2_tools view_frames
```

## Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | Lidar data |
| `/camera/image_raw` | sensor_msgs/Image | Camera image |
| `/camera/camera_info` | sensor_msgs/CameraInfo | Camera calibration |
| `/imu/data` | sensor_msgs/Imu | IMU measurements |
| `/odom` | nav_msgs/Odometry | Robot odometry |

## Topics Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |

## TF Tree

```
odom
  └─ base_link
       ├─ left_wheel
       ├─ right_wheel
       ├─ lidar_link
       ├─ camera_link
       └─ imu_link
```

## Gazebo Plugins Used

**libgazebo_ros_diff_drive.so**:
- Differential drive controller
- Odometry publishing
- TF publishing

**libgazebo_ros_ray_sensor.so**:
- Lidar simulation

**libgazebo_ros_camera.so**:
- Camera simulation

**libgazebo_ros_imu_sensor.so**:
- IMU simulation

## URDF Structure

The robot URDF includes:
- Base link with wheels (differential drive)
- Sensor links (lidar, camera, IMU)
- Gazebo plugin configurations
- Inertial properties for physics

## Integration Example

This demonstrates a complete simulation pipeline:

```
URDF → robot_state_publisher → /robot_description
                             → TF (static transforms)

Gazebo → Physics simulation
      → Sensor data (/scan, /camera, /imu)
      → Odometry (/odom, odom→base_link TF)

/cmd_vel → Gazebo diff_drive plugin → Robot motion

RViz ← All topics/TF → Visualization
```

## Related Lessons

- Module 4, Lesson 8: Sensor Simulation
- Module 4, Lesson 9: Robot Spawning and Control
- Module 4, Lesson 10: Simulation Integration

## Key Concepts

**Sensor plugins**: Gazebo plugins that generate realistic sensor data
**Differential drive**: Two-wheel robot control
**TF tree**: Coordinate frame relationships
**robot_state_publisher**: Publishes robot structure to TF
**Odometry**: Robot position estimation from wheel encoders

## Next Steps

Try these experiments:
1. Add more obstacles in Gazebo (Insert tab)
2. Write a node to follow walls using `/scan`
3. Create a simple navigation algorithm
4. Add more sensors (depth camera, GPS)
5. Test in the obstacle_course.world from gazebo_world_example
