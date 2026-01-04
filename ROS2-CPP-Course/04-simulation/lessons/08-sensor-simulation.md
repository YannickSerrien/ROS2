# Lesson 8: Sensor Simulation

## Learning Objectives

- Add simulated sensors to robot models
- Configure Gazebo sensor plugins
- Integrate sensors with ROS2 topics
- Simulate realistic sensor noise
- Understand sensor properties and limitations
- Use common sensors (lidar, camera, depth camera, IMU, GPS)
- Visualize sensor data in RViz

## Why Simulate Sensors?

**Benefits**:
- **Develop without hardware**: Test algorithms before buying expensive sensors
- **Reproducible data**: Same conditions every time
- **Extreme scenarios**: Test failure modes safely
- **Perfect ground truth**: Know exact robot position for algorithm validation

**Limitations**:
- Not 100% accurate to real sensors
- Simplified noise models
- Missing some real-world effects (lens distortion, calibration errors)

**Best practice**: Develop in simulation, validate on hardware

## Gazebo Sensor Plugins

### How Sensor Plugins Work

```
┌──────────────────────────────────────┐
│  Robot URDF/SDF with Sensor Link     │
│  (defines where sensor is mounted)   │
└────────────┬─────────────────────────┘
             │
┌────────────▼─────────────────────────┐
│  Gazebo Sensor Plugin                │
│  - Simulates sensor physics          │
│  - Generates data (lidar, image)     │
└────────────┬─────────────────────────┘
             │
┌────────────▼─────────────────────────┐
│  ROS2 Topic                          │
│  (sensor_msgs/LaserScan, Image, etc.)│
└──────────────────────────────────────┘
```

**Steps to add sensor**:
1. Create sensor link in URDF
2. Add Gazebo plugin in `<gazebo>` tag
3. Configure plugin parameters
4. Run simulation
5. Subscribe to sensor topic in RViz/your node

## Lidar (Laser Scanner)

### Adding Lidar to Robot

**URDF sensor link**:
```xml
<!-- Lidar link -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
    <material name="black">
      <color rgba="0.2 0.2 0.2 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.001" ixy="0" ixz="0"
             iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Mount on robot -->
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
</joint>
```

**Gazebo plugin** (add to URDF in `<gazebo>` tag):
```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <visualize>true</visualize>

    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180 degrees -->
          <max_angle>3.14159</max_angle>   <!-- +180 degrees -->
        </horizontal>
      </scan>

      <range>
        <min>0.15</min>    <!-- Minimum detection distance -->
        <max>10.0</max>    <!-- Maximum detection distance -->
        <resolution>0.01</resolution>
      </range>

      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>

    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>

  </sensor>
</gazebo>
```

**Result**: Publishes on `/robot/scan` topic

**Viewing in RViz**:
```bash
# Terminal 1: Gazebo with robot
ros2 launch my_robot gazebo.launch.py

# Terminal 2: RViz
rviz2

# In RViz:
# 1. Add → LaserScan
# 2. Topic: /robot/scan
# 3. Fixed Frame: lidar_link (or map/odom if TF available)
```

### Lidar Parameters Explained

```xml
<samples>360</samples>
<!-- Number of laser rays (more = denser point cloud, slower) -->

<min_angle>-3.14159</min_angle>
<max_angle>3.14159</max_angle>
<!-- Field of view in radians (2π = 360 degrees) -->

<min>0.15</min>
<max>10.0</max>
<!-- Detection range in meters -->

<update_rate>10</update_rate>
<!-- Scans per second (Hz) -->

<noise>
  <stddev>0.01</stddev>
</noise>
<!-- Gaussian noise std dev in meters (realistic: 0.01-0.03) -->
```

**Common configurations**:

```xml
<!-- SICK LMS100: 270° scanner -->
<samples>270</samples>
<min_angle>-2.35619</min_angle>  <!-- -135° -->
<max_angle>2.35619</max_angle>   <!-- +135° -->
<max>20.0</max>

<!-- Hokuyo URG-04LX: 240° scanner -->
<samples>682</samples>
<min_angle>-2.09440</min_angle>
<max_angle>2.09440</max_angle>
<max>5.6</max>

<!-- Velodyne 2D slice: 360° -->
<samples>720</samples>
<min_angle>-3.14159</min_angle>
<max_angle>3.14159</max_angle>
<max>100.0</max>
```

## Camera

### RGB Camera

**Camera link**:
```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
    <material name="black"/>
  </visual>

  <collision>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0"
             iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.3 0 0.2" rpy="0 0 0"/>
</joint>
```

**Gazebo camera plugin**:
```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>  <!-- FPS -->
    <visualize>true</visualize>

    <camera>
      <horizontal_fov>1.57</horizontal_fov>  <!-- 90 degrees in radians -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>100</far>
      </clip>

      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>

    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/image_raw:=camera/image_raw</remapping>
        <remapping>~/camera_info:=camera/camera_info</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>

  </sensor>
</gazebo>
```

**Topics published**:
- `/robot/camera/image_raw` - Raw image (sensor_msgs/Image)
- `/robot/camera/camera_info` - Camera calibration

**Viewing in RViz**:
```
Add → Image
Image Topic: /robot/camera/image_raw
```

### Depth Camera (RGBD)

**Example: Intel RealSense / Kinect**

```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <always_on>true</always_on>
    <update_rate>20</update_rate>

    <camera>
      <horizontal_fov>1.57</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.05</near>
        <far>8.0</far>
      </clip>
    </camera>

    <plugin name="depth_camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>camera_link</frame_name>

      <!-- Publish point cloud -->
      <min_depth>0.1</min_depth>
      <max_depth>8.0</max_depth>
    </plugin>

  </sensor>
</gazebo>
```

**Topics**:
- `/robot/depth_camera/depth/image_raw` - Depth image
- `/robot/depth_camera/depth/points` - Point cloud (sensor_msgs/PointCloud2)
- `/robot/depth_camera/rgb/image_raw` - RGB image

**Viewing point cloud in RViz**:
```
Add → PointCloud2
Topic: /robot/depth_camera/depth/points
```

## IMU (Inertial Measurement Unit)

**Purpose**: Measure acceleration and angular velocity

**IMU link**:
```xml
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
  </visual>

  <collision>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.00001" ixy="0" ixz="0"
             iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

**Gazebo IMU plugin**:
```xml
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>

    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0001</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0001</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0001</stddev>
          </noise>
        </z>
      </angular_velocity>

      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
      <frame_name>imu_link</frame_name>
    </plugin>

  </sensor>
</gazebo>
```

**Topic**: `/robot/imu/data` (sensor_msgs/Imu)

**IMU data includes**:
- Orientation (quaternion)
- Angular velocity (rad/s)
- Linear acceleration (m/s²)

## GPS

**GPS plugin**:
```xml
<gazebo reference="base_link">
  <sensor name="gps" type="gps">
    <always_on>true</always_on>
    <update_rate>10</update_rate>

    <gps>
      <position_sensing>
        <horizontal>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2.0</stddev>  <!-- 2m accuracy -->
          </noise>
        </horizontal>
        <vertical>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>4.0</stddev>  <!-- 4m vertical accuracy -->
          </noise>
        </vertical>
      </position_sensing>

      <velocity_sensing>
        <horizontal>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </noise>
        </horizontal>
        <vertical>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </noise>
        </vertical>
      </velocity_sensing>
    </gps>

    <plugin name="gps_plugin" filename="libgazebo_ros_gps_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=gps/fix</remapping>
      </ros>
      <frame_name>base_link</frame_name>
    </plugin>

  </sensor>
</gazebo>
```

**Topic**: `/robot/gps/fix` (sensor_msgs/NavSatFix)

## Multi-Sensor Robot Example

**Complete robot with multiple sensors**:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="sensor_robot">

  <xacro:property name="lidar_x" value="0.2"/>
  <xacro:property name="camera_x" value="0.3"/>

  <!-- Base link -->
  <link name="base_link">
    <!-- Base geometry -->
  </link>

  <!-- Lidar -->
  <link name="lidar_link">
    <visual>
      <geometry><cylinder radius="0.05" length="0.07"/></geometry>
      <material name="black"><color rgba="0.2 0.2 0.2 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="0.05" length="0.07"/></geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="${lidar_x} 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <geometry><box size="0.02 0.05 0.02"/></geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry><box size="0.02 0.05 0.02"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${camera_x} 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- IMU -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <!-- Lidar configuration -->
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <!-- Camera configuration -->
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <!-- IMU configuration -->
    </sensor>
  </gazebo>

</robot>
```

## Sensor Noise Models

### Why Add Noise?

**Real sensors have**:
- Measurement uncertainty
- Environmental interference
- Manufacturing variations

**Simulating noise**:
- Makes algorithms more robust
- Closer to real-world performance
- Tests edge cases

### Gaussian Noise

**Most common model**:
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>  <!-- Standard deviation -->
</noise>
```

**Interpretation**:
- 68% of measurements within ±1 stddev
- 95% within ±2 stddev
- 99.7% within ±3 stddev

**Example**: Lidar with `stddev=0.01` (1cm)
- Most readings ±1cm from true value
- 5% of readings >2cm off

### Realistic Noise Levels

**Lidar**:
```xml
<stddev>0.01</stddev>  <!-- 1cm typical -->
<stddev>0.03</stddev>  <!-- 3cm low-cost -->
```

**Camera pixel intensity**:
```xml
<stddev>0.007</stddev>  <!-- ~2/255 intensity variation -->
```

**IMU angular velocity**:
```xml
<stddev>0.0001</stddev>  <!-- 0.01 deg/s for good IMU -->
<stddev>0.001</stddev>   <!-- 0.1 deg/s for cheap IMU -->
```

**IMU linear acceleration**:
```xml
<stddev>0.01</stddev>  <!-- 0.01 m/s² -->
```

**GPS**:
```xml
<stddev>2.0</stddev>   <!-- 2m consumer GPS -->
<stddev>0.01</stddev>  <!-- 1cm RTK GPS -->
```

## Sensor Visualization

### RViz Display Configuration

**For comprehensive sensor view**:

```yaml
Panels:
  - Class: rviz_common/Displays

Displays:
  - Class: rviz_default_plugins/RobotModel
    Description Topic: /robot_description

  - Class: rviz_default_plugins/TF
    Show Names: true

  - Class: rviz_default_plugins/LaserScan
    Topic: /robot/scan
    Size: 0.05
    Color: 255; 0; 0

  - Class: rviz_default_plugins/Image
    Topic: /robot/camera/image_raw

  - Class: rviz_default_plugins/PointCloud2
    Topic: /robot/depth_camera/depth/points
    Color Transformer: RGB8

  - Class: rviz_default_plugins/Imu
    Topic: /robot/imu/data
```

## Best Practices

**1. Match real sensor specs**:
```xml
<!-- Research actual sensor specifications -->
<!-- Example: Hokuyo URG-04LX -->
<samples>682</samples>
<min_angle>-2.09440</min_angle>
<max_angle>2.09440</max_angle>
<range>
  <min>0.02</min>
  <max>5.6</max>
</range>
```

**2. Use appropriate update rates**:
```xml
<!-- Match real sensor frequencies -->
<update_rate>30</update_rate>  <!-- Camera: 30 FPS -->
<update_rate>10</update_rate>  <!-- Lidar: 10 Hz -->
<update_rate>100</update_rate> <!-- IMU: 100 Hz -->
```

**3. Add realistic noise**:
```xml
<!-- Don't use perfect sensors -->
<noise>
  <stddev>0.01</stddev>  <!-- Add appropriate noise -->
</noise>
```

**4. Proper sensor placement**:
```xml
<!-- Mount sensors realistically -->
<!-- Lidar on top, cameras forward, IMU near center of mass -->
<origin xyz="0.2 0 0.15" rpy="0 0 0"/>
```

**5. Use visualize during development**:
```xml
<visualize>true</visualize>  <!-- See sensor rays in Gazebo -->
<!-- Set false for production to improve performance -->
```

## Common Issues

**Sensor not publishing**:
```bash
# Check if topic exists
ros2 topic list | grep scan

# Check for plugin errors
gazebo --verbose

# Verify sensor link exists in TF
ros2 run tf2_ros tf2_echo base_link lidar_link
```

**Poor performance**:
- Reduce update_rate
- Lower image resolution
- Fewer lidar samples
- Disable visualize

**Incorrect data**:
- Check frame_id matches TF tree
- Verify sensor orientation (rpy in joint)
- Check min/max range values

## Summary

**Simulated sensors provide**:
- Safe algorithm development
- Reproducible testing
- Ground truth for validation

**Common sensors**:
- **Lidar**: 2D/3D range finding
- **Camera**: RGB images
- **Depth camera**: RGB + depth (point clouds)
- **IMU**: Orientation, acceleration, angular velocity
- **GPS**: Global position

**Key configuration**:
- Update rate (Hz)
- Field of view / range
- Noise parameters
- Frame name for TF

**Workflow**:
1. Add sensor link to URDF
2. Configure Gazebo plugin
3. Launch simulation
4. Subscribe to topics
5. Visualize in RViz

## What's Next?

**Next Lesson**: Robot Spawning and Control - Launch robots programmatically

**You'll learn**:
- Spawning robots from launch files
- Multiple robot instances
- Namespacing for multi-robot
- Robot controllers (diff drive, ackermann)
- Applying forces and velocities

---

**Key Takeaway**: Simulated sensors let you develop perception algorithms before hardware arrives - configure them realistically for best sim-to-real transfer!
