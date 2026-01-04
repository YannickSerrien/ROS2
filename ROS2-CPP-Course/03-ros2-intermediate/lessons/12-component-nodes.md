# Lesson 12: Component Nodes

## Learning Objectives

- Build complete component-based systems
- Launch composed nodes from launch files
- Use ComposableNodeContainer
- Mix composed and standalone nodes
- Debug composition issues
- Apply composition to real-world systems

## Component Container

The container manages component lifecycle:

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_package',
                plugin='TalkerComponent',
                name='talker'
            ),
            ComposableNode(
                package='my_package',
                plugin='ListenerComponent',
                name='listener'
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
```

## Complete Example: Image Pipeline

### Component 1: Camera Publisher

```cpp
// camera_component.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraComponent : public rclcpp::Node
{
public:
    CameraComponent(const rclcpp::NodeOptions & options)
        : Node("camera", options)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // 30 FPS
            [this]() { this->publish_image(); });

        RCLCPP_INFO(this->get_logger(), "Camera component started");
    }

private:
    void publish_image()
    {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        // Fill image data...
        msg->header.stamp = this->now();
        msg->width = 640;
        msg->height = 480;
        // ... set image data

        publisher_->publish(std::move(msg));  // Zero-copy with composition!
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(CameraComponent)
```

### Component 2: Image Processor

```cpp
// processor_component.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

class ProcessorComponent : public rclcpp::Node
{
public:
    ProcessorComponent(const rclcpp::NodeOptions & options)
        : Node("processor", options)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                this->image_callback(msg);
            });

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_processed", 10);

        RCLCPP_INFO(this->get_logger(), "Processor component started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Process image (zero-copy input!)
        auto processed = std::make_unique<sensor_msgs::msg::Image>(*msg);

        // Apply processing...
        // (in real code: edge detection, filtering, etc.)

        publisher_->publish(std::move(processed));
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(ProcessorComponent)
```

### CMakeLists.txt

```cmake
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Camera component
add_library(camera_component SHARED src/camera_component.cpp)
ament_target_dependencies(camera_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(camera_component
  PLUGIN "CameraComponent"
  EXECUTABLE camera_node
)

# Processor component
add_library(processor_component SHARED src/processor_component.cpp)
ament_target_dependencies(processor_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(processor_component
  PLUGIN "ProcessorComponent"
  EXECUTABLE processor_node
)

# Install
install(TARGETS camera_component processor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

### Launch File

```python
"""
Launch file for composed image pipeline
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Composed pipeline in single process
    container = ComposableNodeContainer(
        name='image_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Camera component
            ComposableNode(
                package='image_pipeline',
                plugin='CameraComponent',
                name='camera',
                parameters=[{
                    'frame_rate': 30,
                    'resolution': '640x480'
                }]
            ),
            # Processor component
            ComposableNode(
                package='image_pipeline',
                plugin='ProcessorComponent',
                name='processor',
                parameters=[{
                    'algorithm': 'edge_detection'
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
```

**Running**:
```bash
ros2 launch image_pipeline composed.launch.py
```

## Mixing Composed and Standalone

You can mix both in same system:

```python
from launch_ros.actions import Node, ComposableNodeContainer

def generate_launch_description():
    # Composed: High-frequency image pipeline
    vision_container = ComposableNodeContainer(
        name='vision_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='camera_pkg', plugin='CameraComponent'),
            ComposableNode(package='vision_pkg', plugin='DetectorComponent')
        ]
    )

    # Standalone: Low-frequency planner (doesn't need composition)
    planner_node = Node(
        package='planning',
        executable='planner_node'
    )

    # Standalone: UI (runs separately for stability)
    ui_node = Node(
        package='ui',
        executable='dashboard'
    )

    return LaunchDescription([vision_container, planner_node, ui_node])
```

## Parameters in Components

```python
ComposableNode(
    package='my_package',
    plugin='MyComponent',
    name='my_node',
    parameters=[{
        'param1': 'value1',
        'param2': 42
    }],
    remappings=[
        ('/input', '/sensor/data')
    ],
    extra_arguments=[{
        'use_intra_process_comms': True  # Enable zero-copy
    }]
)
```

## Debugging Composition

### Check Available Components

```bash
# List component plugins
ros2 component types

# Check specific package
ros2 component types my_package
```

### Runtime Monitoring

```bash
# List running containers
ros2 component list

# Show components in container
ros2 component list /container_name
```

### Common Issues

**1. Component Not Found**
```
Error: Could not find component 'MyComponent'
```
- Check plugin name matches `RCLCPP_COMPONENTS_REGISTER_NODE`
- Rebuild package: `colcon build --packages-select my_package`

**2. Segmentation Fault**
```
Segmentation fault (core dumped)
```
- Check NodeOptions is passed to base class
- Ensure all shared_ptrs are valid

**3. Intra-Process Not Working**
```
Still seeing serialization overhead
```
- Enable explicitly:
  ```python
  extra_arguments=[{'use_intra_process_comms': True}]
  ```
- Check both nodes are in same container

## Performance Tuning

### QoS for Zero-Copy

```cpp
// Publisher
auto qos = rclcpp::QoS(10).reliable();
publisher_ = this->create_publisher<Image>("topic", qos);

// Subscriber
subscription_ = this->create_subscription<Image>(
    "topic", qos,
    [this](Image::SharedPtr msg) { /* zero-copy! */ });
```

### Memory Pooling

For high-frequency components, pre-allocate:

```cpp
std::vector<std::unique_ptr<Image>> image_pool_;

void initialize_pool() {
    for (int i = 0; i < 10; ++i) {
        image_pool_.push_back(std::make_unique<Image>());
    }
}

void publish() {
    auto msg = std::move(image_pool_.back());
    image_pool_.pop_back();

    // Fill msg...
    publisher_->publish(std::move(msg));

    // Replenish pool
    image_pool_.push_back(std::make_unique<Image>());
}
```

## When to Use Composition

**Use Composition**:
✓ High-frequency data (>10 Hz)
✓ Large messages (images, point clouds)
✓ Real-time requirements
✓ Embedded/resource-constrained
✓ Tight coupling between nodes

**Use Separate Processes**:
✓ Low-frequency data
✓ Small messages
✓ Need isolation (crash isolation)
✓ Different programming languages
✓ Independent development

## Summary

Component nodes enable high-performance ROS2 systems:

**Key Points**:
- Multiple nodes in one process
- Zero-copy message passing
- 10-100x faster for large messages
- Configured via launch files
- Mix with standalone nodes

**Conversion**:
```cpp
// Add NodeOptions
MyNode(const rclcpp::NodeOptions & options) : Node("name", options) {}

// Register
RCLCPP_COMPONENTS_REGISTER_NODE(MyNode)

// Build as shared library
add_library(my_component SHARED ...)
rclcpp_components_register_node(my_component ...)
```

**Launch**:
```python
ComposableNodeContainer(
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(package='pkg', plugin='Component')
    ]
)
```

## Module 3 Complete!

Congratulations! You've completed all Module 3 lessons:

**Actions**: Long-running tasks with feedback
**TF2**: Coordinate transformations
**Launch Files**: System orchestration
**Composition**: Performance optimization

You're now ready for advanced ROS2 development!

## What's Next?

- **Practice**: Complete Module 3 exercises
- **Build**: Create the robot arm mini-project
- **Continue**: Module 4 - Simulation with Gazebo and RViz

---

**Module 3 Exercises**: [exercises/](../exercises/)
**Mini-Project**: [mini-project/](../mini-project/)
**Next Module**: [Module 4: Simulation](../../04-simulation/)
