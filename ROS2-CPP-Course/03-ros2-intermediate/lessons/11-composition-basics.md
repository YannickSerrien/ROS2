# Lesson 11: Composition Basics

## Learning Objectives

- Understand what node composition is and why it matters
- Compare separate processes vs composed nodes
- Understand intra-process communication
- Identify when to use composition
- Convert regular nodes to component nodes
- Understand zero-copy message passing

## What is Composition?

**Traditional ROS2**: Each node runs in a separate process
```
Process 1: Camera Node
Process 2: Image Processor
Process 3: Object Detector
```
Messages serialized → sent via IPC → deserialized

**Composition**: Multiple nodes in ONE process
```
Single Process:
├─ Camera Node (component)
├─ Image Processor (component)
└─ Object Detector (component)
```
Messages passed by pointer (zero-copy!)

## Why Composition?

### Performance Benefits

**Separate Processes**:
1. Node A publishes image
2. Serialize image to bytes
3. Send via inter-process communication (IPC)
4. Node B deserializes bytes
5. Process image

**Composed Nodes**:
1. Node A publishes image
2. Node B receives pointer directly (zero-copy!)
3. Process image

**Result**: 10-100x faster for large messages (images, point clouds)

### Resource Benefits

**Separate Processes**:
- 10 nodes = 10 processes
- Each has memory overhead
- OS context switching between processes

**Composition**:
- 10 nodes = 1 process
- Shared memory
- No context switching

### Use Cases

**Good for Composition**:
- High-frequency data (camera, lidar)
- Large messages (images, point clouds)
- Real-time processing pipelines
- Resource-constrained systems (embedded)

**Not Needed**:
- Low-frequency data
- Small messages
- Independent systems
- When isolation is important

## Intra-Process Communication

**Inter-Process** (default):
```cpp
// Publisher
auto msg = std::make_unique<Image>();
publisher->publish(std::move(msg));
// msg is serialized and sent

// Subscriber
void callback(const Image::SharedPtr msg) {
    // msg was deserialized
}
```

**Intra-Process** (composition):
```cpp
// Publisher
auto msg = std::make_unique<Image>();
publisher->publish(std::move(msg));
// msg pointer directly passed!

// Subscriber
void callback(const Image::SharedPtr msg) {
    // Same pointer! No copy!
}
```

**Zero-Copy**: Subscribers receive the original object pointer, not a copy.

## Component Nodes vs Regular Nodes

### Regular Node (Process)

```cpp
// separate_node.cpp
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Component Node (Composable)

```cpp
// component_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

class MyComponentNode : public rclcpp::Node
{
public:
    MyComponentNode(const rclcpp::NodeOptions & options)
        : Node("my_component", options)
    {
        // Node logic here
    }
};

// Register as component
RCLCPP_COMPONENTS_REGISTER_NODE(MyComponentNode)
```

**Key Differences**:
1. Constructor takes `NodeOptions`
2. No `main()` function
3. `RCLCPP_COMPONENTS_REGISTER_NODE` macro
4. Built as shared library (`.so`), not executable

## Converting to Component

**Before** (regular node):
```cpp
class TalkerNode : public rclcpp::Node
{
public:
    TalkerNode() : Node("talker")  // No NodeOptions
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = this->create_wall_timer(1s, [this]() { this->publish(); });
    }

private:
    void publish() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello";
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalkerNode>());
    rclcpp::shutdown();
    return 0;
}
```

**After** (component):
```cpp
#include <rclcpp_components/register_node_macro.hpp>

class TalkerComponent : public rclcpp::Node
{
public:
    // Add NodeOptions parameter
    TalkerComponent(const rclcpp::NodeOptions & options)
        : Node("talker", options)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = this->create_wall_timer(1s, [this]() { this->publish(); });
    }

private:
    void publish() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello";
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Register component (replaces main)
RCLCPP_COMPONENTS_REGISTER_NODE(TalkerComponent)
```

## Building Components

### CMakeLists.txt

```cmake
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# Build as shared library
add_library(talker_component SHARED
  src/talker_component.cpp
)

ament_target_dependencies(talker_component
  rclcpp
  rclcpp_components
  std_msgs
)

# Register component
rclcpp_components_register_node(talker_component
  PLUGIN "TalkerComponent"
  EXECUTABLE talker_node
)

# Install library
install(TARGETS talker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

### package.xml

```xml
<depend>rclcpp</depend>
<depend>rclcpp_components</depend>
```

## Running Components

### Method 1: Standalone (like regular node)

```bash
# The EXECUTABLE from CMakeLists.txt
ros2 run my_package talker_node
```

### Method 2: Manual Composition

```bash
# Start component container
ros2 run rclcpp_components component_container

# In another terminal, load component
ros2 component load /ComponentManager my_package TalkerComponent

# List loaded components
ros2 component list

# Unload component
ros2 component unload /ComponentManager 1
```

### Method 3: Launch File (next lesson)

```python
# See Lesson 12 for composition in launch files
```

## Performance Comparison

**Example**: Publishing 1920x1080 images at 30 Hz

**Separate Processes**:
- Latency: ~50ms
- CPU usage: 40%
- Memory copies: 2

**Composed Nodes**:
- Latency: ~5ms (10x faster!)
- CPU usage: 15%
- Memory copies: 0 (zero-copy!)

## Summary

Composition runs multiple nodes in one process:

**Benefits**:
- Zero-copy message passing
- Lower latency
- Reduced memory usage
- Better for embedded systems

**Conversion Steps**:
1. Add `NodeOptions` parameter to constructor
2. Remove `main()` function
3. Add `RCLCPP_COMPONENTS_REGISTER_NODE` macro
4. Update CMakeLists.txt (shared library)

**When to Use**:
- High-frequency / large messages
- Performance-critical pipelines
- Resource-constrained platforms

## What's Next?

- **Lesson 12**: Component Nodes - Practical implementation and launch integration

---

**Next Lesson**: [Lesson 12: Component Nodes](12-component-nodes.md)
