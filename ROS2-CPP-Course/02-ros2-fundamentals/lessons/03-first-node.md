# Lesson 3: Your First ROS2 Node

## Learning Objectives

- Create a minimal ROS2 C++ node
- Understand node structure and lifecycle
- Build and run a ROS2 node
- Use RCLCPP logging
- Understand the main function pattern

## Introduction

In this lesson, you'll create your first ROS2 node in C++! A **node** is an executable that uses ROS2 to communicate with other nodes.

## Minimal ROS2 Node

Here's the simplest possible ROS2 node:

```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = std::make_shared<rclcpp::Node>("my_first_node");

    // Log a message
    RCLCPP_INFO(node->get_logger(), "Hello, ROS2!");

    // Keep node alive
    rclcpp::spin(node);

    // Shutdown
    rclcpp::shutdown();
    return 0;
}
```

### Breaking It Down

#### 1. Include ROS2 Header
```cpp
#include <rclcpp/rclcpp.hpp>
```
- `rclcpp` is the ROS2 C++ client library
- Provides all core ROS2 functionality

#### 2. Initialize ROS2
```cpp
rclcpp::init(argc, argv);
```
- **Must be called first** before any ROS2 operations
- Parses ROS2 arguments from command line
- Sets up DDS communication

#### 3. Create Node
```cpp
auto node = std::make_shared<rclcpp::Node>("my_first_node");
```
- Creates a node with name "my_first_node"
- Returns `shared_ptr` (ROS2 uses shared ownership)
- Node name must be unique in the system

#### 4. Log Message
```cpp
RCLCPP_INFO(node->get_logger(), "Hello, ROS2!");
```
- Logs info-level message
- `get_logger()` returns node's logger
- Similar to `print()` in Python but with log levels

#### 5. Spin (Keep Alive)
```cpp
rclcpp::spin(node);
```
- **Blocks** and processes callbacks
- Node stays alive until Ctrl+C
- Handles incoming messages, services, etc.

#### 6. Shutdown
```cpp
rclcpp::shutdown();
```
- Cleanup ROS2 resources
- Should be called before program exits

## Node Class Pattern

The more common pattern is to create a **node class**:

```cpp
#include <rclcpp/rclcpp.hpp>

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        RCLCPP_INFO(this->get_logger(), "Node has been started!");
    }

private:
    // Add members here later (publishers, subscribers, etc.)
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Why Use a Class?

- **Encapsulation**: Keep node data and behavior together
- **Inheritance**: Use `rclcpp::Node` functionality
- **Organization**: Clean code structure
- **ROS2 pattern**: Standard approach in ROS2

## Complete Example: Counter Node

Let's create a node that counts and prints:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CounterNode : public rclcpp::Node {
public:
    CounterNode() : Node("counter_node"), counter_(0) {
        RCLCPP_INFO(this->get_logger(), "Counter node started");

        // Create timer (1 second interval)
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&CounterNode::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

**Output:**
```
[INFO] [counter_node]: Counter node started
[INFO] [counter_node]: Counter: 1
[INFO] [counter_node]: Counter: 2
[INFO] [counter_node]: Counter: 3
...
```

## Logging Levels

ROS2 provides multiple logging levels:

```cpp
RCLCPP_DEBUG(logger, "Detailed debug info");     // Debug
RCLCPP_INFO(logger, "General information");      // Info (default)
RCLCPP_WARN(logger, "Warning message");          // Warning
RCLCPP_ERROR(logger, "Error occurred");          // Error
RCLCPP_FATAL(logger, "Fatal error!");            // Fatal
```

### Conditional Logging

```cpp
// Log only once
RCLCPP_INFO_ONCE(logger, "This prints once");

// Log every N times
RCLCPP_INFO_SKIPFIRST(logger, "Skips first call");

// Log with throttle (max once per second)
RCLCPP_INFO_THROTTLE(logger, *get_clock(), 1000, "At most once/sec");
```

### Formatted Logging

```cpp
int value = 42;
double temperature = 23.5;
std::string sensor = "temp1";

RCLCPP_INFO(logger, "Sensor %s reading: %.2f (value: %d)",
    sensor.c_str(), temperature, value);
```

## Building Your First Node

### Step 1: Create Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_first_package \
    --dependencies rclcpp
```

### Step 2: Write Node Code

Create `src/my_first_node.cpp`:

```cpp
#include <rclcpp/rclcpp.hpp>

class MyFirstNode : public rclcpp::Node {
public:
    MyFirstNode() : Node("my_first_node") {
        RCLCPP_INFO(this->get_logger(), "Hello from my first node!");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyFirstNode>();
    RCLCPP_INFO(node->get_logger(), "Node is spinning...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Step 3: Update CMakeLists.txt

Add to `CMakeLists.txt`:

```cmake
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add executable
add_executable(my_first_node src/my_first_node.cpp)

# Link dependencies
ament_target_dependencies(my_first_node rclcpp)

# Install
install(TARGETS
    my_first_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### Step 4: Build

```bash
cd ~/ros2_ws
colcon build --packages-select my_first_package
```

### Step 5: Source and Run

```bash
source install/setup.bash
ros2 run my_first_package my_first_node
```

## Inspecting Your Node

While node is running, open another terminal:

```bash
# List all nodes
ros2 node list

# Get node info
ros2 node info /my_first_node
```

## Node Naming

### Node Names
- Must be **unique** in the system
- Use **lowercase** with underscores
- Be **descriptive**: `camera_driver` not `node1`

```cpp
// Good names
Node("camera_driver")
Node("lidar_processor")
Node("motion_controller")

// Bad names
Node("node1")
Node("MyNode")
Node("n")
```

### Namespaces

Group related nodes:

```cpp
// In code
auto node = std::make_shared<rclcpp::Node>("sensor", "robot1");
// Creates /robot1/sensor

// Or via command line
ros2 run pkg node --ros-args -r __ns:=/robot1
// Creates /robot1/node_name
```

## Node Parameters (Preview)

Nodes can have configurable parameters:

```cpp
class ConfigurableNode : public rclcpp::Node {
public:
    ConfigurableNode() : Node("configurable_node") {
        // Declare parameter with default value
        this->declare_parameter("update_rate", 10.0);

        // Get parameter value
        double rate = this->get_parameter("update_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "Update rate: %.2f Hz", rate);
    }
};
```

Set parameter when running:

```bash
ros2 run pkg node --ros-args -p update_rate:=20.0
```

## Common Patterns

### Pattern 1: Simple Node

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("simple");
    RCLCPP_INFO(node->get_logger(), "Running");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Pattern 2: Node Class

```cpp
class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        // Initialize in constructor
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```

### Pattern 3: Node with Cleanup

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    {
        auto node = std::make_shared<MyNode>();
        rclcpp::spin(node);
    }  // Node destroyed here

    rclcpp::shutdown();
    return 0;
}
```

## Common Pitfalls

### ❌ Forgetting to Initialize

```cpp
// BAD - will crash!
int main(int argc, char** argv) {
    auto node = std::make_shared<rclcpp::Node>("node");  // Missing init!
    rclcpp::spin(node);
    return 0;
}

// GOOD
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Initialize first!
    auto node = std::make_shared<rclcpp::Node>("node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### ❌ Not Using spin()

```cpp
// BAD - node exits immediately!
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node");
    // Missing spin - program exits!
    rclcpp::shutdown();
    return 0;
}
```

### ❌ Duplicate Node Names

```cpp
// BAD - both nodes named "node"
auto node1 = std::make_shared<rclcpp::Node>("node");
auto node2 = std::make_shared<rclcpp::Node>("node");  // Conflict!

// GOOD - unique names
auto node1 = std::make_shared<rclcpp::Node>("camera_node");
auto node2 = std::make_shared<rclcpp::Node>("lidar_node");
```

## Summary

**Key Takeaways:**
- Nodes are independent processes in ROS2
- Always `init` → create node → `spin` → `shutdown`
- Inherit from `rclcpp::Node` for clean structure
- Use logging instead of cout
- Node names must be unique

**Critical Pattern:**
```cpp
rclcpp::init(argc, argv);
auto node = std::make_shared<MyNode>();
rclcpp::spin(node);
rclcpp::shutdown();
```

## Practice Exercise

Create a node that:
1. Prints "Hello" when started
2. Counts from 1-10 with 1 second delay
3. Prints "Goodbye" and exits

## What's Next?

- **Next Lesson**: [Topics: Publishers](04-topics-publishers.md) - Send data
- **Also See**: [Logging](10-logging.md) - Advanced logging techniques

---

**Congratulations!** You've created your first ROS2 node! 🎉
