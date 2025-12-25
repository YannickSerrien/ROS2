# Lesson 9: Namespaces and Modules

## Learning Objectives

- Understand C++ namespaces
- Organize code with namespaces
- Use ROS2 namespace conventions
- Avoid naming conflicts

## What Are Namespaces?

Namespaces prevent naming conflicts and organize code logically.

### Python Comparison

```python
# Python modules provide namespaces
import math
import numpy as np

result1 = math.sqrt(16)    # math's sqrt
result2 = np.sqrt(16)      # numpy's sqrt
```

**C++ namespaces work similarly**

## Basic Namespace Usage

### Defining Namespaces

```cpp
// my_library.hpp
namespace my_library {

class Sensor {
public:
    void read();
};

void process_data();

}  // namespace my_library
```

### Using Namespaces

```cpp
// Option 1: Fully qualified name
my_library::Sensor sensor;
my_library::process_data();

// Option 2: Using declaration
using my_library::Sensor;
Sensor sensor;

// Option 3: Using directive (avoid in headers!)
using namespace my_library;
Sensor sensor;
process_data();
```

## ROS2 Namespaces

ROS2 code uses namespaces extensively:

```cpp
// Common ROS2 namespaces
rclcpp::Node
rclcpp::Publisher
rclcpp::Subscription
std_msgs::msg::String
geometry_msgs::msg::Twist
sensor_msgs::msg::LaserScan
```

### Example ROS2 Node

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace my_robot {
namespace control {

class MotionController : public rclcpp::Node {
public:
    MotionController() : Node("motion_controller") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("cmd", 10);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}  // namespace control
}  // namespace my_robot

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<my_robot::control::MotionController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Nested Namespaces

### C++17 Syntax

```cpp
// Old way
namespace company {
namespace project {
namespace module {

class MyClass {};

}  // namespace module
}  // namespace project
}  // namespace company

// C++17 way (cleaner!)
namespace company::project::module {

class MyClass {};

}  // namespace company::project::module
```

### ROS2 Pattern

```cpp
namespace my_robot::navigation {

class PathPlanner {
    // ...
};

}  // namespace my_robot::navigation
```

## Anonymous Namespaces

For file-local symbols (like `static` in C):

```cpp
// my_file.cpp
namespace {  // Anonymous namespace

void helper_function() {
    // Only visible in this file
}

}  // anonymous namespace

void public_function() {
    helper_function();  // OK
}
```

## Using Declarations

### In Source Files (.cpp)

```cpp
// my_node.cpp
using namespace rclcpp;
using std_msgs::msg::String;

class MyNode : public Node {  // Instead of rclcpp::Node
    // ...
    Publisher<String>::SharedPtr pub_;  // Instead of std_msgs::msg::String
};
```

### In Header Files (.hpp) - Be Careful!

```cpp
// my_class.hpp
// DON'T do this in headers:
// using namespace std;  // Pollutes every file that includes this!

// DO this instead:
namespace my_library {

class MyClass {
public:
    void process(const std::string& data);  // Fully qualified
};

}  // namespace my_library
```

## Common ROS2 Namespace Patterns

### Package Structure

```cpp
namespace package_name {

namespace sensors {
    class Lidar {};
    class Camera {};
}

namespace control {
    class PIDController {};
}

namespace utils {
    double normalize(double value);
}

}  // namespace package_name
```

### Using in ROS2

```cpp
#include "my_package/sensors/lidar.hpp"

// Option 1: Fully qualified
my_package::sensors::Lidar lidar;

// Option 2: Using declaration
using my_package::sensors::Lidar;
Lidar lidar;

// Option 3: Namespace alias
namespace sensors = my_package::sensors;
sensors::Lidar lidar;
```

## Namespace Aliases

```cpp
// Long namespace
namespace very_long_company_name::project::sensors {
    class Lidar {};
}

// Create alias
namespace sensors = very_long_company_name::project::sensors;

// Use alias
sensors::Lidar lidar;
```

## Best Practices

### DO ✓

```cpp
// Use namespace for your package
namespace my_robot_package {
    // Your code
}

// Use nested namespaces for organization
namespace my_robot_package::navigation {
    // Navigation code
}

// Fully qualify in headers
class MyClass {
    std::vector<int> data_;  // Not: vector<int>
};

// Use 'using' in .cpp files
// In my_node.cpp:
using namespace my_robot_package;
```

### DON'T ✗

```cpp
// Don't use 'using namespace' in headers
// my_header.hpp
using namespace std;  // BAD! Affects all includers

// Don't create too-deep nesting
namespace a::b::c::d::e::f {  // TOO DEEP!
    class MyClass {};
}

// Don't use common names at global scope
// my_lib.hpp
class Node {};  // Conflicts with rclcpp::Node!
```

## Standard Namespace

The `std` namespace contains all standard library components:

```cpp
std::vector<int> numbers;
std::string text;
std::cout << "Hello\n";
std::unique_ptr<Data> ptr;
```

## Resolving Conflicts

```cpp
namespace company1 {
    class Logger {};
}

namespace company2 {
    class Logger {};
}

// Explicit selection
company1::Logger logger1;
company2::Logger logger2;

// Or use aliases
namespace c1 = company1;
namespace c2 = company2;

c1::Logger logger1;
c2::Logger logger2;
```

## ROS2 Real-World Example

```cpp
// robot_controller.hpp
#ifndef MY_ROBOT__ROBOT_CONTROLLER_HPP_
#define MY_ROBOT__ROBOT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace my_robot {
namespace control {

class RobotController : public rclcpp::Node {
public:
    RobotController();

private:
    void update_control();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace control
}  // namespace my_robot

#endif  // MY_ROBOT__ROBOT_CONTROLLER_HPP_
```

```cpp
// robot_controller.cpp
#include "my_robot/robot_controller.hpp"

using namespace std::chrono_literals;

namespace my_robot::control {

RobotController::RobotController() : Node("robot_controller") {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(100ms, std::bind(&RobotController::update_control, this));
}

void RobotController::update_control() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.5;
    cmd_pub_->publish(msg);
}

}  // namespace my_robot::control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<my_robot::control::RobotController>());
    rclcpp::shutdown();
    return 0;
}
```

## Summary

**Key Takeaways:**
- Namespaces prevent naming conflicts
- Organize code logically with nested namespaces
- Use fully qualified names in headers
- ROS2 packages should have their own namespace
- `using namespace` is OK in .cpp files, avoid in headers

**Critical for ROS2:**
- Package namespace: `my_package`
- Submodules: `my_package::sensors`
- Follow ROS2 naming conventions
- Avoid conflicts with `rclcpp`, `std_msgs`, etc.

## What's Next?

- **Next Lesson**: [Build Systems](10-build-systems.md)
- **Complete Module 1**: You've learned all C++ fundamentals!

---

**Well done!** You've completed all C++ refresher lessons. Time to practice!
