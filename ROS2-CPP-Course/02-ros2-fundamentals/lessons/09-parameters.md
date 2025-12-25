# Lesson 9: Parameters

## Learning Objectives

- Understand ROS2 parameters
- Declare and use parameters in nodes
- Set parameters from command line and YAML files
- Implement parameter callbacks
- Validate parameter values
- Use parameters for node configuration
- Apply parameter best practices

## What are Parameters?

**Parameters** are configuration values for nodes that can be:
- Set at runtime
- Changed dynamically
- Queried by other nodes
- Saved in configuration files

```
Node: "I need a configurable update_rate"
      ↓
Parameter: update_rate = 10.0 Hz
           ↓
Can be changed without recompiling!
```

### Why Use Parameters?

✓ **Instead of hard-coding:**
```cpp
// Bad: Hard-coded
double rate = 10.0;

// Good: Parameter
double rate = this->get_parameter("update_rate").as_double();
```

✓ **Benefits:**
- Change behavior without recompiling
- Different configurations for different robots
- Easy tuning during development
- Runtime reconfiguration

## Parameter Types

ROS2 supports these parameter types:

| Type | C++ Type | Example |
|------|----------|---------|
| `bool` | `bool` | `true`, `false` |
| `int` | `int64_t` | `42`, `-10` |
| `double` | `double` | `3.14`, `-0.5` |
| `string` | `std::string` | `"hello"`, `"mode"` |
| `byte_array` | `std::vector<uint8_t>` | `[0x01, 0x02]` |
| `bool_array` | `std::vector<bool>` | `[true, false]` |
| `integer_array` | `std::vector<int64_t>` | `[1, 2, 3]` |
| `double_array` | `std::vector<double>` | `[1.0, 2.5]` |
| `string_array` | `std::vector<std::string>` | `["a", "b"]` |

## Your First Parameter

### Basic Example

```cpp
#include <rclcpp/rclcpp.hpp>

class ParameterNode : public rclcpp::Node {
public:
    ParameterNode() : Node("parameter_node") {
        // Declare parameter with default value
        this->declare_parameter("update_rate", 10.0);

        // Get parameter value
        double rate = this->get_parameter("update_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "Update rate: %.2f Hz", rate);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParameterNode>());
    rclcpp::shutdown();
    return 0;
}
```

**Run with default:**
```bash
ros2 run my_package parameter_node
# Output: [INFO] [parameter_node]: Update rate: 10.00 Hz
```

**Run with custom value:**
```bash
ros2 run my_package parameter_node --ros-args -p update_rate:=20.0
# Output: [INFO] [parameter_node]: Update rate: 20.00 Hz
```

## Declaring Parameters

### Basic Declaration

```cpp
// Declare with default value
this->declare_parameter("param_name", default_value);

// Different types
this->declare_parameter("max_speed", 1.5);          // double
this->declare_parameter("robot_name", "robot1");    // string
this->declare_parameter("enabled", true);           // bool
this->declare_parameter("retry_count", 5);          // int
```

### Declaration with Description

```cpp
#include <rclcpp/rclcpp.hpp>

this->declare_parameter("max_speed", 1.0);

// Add description (for documentation)
auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
descriptor.description = "Maximum robot speed in m/s";

this->declare_parameter("max_speed", 1.0, descriptor);
```

### With Value Constraints

```cpp
auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
descriptor.description = "Update rate in Hz";

// Set range constraints
descriptor.floating_point_range.resize(1);
descriptor.floating_point_range[0].from_value = 1.0;
descriptor.floating_point_range[0].to_value = 100.0;

this->declare_parameter("update_rate", 10.0, descriptor);
```

## Getting Parameter Values

### Get Single Parameter

```cpp
// Method 1: get_parameter()
double rate = this->get_parameter("update_rate").as_double();
std::string name = this->get_parameter("robot_name").as_string();
bool enabled = this->get_parameter("enabled").as_bool();
int count = this->get_parameter("retry_count").as_int();

// Method 2: get_parameter_or()
double rate = this->get_parameter("update_rate").get_parameter_value().get<double>();
```

### Safe Parameter Access

```cpp
// Check if parameter exists
if (this->has_parameter("max_speed")) {
    double speed = this->get_parameter("max_speed").as_double();
} else {
    RCLCPP_WARN(this->get_logger(), "Parameter max_speed not declared");
}

// Get with fallback
rclcpp::Parameter param;
if (this->get_parameter("max_speed", param)) {
    double speed = param.as_double();
} else {
    double speed = 1.0;  // Default
}
```

## Setting Parameters at Runtime

### From Command Line

```bash
# Single parameter
ros2 run pkg node --ros-args -p param_name:=value

# Multiple parameters
ros2 run pkg node --ros-args -p param1:=value1 -p param2:=value2

# Examples
ros2 run pkg node --ros-args -p max_speed:=2.0
ros2 run pkg node --ros-args -p robot_name:=robot1 -p enabled:=true
```

### Using ros2 param Command

```bash
# List parameters
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name param_name

# Set parameter value
ros2 param set /node_name param_name value

# Dump all parameters to file
ros2 param dump /node_name > params.yaml

# Load parameters from file
ros2 param load /node_name params.yaml
```

## Complete Parameter Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class ConfigurableNode : public rclcpp::Node {
public:
    ConfigurableNode() : Node("configurable_node"), count_(0) {
        // Declare parameters with defaults
        this->declare_parameter("update_rate", 1.0);
        this->declare_parameter("robot_name", "default_robot");
        this->declare_parameter("max_count", 100);
        this->declare_parameter("enabled", true);

        // Get parameter values
        double rate = this->get_parameter("update_rate").as_double();
        robot_name_ = this->get_parameter("robot_name").as_string();
        max_count_ = this->get_parameter("max_count").as_int();
        enabled_ = this->get_parameter("enabled").as_bool();

        // Log configuration
        RCLCPP_INFO(this->get_logger(), "Configuration:");
        RCLCPP_INFO(this->get_logger(), "  Robot name: %s", robot_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Update rate: %.2f Hz", rate);
        RCLCPP_INFO(this->get_logger(), "  Max count: %d", max_count_);
        RCLCPP_INFO(this->get_logger(), "  Enabled: %s", enabled_ ? "true" : "false");

        // Create timer based on parameter
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = this->create_wall_timer(
            period,
            std::bind(&ConfigurableNode::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        if (!enabled_) {
            return;
        }

        count_++;
        RCLCPP_INFO(this->get_logger(), "[%s] Count: %d / %d",
                    robot_name_.c_str(), count_, max_count_);

        if (count_ >= max_count_) {
            RCLCPP_INFO(this->get_logger(), "Max count reached!");
            rclcpp::shutdown();
        }
    }

    std::string robot_name_;
    int max_count_;
    bool enabled_;
    int count_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConfigurableNode>());
    rclcpp::shutdown();
    return 0;
}
```

**Run with custom configuration:**
```bash
ros2 run my_package configurable_node --ros-args \
    -p robot_name:=robot_alpha \
    -p update_rate:=2.0 \
    -p max_count:=10 \
    -p enabled:=true
```

## YAML Parameter Files

### Create Parameter File

Create `config/params.yaml`:

```yaml
/**:
  ros__parameters:
    update_rate: 5.0
    robot_name: "production_robot"
    max_count: 50
    enabled: true
```

### Load from File

```bash
ros2 run my_package configurable_node --ros-args --params-file config/params.yaml
```

### Node-Specific Parameters

```yaml
# For specific node
/configurable_node:
  ros__parameters:
    update_rate: 5.0
    robot_name: "robot1"

# For all nodes (use /**)
/**:
  ros__parameters:
    global_param: value
```

## Parameter Callbacks

### Dynamic Parameter Updates

```cpp
class DynamicNode : public rclcpp::Node {
public:
    DynamicNode() : Node("dynamic_node"), max_speed_(1.0) {
        // Declare parameter
        this->declare_parameter("max_speed", 1.0);
        max_speed_ = this->get_parameter("max_speed").as_double();

        // Register callback for parameter changes
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DynamicNode::parameters_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Node ready. Max speed: %.2f", max_speed_);
    }

private:
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name() == "max_speed") {
                // Validate
                double new_speed = param.as_double();
                if (new_speed < 0.0 || new_speed > 5.0) {
                    result.successful = false;
                    result.reason = "max_speed must be between 0.0 and 5.0";
                    RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                    return result;
                }

                // Accept change
                max_speed_ = new_speed;
                RCLCPP_INFO(this->get_logger(), "Max speed updated to: %.2f", max_speed_);
            }
        }

        return result;
    }

    double max_speed_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
```

**Test dynamic update:**
```bash
# Terminal 1
ros2 run my_package dynamic_node

# Terminal 2
ros2 param set /dynamic_node max_speed 2.5
# Output: Successfully set parameter

ros2 param set /dynamic_node max_speed 10.0
# Output: Setting parameter failed: max_speed must be between 0.0 and 5.0
```

## Array Parameters

### Declare and Use Arrays

```cpp
class ArrayParamNode : public rclcpp::Node {
public:
    ArrayParamNode() : Node("array_param_node") {
        // Declare array parameters
        this->declare_parameter("waypoints_x", std::vector<double>{0.0, 1.0, 2.0});
        this->declare_parameter("waypoints_y", std::vector<double>{0.0, 1.0, 0.0});
        this->declare_parameter("robot_names", std::vector<std::string>{"r1", "r2"});

        // Get arrays
        auto x_coords = this->get_parameter("waypoints_x").as_double_array();
        auto y_coords = this->get_parameter("waypoints_y").as_double_array();
        auto names = this->get_parameter("robot_names").as_string_array();

        // Use arrays
        RCLCPP_INFO(this->get_logger(), "Waypoints:");
        for (size_t i = 0; i < x_coords.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  (%f, %f)", x_coords[i], y_coords[i]);
        }

        RCLCPP_INFO(this->get_logger(), "Robots: %s", names[0].c_str());
    }
};
```

**YAML for arrays:**
```yaml
/**:
  ros__parameters:
    waypoints_x: [0.0, 1.0, 2.0, 3.0]
    waypoints_y: [0.0, 1.0, 0.0, 1.0]
    robot_names: ["alpha", "beta", "gamma"]
```

## Parameter Validation

### Range Validation

```cpp
rcl_interfaces::msg::SetParametersResult validate_parameters(
    const std::vector<rclcpp::Parameter>& params)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto& param : params) {
        if (param.get_name() == "max_speed") {
            double value = param.as_double();
            if (value < 0.0 || value > 10.0) {
                result.successful = false;
                result.reason = "max_speed out of range [0.0, 10.0]";
                return result;
            }
        }

        if (param.get_name() == "robot_id") {
            int value = param.as_int();
            if (value < 1 || value > 100) {
                result.successful = false;
                result.reason = "robot_id must be 1-100";
                return result;
            }
        }
    }

    return result;
}
```

### Type Validation

```cpp
if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    result.successful = false;
    result.reason = "Expected double parameter";
    return result;
}
```

## Common Parameter Patterns

### Pattern 1: Configuration at Startup

```cpp
class StartupConfigNode : public rclcpp::Node {
public:
    StartupConfigNode() : Node("startup_config") {
        // All configuration via parameters
        this->declare_parameter("device_name", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("timeout_ms", 1000);

        // Get and use
        device_ = this->get_parameter("device_name").as_string();
        baud_ = this->get_parameter("baud_rate").as_int();
        timeout_ = this->get_parameter("timeout_ms").as_int();

        // Initialize with config
        initialize_device(device_, baud_, timeout_);
    }
};
```

### Pattern 2: Runtime Reconfiguration

```cpp
class ReconfigurableNode : public rclcpp::Node {
public:
    ReconfigurableNode() : Node("reconfigurable") {
        this->declare_parameter("gain", 1.0);
        gain_ = this->get_parameter("gain").as_double();

        // Callback for updates
        param_callback_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for (const auto& p : params) {
                    if (p.get_name() == "gain") {
                        gain_ = p.as_double();
                        RCLCPP_INFO(this->get_logger(), "Gain updated: %.2f", gain_);
                    }
                }
                auto result = rcl_interfaces::msg::SetParametersResult();
                result.successful = true;
                return result;
            }
        );
    }

private:
    double gain_;
};
```

### Pattern 3: Namespace Parameters

```cpp
class NamespacedNode : public rclcpp::Node {
public:
    NamespacedNode() : Node("namespaced") {
        // Parameters under namespaces
        this->declare_parameter("controller.kp", 1.0);
        this->declare_parameter("controller.ki", 0.1);
        this->declare_parameter("controller.kd", 0.05);

        kp_ = this->get_parameter("controller.kp").as_double();
        ki_ = this->get_parameter("controller.ki").as_double();
        kd_ = this->get_parameter("controller.kd").as_double();
    }
};
```

**YAML:**
```yaml
/**:
  ros__parameters:
    controller:
      kp: 2.0
      ki: 0.5
      kd: 0.1
```

## Best Practices

### DO ✓

```cpp
// Declare all parameters in constructor
MyNode() {
    declare_parameter("param1", default1);
    declare_parameter("param2", default2);
}

// Provide defaults
declare_parameter("timeout", 5.0);  // Good default

// Validate in callbacks
if (value < min || value > max) {
    result.successful = false;
    return result;
}

// Use descriptive names
declare_parameter("max_linear_velocity", 1.0);  // Clear

// Add descriptions
descriptor.description = "Maximum velocity in m/s";
declare_parameter("max_vel", 1.0, descriptor);
```

### DON'T ✗

```cpp
// Don't use magic numbers
double speed = 1.5;  // Where does 1.5 come from?

// Don't skip validation
max_speed_ = param.as_double();  // What if negative?

// Don't use generic names
declare_parameter("value", 10);  // Bad name

// Don't access undeclared parameters
auto val = get_parameter("unknown").as_double();  // Will fail!
```

## Summary

**Key Takeaways:**
- Parameters configure nodes without recompiling
- Declare with `declare_parameter(name, default)`
- Get with `get_parameter(name).as_type()`
- Set from CLI: `-p name:=value`
- Set from file: `--params-file config.yaml`
- Use callbacks for dynamic updates
- Always validate parameter values

**Critical Patterns:**

**Declaration:**
```cpp
this->declare_parameter("param_name", default_value);
```

**Getting:**
```cpp
auto value = this->get_parameter("param_name").as_double();
```

**Callback:**
```cpp
add_on_set_parameters_callback([](auto params) {
    // Validate and update
    return result;
});
```

## Practice Exercise

Create a configurable node with:
1. Parameters: `update_rate` (double), `robot_id` (int), `enabled` (bool)
2. Set parameters from command line
3. Create a YAML file with different values
4. Implement parameter validation
5. Add a callback to detect parameter changes

## What's Next?

- **Next Lesson**: [Logging](10-logging.md) - Advanced logging techniques
- **Related**: [Service Servers](07-service-servers.md) - Alternative to param services
- **See Also**: [Launch Files](../03-ros2-intermediate/06-launch-files.md) - Pass params in launch

---

**You can now configure nodes dynamically!** Next: professional logging practices.
