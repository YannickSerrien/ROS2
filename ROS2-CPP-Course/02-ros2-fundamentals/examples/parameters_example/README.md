# Parameters Example

This package demonstrates ROS2 parameter usage for node configuration and dynamic reconfiguration.

## Overview

This package contains three nodes demonstrating different parameter patterns:
1. **parameter_node** - Basic parameter declaration and retrieval
2. **configurable_robot** - Parameter-based robot configuration with validation
3. **dynamic_parameters** - Runtime parameter updates with callbacks

## Concepts Demonstrated

- Declaring parameters with `declare_parameter<>()`
- Getting parameter values with `get_parameter()`
- Different parameter types (int, double, string, bool, array)
- Loading parameters from YAML files
- Parameter validation
- Dynamic parameter updates with callbacks
- Rejecting invalid parameter changes

## Building

```bash
cd ~/ros2_ws
cp -r path/to/parameters_example src/
colcon build --packages-select parameters_example
source install/setup.bash
```

## Running the Examples

### Example 1: Basic Parameters

**Run with defaults:**
```bash
ros2 run parameters_example parameter_node
```

Output shows all parameter values:
```
[INFO] [parameter_node]: my_integer: 42
[INFO] [parameter_node]: my_double: 3.14
[INFO] [parameter_node]: my_string: Hello, ROS2!
[INFO] [parameter_node]: my_bool: true
[INFO] [parameter_node]: my_array has 5 elements:
[INFO] [parameter_node]:   [0] = 1
[INFO] [parameter_node]:   [1] = 2
...
```

**Override parameters from command line:**
```bash
ros2 run parameters_example parameter_node --ros-args \
  -p my_integer:=100 \
  -p my_double:=2.71 \
  -p my_string:="Custom message" \
  -p my_bool:=false \
  -p my_array:="[10, 20, 30]"
```

**Inspect parameters at runtime:**
```bash
# List all parameters
ros2 param list /parameter_node

# Get specific parameter
ros2 param get /parameter_node my_integer

# Dump all parameters
ros2 param dump /parameter_node
```

---

### Example 2: Configurable Robot

**Run with defaults:**
```bash
ros2 run parameters_example configurable_robot
```

Output:
```
[INFO] [configurable_robot]: === Robot Configuration ===
[INFO] [configurable_robot]: Robot Name: RobotAlpha
[INFO] [configurable_robot]: Max Speed: 1.00 m/s
[INFO] [configurable_robot]: Max Acceleration: 0.50 m/s²
[INFO] [configurable_robot]: Wheel Diameter: 0.100 m
[INFO] [configurable_robot]: Safety Enabled: Yes
[INFO] [configurable_robot]: ==========================
```

**Override from command line:**
```bash
ros2 run parameters_example configurable_robot --ros-args \
  -p robot_name:=Beta \
  -p max_speed:=2.0 \
  -p max_acceleration:=1.0 \
  -p wheel_diameter:=0.15 \
  -p publish_rate:=2.0 \
  -p enable_safety:=false
```

**Load from YAML file:**
```bash
ros2 run parameters_example configurable_robot --ros-args \
  --params-file install/parameters_example/share/parameters_example/config/robot_params.yaml
```

**Test validation (invalid values):**
```bash
# This will trigger validation warnings
ros2 run parameters_example configurable_robot --ros-args \
  -p max_speed:=100.0 \
  -p wheel_diameter:=5.0
```

---

### Example 3: Dynamic Parameters

**Terminal 1 - Start the node:**
```bash
ros2 run parameters_example dynamic_parameters
```

**Terminal 2 - Change parameters at runtime:**

**Change message prefix:**
```bash
ros2 param set /dynamic_parameters message_prefix "New Status"
```

**Change publish rate (timer will update):**
```bash
ros2 param set /dynamic_parameters publish_rate 2.0
```

**Change max count:**
```bash
ros2 param set /dynamic_parameters max_count 20
```

**Try invalid values (will be rejected):**
```bash
# Empty string - rejected
ros2 param set /dynamic_parameters message_prefix ""

# Negative rate - rejected
ros2 param set /dynamic_parameters publish_rate -1.0

# Zero max_count - rejected
ros2 param set /dynamic_parameters max_count 0
```

**Monitor parameter changes:**
```bash
# Watch parameter updates
ros2 param list /dynamic_parameters

# Get current value
ros2 param get /dynamic_parameters publish_rate
```

---

## Code Structure

### parameter_node.cpp
```
ParameterNode
├── declare_parameter() calls  (Initialize parameters)
└── log_parameters()          (Display all values)
```

### configurable_robot.cpp
```
ConfigurableRobotNode
├── declare_parameter() calls  (Robot config)
├── validate_parameters()      (Input validation)
├── log_configuration()        (Display config)
├── publish_status()           (Timer callback)
└── publisher_, timer_         (ROS objects)
```

### dynamic_parameters.cpp
```
DynamicParametersNode
├── declare_parameter() calls     (Initial setup)
├── parameters_callback()         (Handle parameter changes)
├── update_timer()                (Recreate timer with new rate)
├── timer_callback()              (Publishing)
└── param_callback_handle_        (Callback registration)
```

---

## Key Concepts

### Declaring Parameters
```cpp
// Basic declaration with default
this->declare_parameter("param_name", default_value);

// Different types
this->declare_parameter("my_int", 42);
this->declare_parameter("my_double", 3.14);
this->declare_parameter("my_string", "hello");
this->declare_parameter("my_bool", true);
this->declare_parameter("my_array", std::vector<int64_t>{1, 2, 3});
```

### Getting Parameters
```cpp
// Get parameter value
int value = this->get_parameter("param_name").as_int();
double d = this->get_parameter("param_name").as_double();
std::string s = this->get_parameter("param_name").as_string();
bool b = this->get_parameter("param_name").as_bool();
std::vector<int64_t> arr = this->get_parameter("param_name").as_integer_array();
```

### Parameter Callbacks
```cpp
// Register callback for parameter changes
param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MyNode::parameters_callback, this, std::placeholders::_1));

// Callback function
rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
        if (param.get_name() == "my_param") {
            // Validate and apply
            if (validate(param.as_double())) {
                my_value_ = param.as_double();
            } else {
                result.successful = false;
                result.reason = "Invalid value";
            }
        }
    }

    return result;
}
```

### YAML Configuration File
```yaml
node_name:
  ros__parameters:
    param1: value1
    param2: 3.14
    param3: true
```

Load with:
```bash
ros2 run package node --ros-args --params-file config/params.yaml
```

---

## Command-Line Parameter Usage

**Single parameter:**
```bash
ros2 run package node --ros-args -p param:=value
```

**Multiple parameters:**
```bash
ros2 run package node --ros-args \
  -p param1:=value1 \
  -p param2:=value2 \
  -p param3:=value3
```

**From YAML file:**
```bash
ros2 run package node --ros-args --params-file path/to/params.yaml
```

**Mix CLI and YAML:**
```bash
ros2 run package node --ros-args \
  --params-file params.yaml \
  -p override_param:=value
```

---

## Runtime Parameter Commands

```bash
# List all parameters for a node
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name param_name

# Set parameter value (if node supports dynamic updates)
ros2 param set /node_name param_name value

# Dump all parameters to screen
ros2 param dump /node_name

# Dump to file
ros2 param dump /node_name > params_backup.yaml

# Load parameters from file
ros2 param load /node_name params.yaml

# Describe parameter
ros2 param describe /node_name param_name
```

---

## Troubleshooting

**Issue: "Parameter not declared"**
- Make sure you call `declare_parameter()` before `get_parameter()`
- Check parameter name spelling

**Issue: "Wrong parameter type"**
```bash
# Check parameter type
ros2 param describe /node_name param_name

# Use correct accessor
as_int(), as_double(), as_string(), as_bool(), as_integer_array()
```

**Issue: "Cannot set parameter at runtime"**
- Node may not have parameter callback registered
- Parameter might be read-only (only read at startup)
- Check if callback is rejecting the value

**Issue: "YAML file not found"**
```bash
# Use absolute path
ros2 run package node --ros-args --params-file /full/path/to/params.yaml

# Or relative to current directory
ros2 run package node --ros-args --params-file ./config/params.yaml
```

---

## Learning Exercises

1. **Add new parameter**: Add a `robot_id` int parameter to configurable_robot
2. **Array parameter**: Add a `sensor_positions` array parameter
3. **Nested config**: Create a YAML file with multiple nodes' parameters
4. **Parameter ranges**: Add range checking (min/max) to dynamic_parameters
5. **Conditional behavior**: Make robot behavior change based on parameters
6. **Save parameters**: Add a service to save current parameters to YAML

---

## Related Lessons

- [Lesson 9: Parameters](../../lessons/09-parameters.md)
- [Lesson 10: Logging](../../lessons/10-logging.md)

---

## Next Steps

After mastering this example:
1. Try the **timer_logging_example** for periodic tasks
2. Combine parameters with services for runtime reconfiguration
3. Create launch files that set multiple parameters
4. Build a parameter server node that manages configuration for multiple nodes
