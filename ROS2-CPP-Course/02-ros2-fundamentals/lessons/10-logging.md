# Lesson 10: Logging

## Learning Objectives

- Master ROS2 logging macros
- Understand logging severity levels
- Use conditional and throttled logging
- Configure logging output
- Apply logging best practices
- Debug with logs effectively

## Why Logging Matters

Good logging is essential for:
- **Debugging**: Track down issues
- **Monitoring**: Watch system health
- **Auditing**: Record events
- **Performance**: Identify bottlenecks
- **Production**: Diagnose field issues

❌ **Don't use cout:**
```cpp
std::cout << "Message" << std::endl;  // NO!
```

✅ **Use ROS2 logging:**
```cpp
RCLCPP_INFO(logger, "Message");  // YES!
```

## Logging Severity Levels

ROS2 provides **5 severity levels**:

| Level | Macro | Purpose | Color |
|-------|-------|---------|-------|
| **DEBUG** | `RCLCPP_DEBUG` | Detailed debugging info | Gray |
| **INFO** | `RCLCPP_INFO` | General information | White |
| **WARN** | `RCLCPP_WARN` | Warning messages | Yellow |
| **ERROR** | `RCLCPP_ERROR` | Error messages | Red |
| **FATAL** | `RCLCPP_FATAL` | Critical errors | Red/Bold |

### Basic Usage

```cpp
#include <rclcpp/rclcpp.hpp>

class LoggingNode : public rclcpp::Node {
public:
    LoggingNode() : Node("logging_node") {
        // Get logger
        auto logger = this->get_logger();

        // Different severity levels
        RCLCPP_DEBUG(logger, "Detailed debug information");
        RCLCPP_INFO(logger, "General information message");
        RCLCPP_WARN(logger, "Warning: something might be wrong");
        RCLCPP_ERROR(logger, "Error: something went wrong");
        RCLCPP_FATAL(logger, "Fatal: critical failure!");
    }
};
```

**Output:**
```
[DEBUG] [logging_node]: Detailed debug information
[INFO]  [logging_node]: General information message
[WARN]  [logging_node]: Warning: something might be wrong
[ERROR] [logging_node]: Error: something went wrong
[FATAL] [logging_node]: Fatal: critical failure!
```

## Formatted Logging

### Printf-Style Formatting

```cpp
int count = 42;
double speed = 1.5;
std::string robot = "alpha";

RCLCPP_INFO(logger, "Count: %d", count);
RCLCPP_INFO(logger, "Speed: %.2f m/s", speed);
RCLCPP_INFO(logger, "Robot: %s", robot.c_str());

// Multiple values
RCLCPP_INFO(logger, "Robot %s: count=%d, speed=%.2f",
            robot.c_str(), count, speed);
```

### Format Specifiers

```cpp
// Integers
RCLCPP_INFO(logger, "Int: %d", 42);
RCLCPP_INFO(logger, "Long: %ld", 1234567890L);
RCLCPP_INFO(logger, "Hex: 0x%x", 255);

// Floats
RCLCPP_INFO(logger, "Float: %f", 3.14159);
RCLCPP_INFO(logger, "Fixed: %.2f", 3.14159);  // 3.14
RCLCPP_INFO(logger, "Scientific: %e", 1000.0);  // 1.000000e+03

// Strings
RCLCPP_INFO(logger, "String: %s", "hello");
RCLCPP_INFO(logger, "C++ string: %s", std::string("world").c_str());

// Booleans
bool enabled = true;
RCLCPP_INFO(logger, "Enabled: %s", enabled ? "true" : "false");
```

## Conditional Logging

### ONCE - Log Only Once

```cpp
void periodic_function() {
    RCLCPP_INFO_ONCE(logger, "This prints only the first time");

    // Called 100 times, but logs only once
}
```

**Output (called 5 times):**
```
[INFO] [node]: This prints only the first time
```

### EXPRESSION - Conditional Logging

```cpp
int count = 0;

void loop() {
    count++;

    // Log only when expression is true
    RCLCPP_INFO_EXPRESSION(logger, count % 10 == 0,
                           "Count is now: %d", count);
}
```

**Output:**
```
[INFO] [node]: Count is now: 10
[INFO] [node]: Count is now: 20
[INFO] [node]: Count is now: 30
```

### SKIPFIRST - Skip First Call

```cpp
void function() {
    RCLCPP_INFO_SKIPFIRST(logger, "This message is skipped the first time");
}
```

## Throttled Logging

### THROTTLE - Rate Limiting

```cpp
void high_frequency_callback() {
    // Log at most once per second (1000ms)
    RCLCPP_INFO_THROTTLE(logger, *get_clock(), 1000,
                         "High frequency callback running");
}
```

**Use cases:**
- Sensor data processing
- High-frequency callbacks
- Prevent log spam

### Example: Temperature Monitoring

```cpp
class TempMonitor : public rclcpp::Node {
public:
    TempMonitor() : Node("temp_monitor") {
        sub_ = create_subscription<std_msgs::msg::Float64>(
            "temperature", 10,
            std::bind(&TempMonitor::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Log every message at DEBUG level
        RCLCPP_DEBUG(this->get_logger(), "Temp: %.2f", msg->data);

        // Log INFO at most once per second
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
                             "Temperature: %.2f°C", msg->data);

        // Log WARN only when hot
        RCLCPP_WARN_EXPRESSION(this->get_logger(), msg->data > 80.0,
                               "HIGH TEMPERATURE: %.2f°C", msg->data);

        // Log ERROR only when critical
        RCLCPP_ERROR_EXPRESSION(this->get_logger(), msg->data > 100.0,
                                "CRITICAL TEMPERATURE: %.2f°C", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
};
```

## Stream-Style Logging

### Using << Operator

```cpp
#include <rclcpp/logging.hpp>

// Stream style (C++ style)
RCLCPP_INFO_STREAM(logger, "Count: " << count << ", Speed: " << speed);

// Equivalent to:
RCLCPP_INFO(logger, "Count: %d, Speed: %.2f", count, speed);
```

**When to use:**
- Complex string formatting
- C++ types without .c_str()
- Mixing types

```cpp
std::string robot_name = "alpha";
int id = 42;
double speed = 1.5;

RCLCPP_INFO_STREAM(logger,
    "Robot '" << robot_name << "' (ID: " << id << ") moving at " << speed << " m/s"
);
```

## Complete Logging Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>

using namespace std::chrono_literals;

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller"), count_(0), speed_(0.0) {
        RCLCPP_INFO(this->get_logger(), "=== Robot Controller Started ===");

        // Declare parameters
        this->declare_parameter("max_speed", 2.0);
        this->declare_parameter("robot_id", 1);

        max_speed_ = this->get_parameter("max_speed").as_double();
        robot_id_ = this->get_parameter("robot_id").as_int();

        RCLCPP_INFO(this->get_logger(), "Configuration:");
        RCLCPP_INFO(this->get_logger(), "  Robot ID: %d", robot_id_);
        RCLCPP_INFO(this->get_logger(), "  Max Speed: %.2f m/s", max_speed_);

        // Create timer
        timer_ = create_wall_timer(
            100ms,
            std::bind(&RobotController::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Controller ready");
    }

private:
    void control_loop() {
        count_++;

        // DEBUG: Every iteration
        RCLCPP_DEBUG(this->get_logger(), "Control loop iteration %d", count_);

        // INFO: Throttled to once per second
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
                             "Status - Count: %d, Speed: %.2f m/s",
                             count_, speed_);

        // Simulate speed change
        speed_ = (count_ % 100) * 0.01;

        // WARN: Only when approaching limit
        if (speed_ > max_speed_ * 0.9) {
            RCLCPP_WARN(this->get_logger(),
                        "Speed %.2f approaching limit %.2f",
                        speed_, max_speed_);
        }

        // ERROR: Only when exceeding limit
        if (speed_ > max_speed_) {
            RCLCPP_ERROR(this->get_logger(),
                         "SPEED EXCEEDED: %.2f > %.2f - EMERGENCY STOP",
                         speed_, max_speed_);
            emergency_stop();
        }

        // Log milestone once
        RCLCPP_INFO_EXPRESSION(this->get_logger(), count_ == 100,
                               "Milestone: 100 iterations completed");
    }

    void emergency_stop() {
        speed_ = 0.0;
        RCLCPP_FATAL(this->get_logger(), "Emergency stop activated!");
        rclcpp::shutdown();
    }

    int count_;
    double speed_;
    double max_speed_;
    int robot_id_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}
```

## Logging Configuration

### Set Log Level at Runtime

```bash
# Set log level for specific node
ros2 run pkg node --ros-args --log-level debug
ros2 run pkg node --ros-args --log-level info
ros2 run pkg node --ros-args --log-level warn

# Set for all nodes
ros2 run pkg node --ros-args --log-level-all debug
```

### Log Levels

```
FATAL > ERROR > WARN > INFO > DEBUG
```

- Setting to INFO shows: INFO, WARN, ERROR, FATAL
- Setting to DEBUG shows: all messages
- Setting to ERROR shows: only ERROR and FATAL

### Named Loggers

```cpp
class MultiLogger : public rclcpp::Node {
public:
    MultiLogger() : Node("multi_logger") {
        // Get named sub-loggers
        auto sensor_logger = rclcpp::get_logger("multi_logger.sensors");
        auto control_logger = rclcpp::get_logger("multi_logger.control");

        RCLCPP_INFO(sensor_logger, "Sensor initialized");
        RCLCPP_INFO(control_logger, "Controller initialized");
    }
};
```

**Configure:**
```bash
ros2 run pkg node --ros-args \
    --log-level multi_logger.sensors:=debug \
    --log-level multi_logger.control:=warn
```

## Logging Output Format

### Default Format

```
[severity] [timestamp] [node_name]: message
```

### Custom Format

Set environment variable:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

# Options:
# {severity} - Log level
# {name} - Logger name
# {message} - Log message
# {time} - Timestamp
# {file_name} - Source file
# {line_number} - Line number
# {function_name} - Function name
```

**Examples:**
```bash
# Minimal
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"

# Detailed
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}] [{severity}] [{name}] {file_name}:{line_number} - {message}"

# Production
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{time}] [{name}] {message}"
```

## Best Practices

### DO ✓

```cpp
// Use appropriate levels
RCLCPP_DEBUG(logger, "Entering function with value: %d", x);
RCLCPP_INFO(logger, "Node initialized successfully");
RCLCPP_WARN(logger, "Sensor reading unstable");
RCLCPP_ERROR(logger, "Failed to connect to device");
RCLCPP_FATAL(logger, "Critical system failure");

// Use throttle for high-frequency logs
RCLCPP_INFO_THROTTLE(logger, *get_clock(), 1000, "Status update");

// Use expression for conditional logs
RCLCPP_WARN_EXPRESSION(logger, value > threshold, "Value too high");

// Provide context
RCLCPP_ERROR(logger, "Failed to open file '%s': %s",
             filename.c_str(), strerror(errno));

// Log state transitions
RCLCPP_INFO(logger, "State changed: %s -> %s", old_state.c_str(), new_state.c_str());
```

### DON'T ✗

```cpp
// Don't use cout/cerr
std::cout << "Message" << std::endl;  // NO!

// Don't log in tight loops without throttle
for (int i = 0; i < 10000; ++i) {
    RCLCPP_INFO(logger, "Loop %d", i);  // Spam!
}

// Don't log sensitive data
RCLCPP_INFO(logger, "Password: %s", password.c_str());  // Security!

// Don't use wrong severity
RCLCPP_ERROR(logger, "Node started");  // Should be INFO
RCLCPP_INFO(logger, "Critical failure");  // Should be ERROR/FATAL

// Don't log without context
RCLCPP_ERROR(logger, "Failed");  // Failed at what?
```

### Choosing Severity Levels

**DEBUG:**
- Function entry/exit
- Variable values
- Detailed algorithm steps
- Development debugging

**INFO:**
- Startup/shutdown messages
- Configuration values
- State changes
- Successful operations

**WARN:**
- Recoverable errors
- Deprecated usage
- Performance issues
- Unusual but handled conditions

**ERROR:**
- Operation failures
- Invalid inputs
- Resource unavailable
- Exceptions caught

**FATAL:**
- Critical system failures
- Unrecoverable errors
- Safety violations
- Immediate shutdown needed

## Logging Patterns

### Pattern 1: Entry/Exit Logging

```cpp
void process_data(const Data& data) {
    RCLCPP_DEBUG(logger, "Entering process_data() with %zu items", data.size());

    // Process...

    RCLCPP_DEBUG(logger, "Exiting process_data() successfully");
}
```

### Pattern 2: State Machine Logging

```cpp
void set_state(State new_state) {
    RCLCPP_INFO(logger, "State transition: %s -> %s",
                state_to_string(current_state_).c_str(),
                state_to_string(new_state).c_str());
    current_state_ = new_state;
}
```

### Pattern 3: Error Handling

```cpp
bool connect_to_device() {
    RCLCPP_INFO(logger, "Attempting to connect to device...");

    if (!device_->connect()) {
        RCLCPP_ERROR(logger, "Failed to connect to device at %s",
                     device_address_.c_str());
        return false;
    }

    RCLCPP_INFO(logger, "Successfully connected to device");
    return true;
}
```

### Pattern 4: Periodic Status

```cpp
void status_callback() {
    RCLCPP_INFO_THROTTLE(logger, *get_clock(), 5000,
        "Status: pos=(%.2f, %.2f), vel=%.2f, battery=%.1f%%",
        pos_x_, pos_y_, velocity_, battery_level_
    );
}
```

## Debugging with Logs

### Enable Debug Logging

```bash
# For development
ros2 run pkg node --ros-args --log-level debug

# Produce detailed logs
```

### Structured Logging for Debugging

```cpp
void debug_complex_operation(const Data& input) {
    RCLCPP_DEBUG(logger, "=== Complex Operation Debug ===");
    RCLCPP_DEBUG(logger, "Input size: %zu", input.size());
    RCLCPP_DEBUG(logger, "Input range: [%.2f, %.2f]", input.min(), input.max());

    auto result = process(input);

    RCLCPP_DEBUG(logger, "Output size: %zu", result.size());
    RCLCPP_DEBUG(logger, "Processing time: %.2f ms", elapsed_ms);
    RCLCPP_DEBUG(logger, "=== End Debug ===");
}
```

## Summary

**Key Takeaways:**
- Use ROS2 logging, not cout/cerr
- Choose appropriate severity levels
- Use THROTTLE for high-frequency logs
- Use EXPRESSION for conditional logs
- Provide context in error messages
- Configure log levels at runtime
- Debug with DEBUG level, run with INFO

**Severity Levels:**
```cpp
RCLCPP_DEBUG(logger, "Detailed info");
RCLCPP_INFO(logger, "General info");
RCLCPP_WARN(logger, "Warning");
RCLCPP_ERROR(logger, "Error");
RCLCPP_FATAL(logger, "Critical!");
```

**Conditional Macros:**
```cpp
RCLCPP_INFO_ONCE(logger, "Once");
RCLCPP_INFO_EXPRESSION(logger, condition, "Message");
RCLCPP_INFO_THROTTLE(logger, *get_clock(), ms, "Message");
```

## Practice Exercise

Create a node that:
1. Logs startup with INFO
2. Logs every callback with DEBUG
3. Logs status once per second with THROTTLE
4. Logs warnings when value > 80
5. Logs errors when value > 100
6. Test with different log levels

## What's Next?

- **Next Lesson**: [Timers](11-timers.md) - Periodic callbacks
- **Related**: [Parameters](09-parameters.md) - Configuration
- **Advanced**: [Executors](12-executors.md) - Callback execution

---

**You can now log professionally in ROS2!** Next: mastering timers for periodic tasks.
