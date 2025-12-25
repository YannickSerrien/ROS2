# Lesson 11: Timers

## Learning Objectives

- Create and use ROS2 timers
- Understand wall timers vs ROS timers
- Implement periodic callbacks
- Manage timer lifecycle
- Use one-shot timers
- Handle timer cancellation
- Apply timer best practices

## What are Timers?

**Timers** trigger callbacks at regular intervals:

```
Timer (1 second) → callback → wait → callback → wait → callback...
```

### Why Use Timers?

✓ **Periodic operations:**
- Sensor polling
- Status publishing
- Heartbeat messages
- Periodic computations
- Watchdog monitoring

## Wall Timer

The most common timer type - based on **wall clock** time (real-world time).

### Basic Wall Timer

```cpp
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TimerNode : public rclcpp::Node {
public:
    TimerNode() : Node("timer_node"), count_(0) {
        // Create timer (1 second period)
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&TimerNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Timer started (1 second)");
    }

private:
    void timer_callback() {
        count_++;
        RCLCPP_INFO(this->get_logger(), "Timer callback #%d", count_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimerNode>());
    rclcpp::shutdown();
    return 0;
}
```

**Output:**
```
[INFO] [timer_node]: Timer started (1 second)
[INFO] [timer_node]: Timer callback #1
[INFO] [timer_node]: Timer callback #2
[INFO] [timer_node]: Timer callback #3
...
```

### Timer Periods

Using `std::chrono_literals`:

```cpp
using namespace std::chrono_literals;

// Seconds
timer_ = create_wall_timer(1s, callback);    // 1 second
timer_ = create_wall_timer(5s, callback);    // 5 seconds

// Milliseconds
timer_ = create_wall_timer(100ms, callback);  // 100 milliseconds
timer_ = create_wall_timer(500ms, callback);  // 500 milliseconds

// Minutes (less common)
timer_ = create_wall_timer(1min, callback);   // 1 minute
```

### Using std::chrono

```cpp
#include <chrono>

// From milliseconds
auto period = std::chrono::milliseconds(500);
timer_ = create_wall_timer(period, callback);

// From duration
auto period = std::chrono::duration<double>(0.1);  // 0.1 seconds
timer_ = create_wall_timer(period, callback);

// From parameter
double rate_hz = 10.0;
auto period = std::chrono::duration<double>(1.0 / rate_hz);
timer_ = create_wall_timer(period, callback);
```

## Timer with Lambda

### Inline Lambda

```cpp
class LambdaTimerNode : public rclcpp::Node {
public:
    LambdaTimerNode() : Node("lambda_timer"), count_(0) {
        // Timer with lambda callback
        timer_ = create_wall_timer(
            1s,
            [this]() {
                count_++;
                RCLCPP_INFO(this->get_logger(), "Lambda callback: %d", count_);
            }
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};
```

**When to use lambdas:**
- ✓ Simple, short callbacks
- ✓ When you need to capture variables
- ✓ One-time timer setups

**When to use member functions:**
- ✓ Complex logic
- ✓ Reusable callbacks
- ✓ Better for debugging

## Complete Timer Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class HeartbeatNode : public rclcpp::Node {
public:
    HeartbeatNode() : Node("heartbeat_node"), sequence_(0) {
        // Declare parameter for rate
        this->declare_parameter("heartbeat_rate", 1.0);
        double rate = this->get_parameter("heartbeat_rate").as_double();

        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("heartbeat", 10);

        // Create timer based on parameter
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = this->create_wall_timer(
            period,
            std::bind(&HeartbeatNode::publish_heartbeat, this)
        );

        RCLCPP_INFO(this->get_logger(),
                    "Heartbeat node started at %.2f Hz", rate);
    }

private:
    void publish_heartbeat() {
        auto message = std_msgs::msg::String();
        message.data = "Heartbeat #" + std::to_string(sequence_++);

        publisher_->publish(message);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000,
                             "Published %d heartbeats", sequence_);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int sequence_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeartbeatNode>());
    rclcpp::shutdown();
    return 0;
}
```

**Run with custom rate:**
```bash
ros2 run my_package heartbeat_node --ros-args -p heartbeat_rate:=2.0
```

## Multiple Timers

One node can have multiple timers:

```cpp
class MultiTimerNode : public rclcpp::Node {
public:
    MultiTimerNode() : Node("multi_timer"), fast_count_(0), slow_count_(0) {
        // Fast timer (100ms)
        fast_timer_ = create_wall_timer(
            100ms,
            std::bind(&MultiTimerNode::fast_callback, this)
        );

        // Slow timer (1s)
        slow_timer_ = create_wall_timer(
            1s,
            std::bind(&MultiTimerNode::slow_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Multi-timer node started");
    }

private:
    void fast_callback() {
        fast_count_++;
        RCLCPP_DEBUG(this->get_logger(), "Fast: %d", fast_count_);
    }

    void slow_callback() {
        slow_count_++;
        RCLCPP_INFO(this->get_logger(),
                    "Slow: %d (fast ran %d times)", slow_count_, fast_count_);
    }

    rclcpp::TimerBase::SharedPtr fast_timer_;
    rclcpp::TimerBase::SharedPtr slow_timer_;
    int fast_count_;
    int slow_count_;
};
```

## Canceling Timers

### Cancel Permanently

```cpp
class CancellableTimer : public rclcpp::Node {
public:
    CancellableTimer() : Node("cancellable"), count_(0) {
        timer_ = create_wall_timer(
            500ms,
            std::bind(&CancellableTimer::callback, this)
        );
    }

private:
    void callback() {
        count_++;
        RCLCPP_INFO(this->get_logger(), "Callback #%d", count_);

        // Stop after 10 calls
        if (count_ >= 10) {
            RCLCPP_INFO(this->get_logger(), "Canceling timer");
            timer_->cancel();
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};
```

### Check if Canceled

```cpp
if (timer_->is_canceled()) {
    RCLCPP_INFO(logger, "Timer is canceled");
} else {
    RCLCPP_INFO(logger, "Timer is active");
}
```

### Reset Timer

```cpp
// Cancel timer
timer_->cancel();

// Create new timer
timer_ = create_wall_timer(1s, callback);
```

## One-Shot Timers

Timer that fires only once:

```cpp
class OneShotTimer : public rclcpp::Node {
public:
    OneShotTimer() : Node("one_shot") {
        RCLCPP_INFO(this->get_logger(), "Starting one-shot timer (5 seconds)");

        timer_ = create_wall_timer(
            5s,
            [this]() {
                RCLCPP_INFO(this->get_logger(), "One-shot timer fired!");
                timer_->cancel();  // Cancel after first call
            }
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## Conditional Timers

### Enable/Disable

```cpp
class ConditionalTimer : public rclcpp::Node {
public:
    ConditionalTimer() : Node("conditional"), enabled_(false) {
        // Declare parameter
        this->declare_parameter("enabled", false);
        enabled_ = this->get_parameter("enabled").as_bool();

        // Create timer
        timer_ = create_wall_timer(
            1s,
            std::bind(&ConditionalTimer::callback, this)
        );

        // Parameter callback for dynamic enable/disable
        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&ConditionalTimer::on_param_change, this, std::placeholders::_1)
        );
    }

private:
    void callback() {
        // Only execute if enabled
        if (!enabled_) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Timer callback (enabled)");
    }

    rcl_interfaces::msg::SetParametersResult on_param_change(
        const std::vector<rclcpp::Parameter>& params)
    {
        for (const auto& param : params) {
            if (param.get_name() == "enabled") {
                enabled_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "Timer %s",
                            enabled_ ? "ENABLED" : "DISABLED");
            }
        }
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        return result;
    }

    bool enabled_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_;
};
```

**Control at runtime:**
```bash
# Enable
ros2 param set /conditional enabled true

# Disable
ros2 param set /conditional enabled false
```

## Timer + Publisher Pattern

Common pattern: periodic publishing

```cpp
class PeriodicPublisher : public rclcpp::Node {
public:
    PeriodicPublisher() : Node("periodic_publisher"), count_(0) {
        // Publisher
        publisher_ = create_publisher<std_msgs::msg::Int32>("counter", 10);

        // Timer
        timer_ = create_wall_timer(
            1s,
            std::bind(&PeriodicPublisher::publish_data, this)
        );
    }

private:
    void publish_data() {
        auto message = std_msgs::msg::Int32();
        message.data = count_++;

        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Published: %d", message.data);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};
```

## Timer + Service Pattern

Periodic service calls:

```cpp
class PeriodicServiceCaller : public rclcpp::Node {
public:
    PeriodicServiceCaller() : Node("periodic_caller") {
        // Service client
        client_ = create_client<std_srvs::srv::Trigger>("check_status");

        // Timer (every 5 seconds)
        timer_ = create_wall_timer(
            5s,
            std::bind(&PeriodicServiceCaller::call_service, this)
        );
    }

private:
    void call_service() {
        // Check if service is available
        if (!client_->wait_for_service(100ms)) {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }

        // Call service
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Status: %s",
                            response->message.c_str());
            }
        );
    }

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## Watchdog Timer

Monitor for events or timeout:

```cpp
class WatchdogNode : public rclcpp::Node {
public:
    WatchdogNode() : Node("watchdog"), last_message_time_(this->now()) {
        // Subscribe to messages
        subscription_ = create_subscription<std_msgs::msg::String>(
            "monitored_topic", 10,
            std::bind(&WatchdogNode::message_callback, this, std::placeholders::_1)
        );

        // Watchdog timer (check every second)
        watchdog_timer_ = create_wall_timer(
            1s,
            std::bind(&WatchdogNode::check_timeout, this)
        );

        timeout_duration_ = rclcpp::Duration(5, 0);  // 5 seconds
    }

private:
    void message_callback(const std_msgs::msg::String::SharedPtr msg) {
        last_message_time_ = this->now();
        RCLCPP_DEBUG(this->get_logger(), "Received: %s", msg->data.c_str());
    }

    void check_timeout() {
        auto time_since_last = this->now() - last_message_time_;

        if (time_since_last > timeout_duration_) {
            RCLCPP_WARN(this->get_logger(),
                        "TIMEOUT! No messages for %.2f seconds",
                        time_since_last.seconds());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Watchdog OK");
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_message_time_;
    rclcpp::Duration timeout_duration_;
};
```

## ROS Time vs Wall Time

### Wall Timer (Real Time)

```cpp
// Based on system clock (wall clock time)
timer_ = create_wall_timer(1s, callback);
```

- ✓ Always progresses at real-time rate
- ✓ Not affected by simulation
- ✓ Use for heartbeats, diagnostics

### ROS Timer (Simulated Time)

```cpp
// Based on ROS time (can be simulation time)
timer_ = create_timer(1s, callback);
```

- ✓ Respects simulation time
- ✓ Can be paused/accelerated
- ✓ Use for robot logic

**For most cases, use `create_wall_timer()`**

## Best Practices

### DO ✓

```cpp
// Use appropriate timer period
timer_ = create_wall_timer(100ms, callback);  // Not too fast!

// Keep callbacks short
void callback() {
    // Quick operation
    publish_message();
}

// Cancel when done
if (finished) {
    timer_->cancel();
}

// Use parameters for configurability
double rate = get_parameter("update_rate").as_double();
auto period = std::chrono::duration<double>(1.0 / rate);
timer_ = create_wall_timer(period, callback);

// Add safety checks
void callback() {
    if (!enabled_) return;
    // Process
}
```

### DON'T ✗

```cpp
// Don't use very short periods unnecessarily
timer_ = create_wall_timer(1ms, callback);  // Too fast for most cases!

// Don't do blocking operations
void callback() {
    sleep(5);  // Blocks everything!
}

// Don't create timers in callbacks
void callback() {
    auto timer = create_wall_timer(...);  // Leak!
}

// Don't forget to store timer
create_wall_timer(1s, callback);  // Lost! Gets destroyed immediately

// Don't access destroyed resources
void callback() {
    deleted_object_->method();  // Crash!
}
```

## Complete Example: Sensor Simulator

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <random>

using namespace std::chrono_literals;

class SensorSimulator : public rclcpp::Node {
public:
    SensorSimulator() : Node("sensor_simulator") {
        // Parameters
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("noise_level", 0.1);
        this->declare_parameter("enabled", true);

        double rate = this->get_parameter("publish_rate").as_double();
        noise_level_ = this->get_parameter("noise_level").as_double();
        enabled_ = this->get_parameter("enabled").as_bool();

        // Publisher
        publisher_ = create_publisher<std_msgs::msg::Float64>("sensor_data", 10);

        // Timer
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = create_wall_timer(
            period,
            std::bind(&SensorSimulator::publish_sensor_data, this)
        );

        // Random generator
        generator_ = std::mt19937(std::random_device{}());
        distribution_ = std::normal_distribution<double>(0.0, noise_level_);

        RCLCPP_INFO(this->get_logger(),
                    "Sensor simulator started at %.2f Hz", rate);
    }

private:
    void publish_sensor_data() {
        if (!enabled_) {
            return;
        }

        // Simulate sensor reading (sine wave + noise)
        double time = this->now().seconds();
        double value = std::sin(time) + distribution_(generator_);

        auto message = std_msgs::msg::Float64();
        message.data = value;

        publisher_->publish(message);

        RCLCPP_DEBUG(this->get_logger(), "Published: %.4f", value);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double noise_level_;
    bool enabled_;

    std::mt19937 generator_;
    std::normal_distribution<double> distribution_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorSimulator>());
    rclcpp::shutdown();
    return 0;
}
```

**Run:**
```bash
ros2 run my_package sensor_simulator --ros-args \
    -p publish_rate:=20.0 \
    -p noise_level:=0.05 \
    -p enabled:=true
```

## Summary

**Key Takeaways:**
- Timers trigger callbacks at regular intervals
- Use `create_wall_timer(period, callback)`
- Periods use `std::chrono_literals` (e.g., `1s`, `100ms`)
- Store timer as member variable
- Cancel with `timer->cancel()`
- One node can have multiple timers
- Keep callbacks short and non-blocking

**Critical Pattern:**
```cpp
// Declare timer member
rclcpp::TimerBase::SharedPtr timer_;

// Create in constructor
timer_ = create_wall_timer(1s, std::bind(&Class::callback, this));

// Callback
void callback() {
    // Periodic operation
}
```

**Common Uses:**
- Periodic publishing
- Status updates
- Sensor polling
- Heartbeat messages
- Watchdog monitoring
- Timed state transitions

## Practice Exercise

Create a timer node that:
1. Has a configurable update rate (parameter)
2. Publishes a counter value periodically
3. Cancels after 100 messages
4. Logs progress every 10 messages (throttled)
5. Can be enabled/disabled via parameter

## What's Next?

- **Next Lesson**: [Executors](12-executors.md) - Callback execution control
- **Related**: [Publishers](04-topics-publishers.md) - Periodic publishing
- **See Also**: [Parameters](09-parameters.md) - Timer configuration

---

**You can now use timers for periodic tasks!** Next: mastering executors for advanced callback control.
