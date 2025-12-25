# Lesson 5: Topics - Subscribers

## Learning Objectives

- Create ROS2 subscribers
- Handle incoming messages with callbacks
- Understand callback execution models
- Use lambda functions vs member functions
- Apply QoS compatibility rules
- Implement multi-subscription patterns
- Debug subscriber issues

## What are Subscribers?

**Subscribers** receive messages from topics. They work alongside publishers to form the publish-subscribe pattern:

```
Publisher ──publishes──▶ Topic ──delivers──▶ Subscriber
                                              (callback)
```

### Key Characteristics

- **Asynchronous**: Callbacks triggered when messages arrive
- **Non-blocking**: Don't wait for messages
- **Automatic**: ROS2 manages message delivery
- **Flexible**: Multiple subscribers per topic
- **QoS-aware**: Must match publisher QoS

## Your First Subscriber

### Minimal Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        // Create subscriber
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic",
            10,
            std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

### Breaking It Down

#### 1. Include Message Header

```cpp
#include <std_msgs/msg/string.hpp>
```
- Same as publisher - must match message type

#### 2. Create Subscriber

```cpp
subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic",           // Topic name
    10,                // QoS history depth
    callback           // Callback function
);
```

**Parameters:**
- **Template**: Message type
- **Topic name**: Must match publisher
- **QoS**: Quality of service settings
- **Callback**: Function to handle messages

#### 3. Callback Function

```cpp
void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    // Process message
    RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
}
```

**Message parameter:**
- `SharedPtr` - Shared ownership (efficient)
- `const` - Read-only (good practice)
- Automatically managed lifetime

#### 4. Bind Callback

```cpp
std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1)
```
- Binds member function to subscriber
- `this` - Current object pointer
- `_1` - Placeholder for message parameter

## Callback Patterns

### Pattern 1: Member Function (Standard)

```cpp
class MySubscriber : public rclcpp::Node {
public:
    MySubscriber() : Node("subscriber") {
        sub_ = create_subscription<MsgType>(
            "topic", 10,
            std::bind(&MySubscriber::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const MsgType::SharedPtr msg) {
        // Process message
    }

    rclcpp::Subscription<MsgType>::SharedPtr sub_;
};
```

✓ **Best for:** Complex logic, accessing member variables

### Pattern 2: Lambda Function

```cpp
class MySubscriber : public rclcpp::Node {
public:
    MySubscriber() : Node("subscriber") {
        sub_ = create_subscription<MsgType>(
            "topic", 10,
            [this](const MsgType::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Got: %d", msg->data);
                counter_++;  // Access members
            }
        );
    }

private:
    int counter_ = 0;
    rclcpp::Subscription<MsgType>::SharedPtr sub_;
};
```

✓ **Best for:** Simple processing, inline logic

### Pattern 3: Const Reference (More Efficient)

```cpp
void callback(const MsgType::SharedPtr msg) {
    // Use SharedPtr
}

// OR even better for large messages:
void callback(const MsgType::ConstSharedPtr msg) {
    // Explicitly const
}

// OR for small messages:
void callback(MsgType::ConstSharedPtr msg) {
    // Direct copy (auto, int, etc.)
}
```

## Complete Subscriber Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class CounterSubscriber : public rclcpp::Node {
public:
    CounterSubscriber() : Node("counter_subscriber"), count_(0) {
        // Create subscriber
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "counter",
            10,
            std::bind(&CounterSubscriber::counter_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Counter subscriber started");
    }

private:
    void counter_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        count_++;
        RCLCPP_INFO(
            this->get_logger(),
            "Received: %d (total messages: %d)",
            msg->data,
            count_
        );
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    int count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CounterSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

## Message Types

### Subscribing to Different Types

```cpp
// String messages
#include <std_msgs/msg/string.hpp>
auto sub = create_subscription<std_msgs::msg::String>(...);

// Numeric messages
#include <std_msgs/msg/float64.hpp>
auto sub = create_subscription<std_msgs::msg::Float64>(...);

// Geometry messages
#include <geometry_msgs/msg/twist.hpp>
auto sub = create_subscription<geometry_msgs::msg::Twist>(...);

// Sensor messages
#include <sensor_msgs/msg/laser_scan.hpp>
auto sub = create_subscription<sensor_msgs::msg::LaserScan>(...);
```

### Processing Complex Messages

```cpp
void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Access nested fields
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    RCLCPP_INFO(
        this->get_logger(),
        "Velocity - Linear: %.2f, Angular: %.2f",
        linear_x, angular_z
    );
}
```

## Quality of Service (QoS)

### QoS Compatibility

Publisher and subscriber QoS must be **compatible**:

```cpp
// Publisher uses reliable
auto pub_qos = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

// Subscriber MUST use reliable (compatible)
auto sub_qos = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
```

### Common QoS Patterns

```cpp
// Default (reliable, volatile)
auto sub = create_subscription<MsgType>("topic", 10, callback);

// Sensor data (best effort)
auto qos = rclcpp::SensorDataQoS();
auto sub = create_subscription<MsgType>("topic", qos, callback);

// Custom QoS
auto qos = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
auto sub = create_subscription<MsgType>("topic", qos, callback);
```

### QoS Compatibility Rules

| Publisher | Subscriber | Compatible? |
|-----------|------------|-------------|
| Reliable | Reliable | ✓ Yes |
| Reliable | Best Effort | ✓ Yes |
| Best Effort | Reliable | ✗ No |
| Best Effort | Best Effort | ✓ Yes |

```cpp
// Check QoS compatibility
ros2 topic info /topic_name --verbose
```

## Multi-Subscription Node

Subscribe to multiple topics:

```cpp
class MultiSubscriber : public rclcpp::Node {
public:
    MultiSubscriber() : Node("multi_subscriber") {
        // Subscribe to strings
        string_sub_ = create_subscription<std_msgs::msg::String>(
            "strings", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "String: %s", msg->data.c_str());
            }
        );

        // Subscribe to numbers
        int_sub_ = create_subscription<std_msgs::msg::Int32>(
            "numbers", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Number: %d", msg->data);
            }
        );
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr int_sub_;
};
```

## Callback Execution

### How Callbacks Work

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MySubscriber>();

    // spin() processes callbacks
    rclcpp::spin(node);  // Blocks here, executes callbacks when messages arrive

    rclcpp::shutdown();
    return 0;
}
```

### Single-Threaded Execution (Default)

```cpp
// Callbacks execute sequentially
rclcpp::spin(node);

// Messages queued if callback is slow
```

### Multi-Threaded Execution

```cpp
#include <rclcpp/executors.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MySubscriber>();

    // Multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
```

## Processing Patterns

### Pattern 1: Direct Processing

```cpp
void callback(const MsgType::SharedPtr msg) {
    // Process immediately
    process_data(msg->data);
    update_state(msg);
}
```

✓ Simple, synchronous
✗ Blocks other callbacks

### Pattern 2: Store and Process Later

```cpp
class SmartSubscriber : public rclcpp::Node {
public:
    SmartSubscriber() : Node("smart_subscriber") {
        sub_ = create_subscription<MsgType>(
            "topic", 10,
            [this](const MsgType::SharedPtr msg) {
                // Store message
                latest_msg_ = msg;
            }
        );

        // Process in timer
        timer_ = create_wall_timer(
            100ms,
            [this]() { process_latest_message(); }
        );
    }

private:
    void process_latest_message() {
        if (latest_msg_) {
            // Heavy processing here
            RCLCPP_INFO(this->get_logger(), "Processing: %d", latest_msg_->data);
        }
    }

    MsgType::SharedPtr latest_msg_;
    rclcpp::Subscription<MsgType>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

✓ Non-blocking callbacks
✓ Control processing rate
✗ More complex

### Pattern 3: Message Filtering

```cpp
void callback(const std_msgs::msg::Float64::SharedPtr msg) {
    // Filter based on value
    if (msg->data > threshold_) {
        RCLCPP_WARN(this->get_logger(), "High value: %.2f", msg->data);
        handle_high_value(msg->data);
    }
}
```

## Pub-Sub Node (Both Publisher and Subscriber)

```cpp
class EchoNode : public rclcpp::Node {
public:
    EchoNode() : Node("echo_node") {
        // Subscribe to input
        sub_ = create_subscription<std_msgs::msg::String>(
            "input", 10,
            std::bind(&EchoNode::input_callback, this, std::placeholders::_1)
        );

        // Publish to output
        pub_ = create_publisher<std_msgs::msg::String>("output", 10);
    }

private:
    void input_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Echo message to output topic
        RCLCPP_INFO(this->get_logger(), "Echoing: %s", msg->data.c_str());

        auto output_msg = std_msgs::msg::String();
        output_msg.data = "ECHO: " + msg->data;
        pub_->publish(output_msg);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
```

## Best Practices

### DO ✓

```cpp
// Use const references
void callback(const MsgType::SharedPtr msg) {
    // msg is const
}

// Keep callbacks fast
void callback(const MsgType::SharedPtr msg) {
    // Quick processing only
    data_ = msg->data;  // Fast
}

// Check for nullptr (paranoid mode)
void callback(const MsgType::SharedPtr msg) {
    if (!msg) return;
    // Process
}

// Use appropriate QoS
auto qos = rclcpp::SensorDataQoS();  // For sensors
auto sub = create_subscription<MsgType>("topic", qos, callback);
```

### DON'T ✗

```cpp
// Don't do heavy processing in callback
void callback(const MsgType::SharedPtr msg) {
    expensive_computation();  // Blocks other callbacks!
}

// Don't modify message
void callback(const MsgType::SharedPtr msg) {
    msg->data = 42;  // Bad! Shared with other subscribers
}

// Don't call spin in callback
void callback(const MsgType::SharedPtr msg) {
    rclcpp::spin_some(node);  // Deadlock risk!
}

// Don't create subscribers in callbacks
void callback(const MsgType::SharedPtr msg) {
    auto new_sub = create_subscription<...>(...);  // Memory leak!
}
```

## Message Synchronization

### Problem: Multiple Topics

When you need data from multiple topics at the same time:

```cpp
class SyncSubscriber : public rclcpp::Node {
public:
    SyncSubscriber() : Node("sync_subscriber") {
        camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "camera", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                latest_image_ = msg;
            }
        );

        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "lidar", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                latest_scan_ = msg;
                process_fused_data();  // Process when both available
            }
        );
    }

private:
    void process_fused_data() {
        if (latest_image_ && latest_scan_) {
            RCLCPP_INFO(this->get_logger(), "Processing fused sensor data");
            // Combine camera and lidar data
        }
    }

    sensor_msgs::msg::Image::SharedPtr latest_image_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
};
```

## Testing Your Subscriber

### Command Line Testing

```bash
# Run subscriber
ros2 run my_package subscriber_node

# In another terminal - publish test message
ros2 topic pub /topic std_msgs/msg/String "data: 'Hello!'"

# Publish continuously (1 Hz)
ros2 topic pub -r 1 /topic std_msgs/msg/String "data: 'Hello!'"

# Check subscriber is connected
ros2 topic info /topic

# Monitor message rate
ros2 topic hz /topic
```

### Debugging Subscribers

```bash
# Check if topic exists
ros2 topic list

# Check message type
ros2 topic type /topic

# Check QoS settings
ros2 topic info /topic --verbose

# Echo topic to verify data
ros2 topic echo /topic
```

## Common Issues

### Issue 1: No Messages Received

**Problem:**
```cpp
// Subscriber created but no messages
```

**Solutions:**
```bash
# Check topic name matches
ros2 topic list

# Check publisher exists
ros2 topic info /topic

# Check QoS compatibility
ros2 topic info /topic --verbose

# Check message type
ros2 topic type /topic
```

### Issue 2: QoS Incompatibility

**Problem:**
```cpp
// Publisher: best effort
// Subscriber: reliable
// Result: No connection!
```

**Solution:**
```cpp
// Match QoS policies
auto qos = rclcpp::SensorDataQoS();  // Both use same
```

### Issue 3: Slow Callback

**Problem:**
```cpp
void callback(const MsgType::SharedPtr msg) {
    slow_processing();  // Blocks executor!
}
```

**Solution:**
```cpp
// Store and process later
void callback(const MsgType::SharedPtr msg) {
    message_queue_.push(msg);  // Fast
}

// Process in timer or separate thread
```

## Complete Example: Temperature Monitor

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class TemperatureMonitor : public rclcpp::Node {
public:
    TemperatureMonitor() : Node("temperature_monitor"), count_(0), sum_(0.0) {
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "temperature",
            10,
            std::bind(&TemperatureMonitor::temp_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Temperature monitor started");
    }

private:
    void temp_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        count_++;
        sum_ += msg->data;
        double average = sum_ / count_;

        RCLCPP_INFO(
            this->get_logger(),
            "Temp: %.2f°C | Average: %.2f°C | Samples: %d",
            msg->data, average, count_
        );

        // Alert on high temperature
        if (msg->data > 30.0) {
            RCLCPP_WARN(this->get_logger(), "HIGH TEMPERATURE ALERT!");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    int count_;
    double sum_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureMonitor>());
    rclcpp::shutdown();
    return 0;
}
```

## Summary

**Key Takeaways:**
- Subscribers receive messages via callbacks
- Use `create_subscription<Type>("topic", qos, callback)`
- Callbacks execute in `spin()` loop
- QoS must be compatible with publisher
- Keep callbacks fast and non-blocking
- Use lambdas for simple logic, member functions for complex

**Critical Pattern:**
```cpp
subscription_ = create_subscription<MsgType>(
    "topic",
    10,
    std::bind(&MyNode::callback, this, std::placeholders::_1)
);

void callback(const MsgType::SharedPtr msg) {
    // Process message
}
```

**Callback Choices:**
1. **Member function**: `std::bind(&Class::method, this, _1)`
2. **Lambda**: `[this](const MsgType::SharedPtr msg) { ... }`
3. **Standalone**: `[](const MsgType::SharedPtr msg) { ... }`

## Practice Exercise

Create a subscriber that:
1. Subscribes to Float64 messages on "sensor_data"
2. Calculates running average
3. Prints min, max, and average every 10 messages
4. Warns if value exceeds 100.0

## What's Next?

- **Next Lesson**: [Services: Overview](06-services-overview.md)
- **Related**: [Parameters](09-parameters.md)
- **Advanced**: [Executors and Callbacks](12-executors.md)

---

**You can now receive data in ROS2!** Next: request-response with services.
