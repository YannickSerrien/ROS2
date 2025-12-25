# Lesson 4: Topics - Publishers

## Learning Objectives

- Understand the publish-subscribe pattern
- Create a ROS2 publisher
- Publish different message types
- Use timers for periodic publishing
- Apply best practices for publishers

## What are Topics?

**Topics** are named channels for message passing in ROS2. They use the **publish-subscribe** pattern:

- **Publishers** send messages to topics
- **Subscribers** receive messages from topics
- **Many-to-many**: Multiple publishers and subscribers per topic
- **Asynchronous**: Fire-and-forget communication

```
┌────────────┐                    ┌─────────────┐
│ Publisher  │──messages────────▶│ Subscriber  │
└────────────┘                    └─────────────┘
     │                                   │
     │         ┌────────┐                │
     └────────▶│ Topic  │◀───────────────┘
               └────────┘
```

## Your First Publisher

### Minimal Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("minimal_publisher") {
        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

        // Create timer (500ms)
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&MinimalPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, ROS2!";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

### Breaking It Down

#### 1. Include Message Header

```cpp
#include <std_msgs/msg/string.hpp>
```
- Message types are in separate packages
- Format: `<package>/msg/<type>.hpp`
- Common packages: `std_msgs`, `geometry_msgs`, `sensor_msgs`

#### 2. Create Publisher

```cpp
publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
```
- **Template parameter**: Message type
- **First argument**: Topic name
- **Second argument**: Queue size (QoS history depth)

#### 3. Create Message

```cpp
auto message = std_msgs::msg::String();
message.data = "Hello, ROS2!";
```
- Instantiate message object
- Set message fields

#### 4. Publish

```cpp
publisher_->publish(message);
```
- Send message to all subscribers
- Non-blocking operation

## Message Types

### std_msgs - Basic Types

```cpp
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

// Int32
auto int_msg = std_msgs::msg::Int32();
int_msg.data = 42;

// Float64
auto float_msg = std_msgs::msg::Float64();
float_msg.data = 3.14159;

// Bool
auto bool_msg = std_msgs::msg::Bool();
bool_msg.data = true;

// String
auto string_msg = std_msgs::msg::String();
string_msg.data = "hello";
```

### geometry_msgs - Geometric Types

```cpp
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

// Twist (velocity command)
auto twist_msg = geometry_msgs::msg::Twist();
twist_msg.linear.x = 1.0;   // Forward velocity
twist_msg.angular.z = 0.5;  // Rotation velocity

// Point
auto point_msg = geometry_msgs/msg::Point();
point_msg.x = 1.0;
point_msg.y = 2.0;
point_msg.z = 3.0;
```

### sensor_msgs - Sensor Data

```cpp
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

// LaserScan
auto scan_msg = sensor_msgs::msg::LaserScan();
scan_msg.ranges = {1.0, 1.5, 2.0, ...};
scan_msg.angle_min = -1.57;
scan_msg.angle_max = 1.57;
```

## Complete Publisher Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CounterPublisher : public rclcpp::Node {
public:
    CounterPublisher() : Node("counter_publisher"), count_(0) {
        // Create publisher with QoS
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);

        // Timer callback every 1 second
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&CounterPublisher::publish_count, this)
        );

        RCLCPP_INFO(this->get_logger(), "Counter publisher started");
    }

private:
    void publish_count() {
        auto message = std_msgs::msg::Int32();
        message.data = count_++;

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CounterPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

## Publishing with unique_ptr (Efficient)

For better performance, use `unique_ptr` with move semantics:

```cpp
void publish_efficient() {
    // Create message with unique_ptr
    auto message = std::make_unique<std_msgs::msg::String>();
    message->data = "Efficient message";

    // Move (zero-copy)
    publisher_->publish(std::move(message));
    // message is now nullptr!
}
```

## Quality of Service (QoS)

Control how messages are delivered:

```cpp
// Default QoS (reliable, volatile, keep last 10)
auto pub = this->create_publisher<MsgType>("topic", 10);

// Sensor data QoS (best effort)
auto sensor_qos = rclcpp::SensorDataQoS();
auto pub = this->create_publisher<MsgType>("topic", sensor_qos);

// Custom QoS
auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
auto pub = this->create_publisher<MsgType>("topic", custom_qos);
```

### QoS Policies

```cpp
// Reliability
.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)       // Guaranteed delivery
.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)    // May drop messages

// Durability (for late-joining subscribers)
.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)  // Keep last message
.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)         // Don't keep

// History
.keep_last(10)    // Keep last N messages
.keep_all()       // Keep all messages
```

## Multi-Publisher Node

Publishing to multiple topics:

```cpp
class MultiPublisher : public rclcpp::Node {
public:
    MultiPublisher() : Node("multi_publisher") {
        // Create multiple publishers
        string_pub_ = create_publisher<std_msgs::msg::String>("strings", 10);
        int_pub_ = create_publisher<std_msgs::msg::Int32>("numbers", 10);

        timer_ = create_wall_timer(
            1s,
            std::bind(&MultiPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        // Publish to different topics
        auto str_msg = std_msgs::msg::String();
        str_msg.data = "Hello";
        string_pub_->publish(str_msg);

        auto int_msg = std_msgs::msg::Int32();
        int_msg.data = 42;
        int_pub_->publish(int_msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr int_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## Timers

### Wall Timer (Real-world Time)

```cpp
using namespace std::chrono_literals;

// 1 second timer
timer_ = create_wall_timer(1s, callback);

// 100 milliseconds
timer_ = create_wall_timer(100ms, callback);

// With lambda
timer_ = create_wall_timer(500ms, [this]() {
    this->publish_message();
});
```

### Timer with std::chrono

```cpp
auto period = std::chrono::milliseconds(500);
timer_ = create_wall_timer(period, callback);
```

## Best Practices

### DO ✓

```cpp
// Use descriptive topic names
create_publisher<MsgType>("/robot/sensors/lidar", 10);

// Use unique_ptr for efficiency
auto msg = std::make_unique<MsgType>();
publisher->publish(std::move(msg));

// Check subscriber count
if (publisher_->get_subscription_count() > 0) {
    publisher_->publish(msg);
}

// Use appropriate QoS
auto qos = rclcpp::SensorDataQoS();  // For sensor data
auto qos = rclcpp::SystemDefaultsQoS();  // For commands
```

### DON'T ✗

```cpp
// Don't use generic names
create_publisher<MsgType>("data", 10);  // Bad

// Don't publish in constructor (timing issues)
MyNode() : Node("node") {
    publisher_ = create_publisher<MsgType>("topic", 10);
    publisher_->publish(msg);  // Too early!
}

// Don't publish without timer/trigger
while (rclcpp::ok()) {
    publisher_->publish(msg);  // Blocks spin!
}
```

## Common Patterns

### Pattern 1: Periodic Publishing

```cpp
class PeriodicPublisher : public rclcpp::Node {
public:
    PeriodicPublisher() : Node("periodic") {
        pub_ = create_publisher<MsgType>("topic", 10);
        timer_ = create_wall_timer(1s, std::bind(&PeriodicPublisher::publish, this));
    }

private:
    void publish() {
        auto msg = MsgType();
        // Fill message
        pub_->publish(msg);
    }
};
```

### Pattern 2: Event-Driven Publishing

```cpp
class EventPublisher : public rclcpp::Node {
public:
    void on_event(const Data& data) {
        auto msg = MsgType();
        msg.data = data;
        publisher_->publish(msg);
    }
};
```

## Testing Your Publisher

### Command Line

```bash
# Run publisher
ros2 run my_package publisher_node

# In another terminal - list topics
ros2 topic list

# Echo messages
ros2 topic echo /topic_name

# Check message rate
ros2 topic hz /topic_name

# Check number of publishers
ros2 topic info /topic_name
```

## Summary

**Key Takeaways:**
- Publishers send messages to topics
- Use `create_publisher<Type>("name", qos)`
- Timers enable periodic publishing
- Move semantics (`unique_ptr`) for efficiency
- QoS controls message delivery
- Always initialize publisher before using

**Critical Pattern:**
```cpp
publisher_ = create_publisher<MsgType>("topic", 10);
timer_ = create_wall_timer(1s, callback);
// In callback:
publisher_->publish(message);
```

## Practice Exercise

Create a publisher that:
1. Publishes temperature values (Float64)
2. Updates every 500ms
3. Increments temperature by 0.1 each time
4. Starts at 20.0°C

## What's Next?

- **Next Lesson**: [Topics: Subscribers](05-topics-subscribers.md)
- **See Also**: [Custom Messages](../03-ros2-intermediate/01-custom-messages.md)

---

**You can now publish data in ROS2!** Next: receiving data with subscribers.
