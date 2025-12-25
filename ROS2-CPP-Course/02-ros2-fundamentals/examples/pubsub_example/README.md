# Publisher and Subscriber Example

This package demonstrates basic ROS2 topic communication using publishers and subscribers.

## Overview

This package contains three executable nodes:
1. **talker** - Simple publisher that sends string messages periodically
2. **listener** - Subscriber that receives and logs messages
3. **parameter_talker** - Configurable publisher using ROS2 parameters

## Concepts Demonstrated

- Creating publishers with `create_publisher<>()`
- Creating subscribers with `create_subscription<>()`
- Timer-based periodic publishing
- Message handling in callbacks
- Using parameters for node configuration
- Lambda vs std::bind for callbacks
- Basic logging with RCLCPP_INFO

## Building

```bash
# From your workspace root
cd ~/ros2_ws

# Copy this package to src/ if needed
cp -r path/to/pubsub_example src/

# Build
colcon build --packages-select pubsub_example

# Source the workspace
source install/setup.bash
```

## Running the Examples

### Example 1: Basic Talker and Listener

**Terminal 1 - Start the talker:**
```bash
ros2 run pubsub_example talker
```

You should see output like:
```
[INFO] [talker]: Talker node started, publishing on 'chatter' topic
[INFO] [talker]: Publishing: 'Hello, ROS2! Message #0'
[INFO] [talker]: Publishing: 'Hello, ROS2! Message #1'
...
```

**Terminal 2 - Start the listener:**
```bash
ros2 run pubsub_example listener
```

You should see output like:
```
[INFO] [listener_lambda]: Listener (lambda) node started
[INFO] [listener_lambda]: Lambda heard: 'Hello, ROS2! Message #5'
[INFO] [listener_lambda]: Lambda heard: 'Hello, ROS2! Message #6'
...
```

**Terminal 3 - Inspect the topic:**
```bash
# List active topics
ros2 topic list

# Show topic info
ros2 topic info /chatter

# Echo messages
ros2 topic echo /chatter

# Show message rate
ros2 topic hz /chatter
```

### Example 2: Parameter-Driven Publisher

**Default parameters:**
```bash
ros2 run pubsub_example parameter_talker
```

**Custom topic and rate:**
```bash
ros2 run pubsub_example parameter_talker --ros-args \
  -p topic:=custom_topic \
  -p rate:=2.0 \
  -p message_prefix:="Custom message"
```

**Listen to custom topic:**
```bash
ros2 topic echo /custom_topic
```

**Set parameters at runtime:**
```bash
# In another terminal, list parameters
ros2 param list /parameter_talker

# Get parameter value
ros2 param get /parameter_talker rate

# Note: Changing parameters at runtime won't affect this node
# since it only reads parameters during initialization
```

## Code Structure

### talker_node.cpp
```
TalkerNode
├── publisher_      (Publisher shared pointer)
├── timer_          (Timer shared pointer)
├── count_          (Message counter)
└── timer_callback() (Publish function)
```

### listener_node.cpp
```
ListenerNode
├── subscription_     (Subscription shared pointer)
└── topic_callback()  (Message handler)

ListenerNodeLambda
├── subscription_     (Subscription shared pointer)
└── [lambda callback] (Inline message handler)
```

### parameter_talker.cpp
```
ParameterTalkerNode
├── publisher_       (Publisher shared pointer)
├── timer_           (Timer shared pointer)
├── message_prefix_  (Parameter-driven prefix)
├── count_           (Message counter)
└── timer_callback() (Publish function)
```

## Key Concepts

### Publisher Pattern
```cpp
// 1. Declare publisher member
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

// 2. Create publisher in constructor
publisher_ = this->create_publisher<std_msgs::msg::String>("topic_name", queue_size);

// 3. Publish messages
auto msg = std_msgs::msg::String();
msg.data = "Hello";
publisher_->publish(msg);
```

### Subscriber Pattern (std::bind)
```cpp
// 1. Declare subscription member
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

// 2. Create subscription with std::bind callback
subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic_name",
    queue_size,
    std::bind(&MyNode::callback, this, std::placeholders::_1));

// 3. Define callback
void callback(const std_msgs::msg::String::SharedPtr msg) {
    // Handle message
}
```

### Subscriber Pattern (Lambda)
```cpp
// Create subscription with lambda callback
subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic_name",
    queue_size,
    [this](const std_msgs::msg::String::SharedPtr msg) {
        // Handle message inline
    });
```

### Parameter Usage
```cpp
// 1. Declare parameter with default
this->declare_parameter("param_name", default_value);

// 2. Get parameter value
auto value = this->get_parameter("param_name").as_string();
double rate = this->get_parameter("rate").as_double();
int count = this->get_parameter("count").as_int();
```

## Troubleshooting

**Issue: "Package not found"**
```bash
# Make sure you built and sourced
colcon build --packages-select pubsub_example
source install/setup.bash
```

**Issue: "No messages received"**
- Check that both talker and listener are running
- Verify topic name matches: `ros2 topic list`
- Check message types: `ros2 topic info /chatter`

**Issue: "Parameter not found"**
- Make sure parameter name is correct
- Check declared parameters: `ros2 param list /node_name`

## Learning Exercises

1. **Modify talker**: Change the publishing rate to 2 Hz
2. **Add timestamp**: Include a timestamp in each message
3. **Multiple topics**: Make talker publish to two different topics
4. **Message filtering**: Make listener only log messages containing specific text
5. **QoS settings**: Experiment with different QoS profiles (reliable vs best effort)

## Related Lessons

- [Lesson 4: Topics and Publishers](../../lessons/04-topics-publishers.md)
- [Lesson 5: Topics and Subscribers](../../lessons/05-topics-subscribers.md)
- [Lesson 9: Parameters](../../lessons/09-parameters.md)
- [Lesson 10: Logging](../../lessons/10-logging.md)
- [Lesson 11: Timers](../../lessons/11-timers.md)

## Next Steps

After mastering this example:
1. Try the **service_example** package for request-response communication
2. Experiment with different message types (geometry_msgs, sensor_msgs)
3. Add QoS configuration for reliability and durability
4. Create a processing pipeline with multiple publishers and subscribers
