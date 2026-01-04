# Exercise 1: Publisher and Subscriber

## Objective
Create a simple publisher-subscriber system where a counter node publishes numbers and a listener node receives and processes them.

## Learning Goals
- Create publishers and subscribers
- Use std_msgs message types
- Implement timer-based publishing
- Process incoming messages with callbacks

## Task Description

Create two nodes:

### 1. CounterPublisher Node
- Publishes integers to a topic named `/counter`
- Starts at 0 and increments by 1 every second
- Logs each published value

### 2. CounterListener Node
- Subscribes to the `/counter` topic
- Receives integer messages
- Logs received values
- Calculates and logs the running sum

## Requirements

### CounterPublisher
- Use `std_msgs::msg::Int32` message type
- Publish at 1 Hz (every 1 second)
- Log format: `"Publishing: <value>"`

### CounterListener
- Subscribe to `/counter` topic
- Maintain a running sum of all received values
- Log format: `"Received: <value>, Running sum: <sum>"`

## Files to Complete

**Starter code**: `starter/`
- `src/counter_publisher.cpp` - Complete the TODOs
- `src/counter_listener.cpp` - Complete the TODOs
- `CMakeLists.txt` - Already configured
- `package.xml` - Already configured

**Solution**: `solution/` (for reference after attempting)

## Build Instructions

```bash
cd ROS2-CPP-Course/02-ros2-fundamentals/exercises/01-pubsub-exercise/starter
colcon build --packages-select pubsub_exercise
source install/setup.bash
```

## Running the Exercise

Terminal 1:
```bash
ros2 run pubsub_exercise counter_publisher
```

Terminal 2:
```bash
ros2 run pubsub_exercise counter_listener
```

## Expected Output

**counter_publisher terminal:**
```
[INFO] [counter_publisher]: Starting Counter Publisher
[INFO] [counter_publisher]: Publishing: 0
[INFO] [counter_publisher]: Publishing: 1
[INFO] [counter_publisher]: Publishing: 2
...
```

**counter_listener terminal:**
```
[INFO] [counter_listener]: Counter Listener started
[INFO] [counter_listener]: Received: 0, Running sum: 0
[INFO] [counter_listener]: Received: 1, Running sum: 1
[INFO] [counter_listener]: Received: 2, Running sum: 3
[INFO] [counter_listener]: Received: 3, Running sum: 6
...
```

## Testing Your Solution

1. Both nodes should run without errors
2. Counter should increment by 1 each second
3. Running sum should be correct (sum of 0..n)
4. No missed messages (sum at message n should equal n*(n+1)/2)

## Hints

1. Use `std_msgs::msg::Int32` for the message type
2. Publisher queue size: 10 is reasonable
3. Timer period: `std::chrono::seconds(1)` or `1s` with `using namespace std::chrono_literals;`
4. Remember to include necessary headers:
   - `<rclcpp/rclcpp.hpp>`
   - `<std_msgs/msg/int32.hpp>`

## Verification Questions

After completing this exercise, you should be able to answer:
1. What is the difference between a publisher and a subscriber?
2. What does the queue size parameter control?
3. Why do we use smart pointers (`std::make_shared`) in ROS2?
4. What would happen if the subscriber processed messages slower than the publisher sent them?

## Common Mistakes

- Forgetting to include `std_msgs/msg/int32.hpp`
- Not initializing the counter or sum to 0
- Using the wrong message type
- Incorrect topic name (must match exactly: `/counter`)
- Not spinning the node in main()

## Related Lessons

- [Lesson 1: ROS2 Basics](../../lessons/01-ros2-basics.md)
- [Lesson 2: Nodes](../../lessons/02-nodes.md)
- [Lesson 3: Topics](../../lessons/03-topics.md)

## Next Steps

Once you've completed this exercise, move on to:
- **Exercise 2**: Service-based calculator
