# Lesson 12: Executors and Callbacks

## Learning Objectives

- Understand ROS2 executors
- Use single-threaded vs multi-threaded executors
- Control callback execution
- Implement callback groups
- Handle concurrent callbacks safely
- Choose appropriate executor types
- Apply executor best practices

## What are Executors?

**Executors** control how and when callbacks are executed in ROS2.

```
Executor: Manages callback queue
    ↓
Callbacks: Subscriptions, Timers, Services
    ↓
Execution: Single-threaded or Multi-threaded
```

### Why Executors Matter

- Control callback concurrency
- Manage thread safety
- Optimize performance
- Prevent blocking
- Enable real-time behavior

## Default Execution Model

### Single-Threaded (Default)

When you use `rclcpp::spin()`:

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);  // Single-threaded executor
    rclcpp::shutdown();
    return 0;
}
```

**How it works:**
- One thread processes all callbacks
- Callbacks execute **sequentially**
- If one callback blocks, all others wait
- Simple, thread-safe by default

```
Timer1 → Sub1 → Timer2 → Sub2 → Timer1 → ...
[Sequential execution, one at a time]
```

## Single-Threaded Executor

### Explicit Usage

```cpp
#include <rclcpp/executors.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // Create node
    auto node = std::make_shared<MyNode>();

    // Add node to executor
    executor.add_node(node);

    // Spin
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
```

### Multiple Nodes

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    // Add multiple nodes
    auto node1 = std::make_shared<Node1>();
    auto node2 = std::make_shared<Node2>();

    executor.add_node(node1);
    executor.add_node(node2);

    // All nodes' callbacks execute in one thread
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
```

## Multi-Threaded Executor

### Basic Usage

```cpp
#include <rclcpp/executors.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Multi-threaded executor (4 threads)
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        4  // Number of threads
    );

    auto node = std::make_shared<MyNode>();
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
```

**How it works:**
- Multiple threads process callbacks
- Callbacks can execute **concurrently**
- Faster for multiple independent operations
- **Requires thread safety!**

```
Thread 1: Timer1 → Sub1 → Timer1 → ...
Thread 2: Timer2 → Sub2 → Timer2 → ...
Thread 3: Service → Timer3 → ...
Thread 4: Sub3 → Sub3 → ...
[Concurrent execution]
```

### Example: Multi-Threaded Node

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class MultiThreadedNode : public rclcpp::Node {
public:
    MultiThreadedNode() : Node("multi_threaded") {
        // Subscriber 1 (slow processing)
        sub1_ = create_subscription<std_msgs::msg::String>(
            "topic1", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "[Sub1] Received: %s", msg->data.c_str());
                std::this_thread::sleep_for(2s);  // Simulate slow processing
                RCLCPP_INFO(this->get_logger(), "[Sub1] Done processing");
            }
        );

        // Subscriber 2 (fast processing)
        sub2_ = create_subscription<std_msgs::msg::String>(
            "topic2", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "[Sub2] Received: %s", msg->data.c_str());
                // Fast processing
                RCLCPP_INFO(this->get_logger(), "[Sub2] Done");
            }
        );

        // Timer (periodic)
        timer_ = create_wall_timer(
            1s,
            [this]() {
                RCLCPP_INFO(this->get_logger(), "[Timer] Tick");
            }
        );
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // With single-threaded executor:
    // - Sub1 blocks Sub2 and Timer
    // With multi-threaded executor:
    // - Sub2 and Timer can run while Sub1 processes

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 3
    );

    auto node = std::make_shared<MultiThreadedNode>();
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
```

## Callback Groups

Control which callbacks can run concurrently:

### Mutually Exclusive Group (Default)

Callbacks in this group **cannot run concurrently**:

```cpp
#include <rclcpp/rclcpp.hpp>

class CallbackGroupNode : public rclcpp::Node {
public:
    CallbackGroupNode() : Node("callback_group") {
        // Create mutually exclusive callback group
        callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        // Add callbacks to group
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_;

        sub_ = create_subscription<std_msgs::msg::String>(
            "topic", 10, callback, sub_options
        );
    }

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
```

### Reentrant Group

Callbacks in this group **can run concurrently**:

```cpp
class ReentrantNode : public rclcpp::Node {
public:
    ReentrantNode() : Node("reentrant"), counter_(0) {
        // Create reentrant callback group
        callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::Reentrant
        );

        // Subscribe with reentrant group
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_;

        sub_ = create_subscription<std_msgs::msg::String>(
            "topic", 10,
            std::bind(&ReentrantNode::callback, this, std::placeholders::_1),
            sub_options
        );

        // Timer with same group
        auto timer_options = rclcpp::TimerOptions();
        timer_options.callback_group = callback_group_;

        timer_ = create_wall_timer(
            1s,
            std::bind(&ReentrantNode::timer_callback, this),
            callback_group_
        );
    }

private:
    void callback(std_msgs::msg::String::SharedPtr msg) {
        // Can run concurrently with timer_callback
        std::lock_guard<std::mutex> lock(mutex_);  // Thread safety!
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Callback: %d", counter_);
    }

    void timer_callback() {
        std::lock_guard<std::mutex> lock(mutex_);  // Thread safety!
        RCLCPP_INFO(this->get_logger(), "Timer: %d", counter_);
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
    std::mutex mutex_;  // Protect shared data!
};
```

## Thread Safety

### Problem: Race Conditions

```cpp
class UnsafeNode : public rclcpp::Node {
public:
    UnsafeNode() : Node("unsafe"), counter_(0) {
        // With multi-threaded executor, this is UNSAFE!
        sub_ = create_subscription<std_msgs::msg::String>(
            "topic", 10,
            [this](auto msg) {
                counter_++;  // Race condition!
                RCLCPP_INFO(this->get_logger(), "Count: %d", counter_);
            }
        );
    }

private:
    int counter_;  // Shared between threads!
};
```

### Solution: Mutex Protection

```cpp
#include <mutex>

class SafeNode : public rclcpp::Node {
public:
    SafeNode() : Node("safe"), counter_(0) {
        sub_ = create_subscription<std_msgs::msg::String>(
            "topic", 10,
            [this](auto msg) {
                std::lock_guard<std::mutex> lock(mutex_);  // Lock
                counter_++;  // Safe!
                RCLCPP_INFO(this->get_logger(), "Count: %d", counter_);
                // Unlock automatically when lock goes out of scope
            }
        );
    }

private:
    int counter_;
    std::mutex mutex_;  // Protect counter_
};
```

### Solution: Atomic Variables

```cpp
#include <atomic>

class AtomicNode : public rclcpp::Node {
public:
    AtomicNode() : Node("atomic"), counter_(0) {
        sub_ = create_subscription<std_msgs::msg::String>(
            "topic", 10,
            [this](auto msg) {
                counter_++;  // Atomic, thread-safe!
                RCLCPP_INFO(this->get_logger(), "Count: %d", counter_.load());
            }
        );
    }

private:
    std::atomic<int> counter_;  // Thread-safe without mutex
};
```

## Executor Comparison

| Feature | Single-Threaded | Multi-Threaded |
|---------|----------------|----------------|
| **Threads** | 1 | Multiple (configurable) |
| **Concurrency** | Sequential | Parallel |
| **Thread safety** | Not needed | Required |
| **Complexity** | Simple | Complex |
| **Performance** | Lower | Higher (if callbacks independent) |
| **Use case** | Simple nodes | Heavy processing, multiple operations |

### When to Use Single-Threaded

✓ Simple nodes with few callbacks
✓ No blocking operations
✓ Don't need thread safety
✓ Easier to debug
✓ Most common case

### When to Use Multi-Threaded

✓ Multiple independent operations
✓ Some callbacks block/take time
✓ Need concurrency
✓ Performance critical
✓ Complex systems

## Spin Variants

### spin()

```cpp
executor.spin();  // Blocks forever
```

- Runs until Ctrl+C or shutdown
- Most common usage

### spin_some()

```cpp
while (rclcpp::ok()) {
    executor.spin_some();  // Process available callbacks once
    // Do other work
    std::this_thread::sleep_for(10ms);
}
```

- Process available callbacks, then return
- Useful for custom loops

### spin_until_future_complete()

```cpp
auto future = client->async_send_request(request);
executor.spin_until_future_complete(future);
auto response = future.get();
```

- Spin until specific future completes
- Used for synchronous service calls

### spin_all()

```cpp
executor.spin_all(std::chrono::milliseconds(100));
```

- Process all available callbacks with timeout
- Returns after timeout or when queue empty

## Complete Example: Multi-Callback Node

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>

using namespace std::chrono_literals;

class ComplexNode : public rclcpp::Node {
public:
    ComplexNode() : Node("complex_node"), count_(0) {
        // Create callback groups
        fast_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        slow_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        // Fast subscriber (in fast_group)
        auto fast_options = rclcpp::SubscriptionOptions();
        fast_options.callback_group = fast_group_;

        fast_sub_ = create_subscription<std_msgs::msg::String>(
            "fast_topic", 10,
            std::bind(&ComplexNode::fast_callback, this, std::placeholders::_1),
            fast_options
        );

        // Slow subscriber (in slow_group)
        auto slow_options = rclcpp::SubscriptionOptions();
        slow_options.callback_group = slow_group_;

        slow_sub_ = create_subscription<std_msgs::msg::String>(
            "slow_topic", 10,
            std::bind(&ComplexNode::slow_callback, this, std::placeholders::_1),
            slow_options
        );

        // Timer (in fast_group)
        timer_ = create_wall_timer(
            1s,
            std::bind(&ComplexNode::timer_callback, this),
            fast_group_
        );

        // Service (in fast_group)
        service_ = create_service<std_srvs::srv::Trigger>(
            "get_count",
            std::bind(&ComplexNode::service_callback, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            fast_group_
        );

        RCLCPP_INFO(this->get_logger(), "Complex node started");
    }

private:
    void fast_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_INFO(this->get_logger(), "Fast: %s", msg->data.c_str());
    }

    void slow_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Slow: Processing %s...", msg->data.c_str());
        std::this_thread::sleep_for(2s);  // Simulate slow processing
        std::lock_guard<std::mutex> lock(mutex_);
        count_++;
        RCLCPP_INFO(this->get_logger(), "Slow: Done. Count: %d", count_);
    }

    void timer_callback() {
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_INFO(this->get_logger(), "Timer tick. Count: %d", count_);
    }

    void service_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response->success = true;
        response->message = "Count: " + std::to_string(count_);
        RCLCPP_INFO(this->get_logger(), "Service called");
    }

    rclcpp::CallbackGroup::SharedPtr fast_group_;
    rclcpp::CallbackGroup::SharedPtr slow_group_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fast_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr slow_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    int count_;
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 3
    );

    auto node = std::make_shared<ComplexNode>();
    executor.add_node(node);

    // Slow callback won't block fast callbacks!
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
```

## Best Practices

### DO ✓

```cpp
// Use single-threaded for simple nodes
rclcpp::spin(node);  // Simple and safe

// Protect shared data in multi-threaded executors
std::lock_guard<std::mutex> lock(mutex_);
shared_data_++;

// Use atomic for simple counters
std::atomic<int> counter_;

// Separate callback groups for independent operations
auto group1 = create_callback_group(MutuallyExclusive);
auto group2 = create_callback_group(MutuallyExclusive);

// Keep callbacks short
void callback() {
    // Quick operation
    process_message();
}
```

### DON'T ✗

```cpp
// Don't access shared data without protection
void callback() {
    counter_++;  // Race condition with multi-threaded!
}

// Don't block in single-threaded executor
void callback() {
    std::this_thread::sleep_for(5s);  // Blocks everything!
}

// Don't use multi-threading unnecessarily
// If single-threaded works, use it (simpler!)

// Don't forget to add node to executor
executor.spin();  // No nodes added!
```

## Summary

**Key Takeaways:**
- Executors control callback execution
- Single-threaded: callbacks run sequentially (default)
- Multi-threaded: callbacks can run concurrently
- Callback groups control concurrency
- Multi-threading requires thread safety (mutex/atomic)
- Most nodes use single-threaded executor
- Use multi-threaded for performance-critical applications

**Executor Types:**

**Single-Threaded:**
```cpp
rclcpp::spin(node);  // Simple
```

**Multi-Threaded:**
```cpp
rclcpp::executors::MultiThreadedExecutor executor(opts, threads);
executor.add_node(node);
executor.spin();
```

**Thread Safety:**
```cpp
std::mutex mutex_;
std::lock_guard<std::mutex> lock(mutex_);
// Access shared data safely
```

**Callback Groups:**
```cpp
// Mutually exclusive (default)
auto group = create_callback_group(MutuallyExclusive);

// Reentrant (can run concurrently)
auto group = create_callback_group(Reentrant);
```

## Practice Exercise

Create a node with:
1. Two subscriptions with different processing times
2. A timer that runs every second
3. Use multi-threaded executor
4. Ensure thread safety for shared counter
5. Observe that slow callback doesn't block timer

## What's Next?

- **Module 2 Complete!** All fundamentals covered
- **Next Module**: [ROS2 Intermediate](../../03-ros2-intermediate/README.md)
- **Topics**: Custom messages, launch files, tf2, actions
- **Practice**: Complete Module 2 exercises and mini-project

---

**Congratulations!** You've completed ROS2 Fundamentals! 🎉

You now know:
- ROS2 architecture and concepts
- Creating nodes, publishers, subscribers
- Services (servers and clients)
- Parameters for configuration
- Professional logging
- Timers for periodic tasks
- Executors for callback control

**Next: Module 3 - ROS2 Intermediate** where you'll learn about custom messages, launch files, coordinate transforms (tf2), and actions for complex robot behaviors!
