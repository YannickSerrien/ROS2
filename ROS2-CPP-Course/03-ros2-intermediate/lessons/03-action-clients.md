# Lesson 3: Action Clients

## Learning Objectives

By the end of this lesson, you will be able to:
- Create action clients to send goals to action servers
- Configure feedback and result callbacks
- Monitor action execution progress
- Cancel goals programmatically
- Handle multiple concurrent goals
- Implement proper error handling for action calls

## Introduction

In Lesson 2, you built action servers that execute tasks. Now you'll build the **client side**—code that sends goals, monitors progress, and processes results.

An action client is like a task requester that:
1. Sends goals to servers
2. Monitors progress via feedback
3. Receives final results
4. Can cancel tasks mid-execution

## Action Client Components

### Required Includes

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "my_package/action/fibonacci.hpp"

using Fibonacci = my_package::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
```

### Creating an Action Client

```cpp
class FibonacciActionClient : public rclcpp::Node
{
public:
    FibonacciActionClient() : Node("fibonacci_action_client")
    {
        // Create action client
        this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
            this,
            "fibonacci");  // Action name (must match server)

        RCLCPP_INFO(this->get_logger(), "Action client created");
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
};
```

## Sending Goals

### Basic Goal Sending

```cpp
void send_goal()
{
    // Wait for server to be available
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    // Create goal message
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    // Send goal (basic - no callbacks)
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg);
}
```

### Goal with Callbacks

```cpp
void send_goal_with_callbacks()
{
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available");
        return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    // Configure send goal options
    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

    // Goal response callback (accepted/rejected)
    send_goal_options.goal_response_callback =
        std::bind(&FibonacciActionClient::goal_response_callback, this, std::placeholders::_1);

    // Feedback callback (progress updates)
    send_goal_options.feedback_callback =
        std::bind(&FibonacciActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    // Result callback (final outcome)
    send_goal_options.result_callback =
        std::bind(&FibonacciActionClient::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}
```

## The Three Callbacks

### 1. Goal Response Callback

Called when server accepts or rejects the goal:

```cpp
void goal_response_callback(
    const GoalHandleFibonacci::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
}
```

### 2. Feedback Callback

Called periodically with progress updates:

```cpp
void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
    std::stringstream ss;
    ss << "Received feedback: ";
    for (auto number : feedback->partial_sequence) {
        ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}
```

### 3. Result Callback

Called once when task completes:

```cpp
void result_callback(const GoalHandleFibonacci::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }

    std::stringstream ss;
    ss << "Result: ";
    for (auto number : result.result->sequence) {
        ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}
```

## Complete Action Client Example

```cpp
/**
 * @file fibonacci_action_client.cpp
 * @brief Action client that sends Fibonacci goals
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_tutorials_interfaces/action/fibonacci.hpp"

using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

class FibonacciActionClient : public rclcpp::Node
{
public:
    FibonacciActionClient() : Node("fibonacci_action_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

        // Send goal after short delay
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&FibonacciActionClient::send_goal, this));
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_goal()
    {
        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&FibonacciActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&FibonacciActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&FibonacciActionClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Feedback: ";
        for (auto number : feedback->partial_sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result");
                return;
        }

        std::stringstream ss;
        ss << "Result: ";
        for (auto number : result.result->sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Canceling Goals

```cpp
void send_goal_and_cancel()
{
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 20;

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

    // Store goal handle for later cancellation
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

    // Wait for goal handle
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
        return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
        return;
    }

    // Cancel after 2 seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(this->get_logger(), "Canceling goal");
    auto cancel_future = this->client_ptr_->async_cancel_goal(goal_handle);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal canceled");
}
```

## Python Comparison

**Python async/await**:
```python
async def call_service_async():
    response = await some_long_task()
    print(f"Result: {response}")

# Using callbacks
some_long_task(callback=lambda x: print(f"Progress: {x}"))
```

**ROS2 Action Client**:
```cpp
// Similar pattern: async call with callbacks
auto future = client->async_send_goal(goal, options);
// Feedback callback handles progress
// Result callback handles completion
```

## Real-World Example: Navigation Client

```cpp
class NavigationClient : public rclcpp::Node
{
public:
    NavigationClient() : Node("navigation_client")
    {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    }

    void navigate_to(double x, double y)
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Navigation server not available");
            return;
        }

        auto goal = NavigateToPose::Goal();
        goal.pose.position.x = x;
        goal.pose.position.y = y;

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        // Monitor distance remaining
        options.feedback_callback = [this](auto, const auto & feedback) {
            RCLCPP_INFO(this->get_logger(),
                       "Distance remaining: %.2f m (%.0f%% complete)",
                       feedback->distance_remaining,
                       feedback->percent_complete);
        };

        // Handle completion
        options.result_callback = [this](const auto & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Reached destination!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Navigation failed");
            }
        };

        client_->async_send_goal(goal, options);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};
```

## Common Pitfalls

### 1. Not Waiting for Server

```cpp
// BAD: Server might not be ready
client_->async_send_goal(goal);  // Fails if server not available
```

```cpp
// GOOD: Wait for server first
if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(logger, "Server not available");
    return;
}
client_->async_send_goal(goal);
```

### 2. Ignoring Goal Rejection

```cpp
// BAD: Assumes goal always accepted
auto future = client_->async_send_goal(goal);
// What if server rejected it?
```

```cpp
// GOOD: Check goal_handle
auto goal_handle = future.get();
if (!goal_handle) {
    RCLCPP_ERROR(logger, "Goal rejected");
    return;
}
```

### 3. Blocking Main Thread

```cpp
// BAD: Blocks node
auto future = client_->async_send_goal(goal);
auto result = future.get();  // BLOCKS until complete!
```

```cpp
// GOOD: Use callbacks
auto options = SendGoalOptions();
options.result_callback = [](auto result) { /* handle */ };
client_->async_send_goal(goal, options);
// Node continues running
```

## Best Practices

1. **Always Check Server Availability**
   ```cpp
   if (!client->wait_for_action_server(timeout)) {
       // Handle unavailable server
   }
   ```

2. **Provide All Callbacks**
   - Goal response: Know if accepted
   - Feedback: Monitor progress
   - Result: Handle completion

3. **Handle All Result Codes**
   ```cpp
   switch (result.code) {
       case SUCCEEDED: /* ... */ break;
       case ABORTED: /* ... */ break;
       case CANCELED: /* ... */ break;
   }
   ```

4. **Use Reasonable Timeouts**
   - Server availability: 5-10 seconds
   - Goal execution: Task-dependent
   - Cancellation: 1-2 seconds

5. **Cancel Gracefully**
   - Store goal_handle for cancellation
   - Check cancel result
   - Handle partial results

## Testing Your Client

```bash
# Terminal 1: Run server
ros2 run action_tutorials_cpp fibonacci_action_server

# Terminal 2: Run client
ros2 run action_tutorials_cpp fibonacci_action_client

# Expected output:
# [fibonacci_action_client]: Sending goal
# [fibonacci_action_client]: Goal accepted
# [fibonacci_action_client]: Feedback: 0 1 1 2 3 5...
# [fibonacci_action_client]: Result: 0 1 1 2 3 5 8 13 21 34 55
```

## Summary

Action clients send goals and process results:

**Key Components**:
- **Client creation**: `create_client<ActionType>(node, "action_name")`
- **Goal sending**: `async_send_goal(goal_msg, options)`
- **Three callbacks**: Goal response, feedback, result

**Workflow**:
1. Wait for server
2. Create goal message
3. Configure callbacks
4. Send goal
5. Monitor via feedback
6. Handle result

**Result Codes**:
- `SUCCEEDED`: Task completed successfully
- `ABORTED`: Server encountered error
- `CANCELED`: Goal was cancelled

## What's Next?

You now know how to create both action servers and clients! Next, you'll learn to define your own custom actions for your specific robot tasks.

- **Lesson 4**: Custom Action Definitions - Create `.action` files for your domain

## Further Reading

- [ROS2 Action Client Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [rclcpp_action Documentation](https://docs.ros2.org/latest/api/rclcpp_action/)

---

**Exercise**: Modify the client to send multiple goals sequentially (send goal 2 only after goal 1 completes).

**Next Lesson**: [Lesson 4: Custom Action Definitions](04-custom-action-definitions.md)
