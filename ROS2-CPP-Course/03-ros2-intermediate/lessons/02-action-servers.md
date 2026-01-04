# Lesson 2: Action Servers

## Learning Objectives

By the end of this lesson, you will be able to:
- Create a ROS2 action server in C++
- Handle goal requests from clients
- Publish feedback during execution
- Return results upon completion or cancellation
- Implement proper error handling and cancellation logic
- Understand action server threading model

## Introduction

In the previous lesson, you learned what actions are. Now you'll build the server side—the component that receives goals, executes them, provides feedback, and returns results.

An action server is like a worker that:
1. Accepts job requests (goals)
2. Reports progress (feedback)
3. Delivers final results
4. Can stop early if requested (cancellation)

## Action Server Components

### Required Includes

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "my_package/action/fibonacci.hpp"  // Your action definition

using Fibonacci = my_package::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
```

### Node with Action Server

```cpp
class FibonacciActionServer : public rclcpp::Node
{
public:
    FibonacciActionServer() : Node("fibonacci_action_server")
    {
        using namespace std::placeholders;

        // Create action server
        this->action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",  // Action name
            std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
            std::bind(&FibonacciActionServer::handle_cancel, this, _1),
            std::bind(&FibonacciActionServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Fibonacci action server ready");
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    // Callbacks defined below...
};
```

## The Three Callbacks

### 1. Handle Goal (Accept or Reject)

```cpp
rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
{
    (void)uuid;  // Unused in simple cases

    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);

    // Validation: Reject invalid goals
    if (goal->order < 0) {
        RCLCPP_WARN(this->get_logger(), "Rejecting negative order");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->order > 100) {
        RCLCPP_WARN(this->get_logger(), "Rejecting order > 100 (too large)");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Accept goal
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```

**Python Comparison**:
```python
# Like a validation function before accepting a task
def validate_request(request):
    if not request.is_valid():
        return "REJECT"
    return "ACCEPT"
```

### 2. Handle Cancel (Allow or Deny Cancellation)

```cpp
rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    // Usually accept all cancel requests
    return rclcpp_action::CancelResponse::ACCEPT;

    // Could reject if in critical section:
    // if (in_critical_operation) {
    //     return rclcpp_action::CancelResponse::REJECT;
    // }
}
```

### 3. Handle Accepted (Start Execution)

This is where the actual work happens:

```cpp
void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    // Execute in separate thread to avoid blocking
    std::thread{
        std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1),
        goal_handle
    }.detach();
}
```

**Important**: Execution must happen in a separate thread, otherwise the node blocks!

## The Execute Function (Core Logic)

```cpp
void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();

    // Initialize sequence
    std::vector<int32_t> sequence = {0, 1};

    // Execution loop
    for (int i = 1; i < goal->order; ++i) {
        // Check for cancellation
        if (goal_handle->is_canceling()) {
            result->sequence = sequence;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        // Compute next Fibonacci number
        sequence.push_back(sequence[i] + sequence[i - 1]);

        // Publish feedback
        feedback->partial_sequence = sequence;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publishing feedback");

        // Simulate work (remove in real applications)
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Check one more time before succeeding
    if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }

    // Success!
    result->sequence = sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}
```

## Complete Example: Fibonacci Action Server

```cpp
/**
 * @file fibonacci_action_server.cpp
 * @brief Action server that generates Fibonacci sequences
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include <thread>

using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FibonacciActionServer : public rclcpp::Node
{
public:
    FibonacciActionServer() : Node("fibonacci_action_server")
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
            std::bind(&FibonacciActionServer::handle_cancel, this, _1),
            std::bind(&FibonacciActionServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Fibonacci action server started");
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);

        if (goal->order < 0 || goal->order > 100) {
            RCLCPP_WARN(this->get_logger(), "Rejecting invalid order");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        // Execute in separate thread
        std::thread{
            std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1),
            goal_handle
        }.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto result = std::make_shared<Fibonacci::Result>();

        std::vector<int32_t> sequence = {0, 1};

        for (int i = 1; i < goal->order; ++i) {
            if (goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            sequence.push_back(sequence[i] + sequence[i - 1]);

            feedback->partial_sequence = sequence;
            goal_handle->publish_feedback(feedback);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        if (goal_handle->is_canceling()) {
            result->sequence = sequence;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Goal States and Transitions

```
Goal Lifecycle:
PENDING → ACTIVE → SUCCEEDED
                 → ABORTED
                 → CANCELED

From ACTIVE:
- goal_handle->succeed(result)  → SUCCEEDED
- goal_handle->abort(result)    → ABORTED
- goal_handle->canceled(result) → CANCELED
```

## ROS2 Context: Real-World Example

**Robot Navigation Server**:

```cpp
void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Navigate::Feedback>();
    auto result = std::make_shared<Navigate::Result>();

    double start_x = current_pose_.x;
    double start_y = current_pose_.y;
    double target_x = goal->target_pose.x;
    double target_y = goal->target_pose.y;

    while (!reached_goal(target_x, target_y)) {
        // Check cancellation
        if (goal_handle->is_canceling()) {
            stop_motors();
            result->success = false;
            result->message = "Navigation cancelled";
            goal_handle->canceled(result);
            return;
        }

        // Move towards goal
        move_towards_target(target_x, target_y);

        // Calculate and publish feedback
        feedback->current_pose = current_pose_;
        feedback->distance_remaining = calculate_distance(current_pose_, goal->target_pose);
        feedback->percent_complete = calculate_progress(start_x, start_y, target_x, target_y);
        goal_handle->publish_feedback(feedback);

        // Control loop rate
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Success
    result->success = true;
    result->message = "Reached goal";
    goal_handle->succeed(result);
}
```

## Common Pitfalls

### 1. Executing in Main Thread (Blocks Node)

```cpp
// BAD: Blocks the node, can't process new goals
void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    execute(goal_handle);  // BLOCKS!
}
```

```cpp
// GOOD: Executes in separate thread
void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    std::thread{std::bind(&Server::execute, this, _1), goal_handle}.detach();
}
```

### 2. Not Checking for Cancellation

```cpp
// BAD: Ignores cancel requests
void execute(goal_handle) {
    for (int i = 0; i < 1000; ++i) {
        do_work();  // What if user cancelled?
    }
    goal_handle->succeed(result);
}
```

```cpp
// GOOD: Checks regularly
void execute(goal_handle) {
    for (int i = 0; i < 1000; ++i) {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }
        do_work();
    }
    goal_handle->succeed(result);
}
```

### 3. Forgetting to Call Final State

```cpp
// BAD: Goal never completes
void execute(goal_handle) {
    do_work();
    // Forgot to call succeed(), abort(), or canceled()!
}
```

```cpp
// GOOD: Always reaches final state
void execute(goal_handle) {
    if (error) {
        goal_handle->abort(result);
    } else if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}
```

## Best Practices

1. **Always Validate Goals**
   - Check parameters in `handle_goal()`
   - Reject invalid requests early
   - Provide clear rejection reasons in logs

2. **Provide Regular Feedback**
   - Update every 0.5-1 seconds
   - Include meaningful progress metrics
   - Don't spam (too frequent wastes bandwidth)

3. **Handle Cancellation Promptly**
   - Check `is_canceling()` in every loop iteration
   - Clean up resources before returning
   - Call `canceled()` within 1-2 seconds of request

4. **Thread Safety**
   - Action callbacks run in separate threads
   - Protect shared member variables with `std::mutex`
   - Be careful with concurrent access

5. **Error Handling**
   - Use `abort()` for failures during execution
   - Include error details in result message
   - Log errors for debugging

## Testing Your Action Server

### Using Command Line

```bash
# Terminal 1: Run server
ros2 run my_package fibonacci_action_server

# Terminal 2: Send goal with CLI
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

# Terminal 3: Monitor feedback
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 10}" --feedback

# Cancel an active goal
# (Press Ctrl+C in the terminal that sent the goal)
```

### Introspection

```bash
# List available actions
ros2 action list

# Get action info
ros2 action info /fibonacci

# Show action definition
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```

## Summary

You've learned how to create action servers:

**Key Components**:
1. **handle_goal()** - Accept or reject incoming goals
2. **handle_cancel()** - Accept or reject cancellation requests
3. **handle_accepted()** - Start execution in separate thread
4. **execute()** - Perform work, send feedback, return result

**Essential Patterns**:
- Execute in separate thread (don't block node)
- Check `is_canceling()` regularly
- Publish feedback for progress updates
- Always call final state: `succeed()`, `abort()`, or `canceled()`

**Goal States**:
- PENDING → ACTIVE → SUCCEEDED/ABORTED/CANCELED

## What's Next?

Now that you can create action servers, the next lesson covers the client side:

- **Lesson 3**: Build action clients that send goals and process feedback
- Send goals with various options
- Handle feedback and results
- Cancel goals programmatically

## Further Reading

- [ROS2 Action Server Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [rclcpp_action API Reference](https://docs.ros2.org/latest/api/rclcpp_action/)

---

**Exercise**: Modify the Fibonacci server to reject goals where order > 50 and add a timeout that aborts if execution takes longer than 60 seconds.

**Next Lesson**: [Lesson 3: Action Clients](03-action-clients.md)
