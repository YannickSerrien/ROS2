# Lesson 1: Actions Overview

## Learning Objectives

By the end of this lesson, you will be able to:
- Explain what actions are and when to use them
- Understand the three-part structure of actions (goal, feedback, result)
- Compare actions to topics and services
- Identify appropriate use cases for actions in robotics
- Understand the communication flow between action clients and servers

## Introduction

Imagine you're building a robot that needs to navigate to a specific location. This task takes 30 seconds. Using a service, the client would be blocked for 30 seconds with no updates. What if you want to know "I'm 50% there" or cancel if an obstacle appears?

**Actions** solve this problem. They're designed for long-running, goal-oriented tasks that provide progress feedback and support cancellation.

## What Are Actions?

Actions are a communication pattern in ROS2 for tasks that:
1. **Take time** (seconds to minutes)
2. **Provide updates** during execution
3. **Can be cancelled** before completion
4. **Return a final result** when done

Think of actions as "smart services" with progress tracking.

## Python Comparison

**Python async/await**:
```python
async def long_task():
    for i in range(100):
        await asyncio.sleep(0.1)
        yield i  # Progress update
    return "Complete"

# Using it:
async for progress in long_task():
    print(f"Progress: {progress}%")
```

**ROS2 Action**:
- Goal = function call with parameters
- Feedback = `yield` statements (progress updates)
- Result = `return` value
- Cancellation = ability to stop mid-execution

## Actions vs Topics vs Services

| Feature | Topic | Service | Action |
|---------|-------|---------|--------|
| **Direction** | One-to-many | One-to-one | One-to-one |
| **Duration** | Continuous stream | Quick (<1 second) | Long-running (seconds+) |
| **Feedback** | N/A | N/A | Yes (during execution) |
| **Cancellation** | N/A | No | Yes |
| **Blocking** | Non-blocking | Blocking | Async with monitoring |
| **Use Case** | Sensor data | Quick queries | Goal-oriented tasks |

**Decision Tree**:
```
Is it continuous data? (e.g., sensor stream)
├─ YES → Use Topic
└─ NO → Does it take time?
    ├─ NO (<1 sec) → Use Service
    └─ YES (>1 sec) → Use Action
```

## The Three Parts of an Action

### 1. Goal
What you want to achieve.

**Example**: "Navigate to position (x=10, y=5)"

```cpp
// Goal message contains target parameters
NavigateToPose::Goal goal_msg;
goal_msg.pose.position.x = 10.0;
goal_msg.pose.position.y = 5.0;
```

### 2. Feedback
Progress updates during execution.

**Example**: "Currently at (x=7, y=3), 60% complete"

```cpp
// Feedback message shows current state
auto feedback = std::make_shared<NavigateToPose::Feedback>();
feedback->current_pose.position.x = 7.0;
feedback->current_pose.position.y = 3.0;
feedback->percent_complete = 60.0;
```

### 3. Result
Final outcome when task completes (or is cancelled).

**Example**: "Successfully reached goal" or "Cancelled by user"

```cpp
// Result message contains final status
auto result = std::make_shared<NavigateToPose::Result>();
result->success = true;
result->message = "Reached goal position";
```

## Action Communication Flow

```
Client                           Server
  |                                 |
  |-------- Send Goal ------------->|
  |                                 |--- Execute
  |<------ Feedback Update ---------|--- (running)
  |<------ Feedback Update ---------|--- (running)
  |                                 |
  |-------- Cancel (optional) ----->|
  |                                 |--- Stop
  |<------ Result ------------------|
  |                                 |
```

**Key Points**:
- Client sends goal, server starts executing
- Server periodically sends feedback (progress updates)
- Client can monitor, wait, or cancel
- Server eventually sends result (success/failure/cancelled)

## ROS2 Context: Where Actions Are Used

### Built-in ROS2 Actions

1. **Navigation2**: `NavigateToPose`
   - Goal: Target pose
   - Feedback: Current pose, distance remaining
   - Result: Success or failure reason

2. **MoveIt2**: `MoveGroup`
   - Goal: Target joint angles or end-effector pose
   - Feedback: Current joint states
   - Result: Trajectory execution success

3. **Perception**: `DetectObjects`
   - Goal: Image or point cloud
   - Feedback: Number of candidates processed
   - Result: Detected objects list

### Custom Action Examples

**File Processing**:
```
Goal: Process 1000 files
Feedback: Files processed (1/1000, 2/1000, ...)
Result: Summary statistics
```

**Robot Charging**:
```
Goal: Charge to 100%
Feedback: Current battery level (50%, 75%, ...)
Result: Charged successfully or timeout
```

**Image Analysis**:
```
Goal: Analyze image for defects
Feedback: Regions scanned (25%, 50%, ...)
Result: Defect locations and confidence
```

## Action Lifecycle States

An action goal goes through these states:

1. **PENDING**: Received by server, not yet executing
2. **ACTIVE**: Currently executing
3. **CANCELING**: Cancel requested, wrapping up
4. **SUCCEEDED**: Completed successfully
5. **ABORTED**: Failed during execution
6. **CANCELED**: Successfully cancelled

```cpp
// In action server callback:
if (goal_handle->is_canceling()) {
    // Handle cancellation
    result->success = false;
    goal_handle->canceled(result);
    return;
}

if (error_occurred) {
    // Handle failure
    goal_handle->abort(result);
    return;
}

// Success
goal_handle->succeed(result);
```

## Code Example: Fibonacci Action

A simple action that generates Fibonacci sequence:

**Action Definition** (`Fibonacci.action`):
```
# Goal: how many numbers to generate
int32 order
---
# Result: the final sequence
int32[] sequence
---
# Feedback: current progress
int32[] partial_sequence
```

**Server (Conceptual)**:
```cpp
void execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();

    std::vector<int32_t> sequence = {0, 1};

    for (int i = 1; i < goal->order; ++i) {
        // Check for cancel
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }

        // Calculate next Fibonacci number
        sequence.push_back(sequence[i] + sequence[i-1]);

        // Send feedback
        feedback->partial_sequence = sequence;
        goal_handle->publish_feedback(feedback);

        // Simulate work
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Success
    result->sequence = sequence;
    goal_handle->succeed(result);
}
```

**Client (Conceptual)**:
```cpp
// Send goal
auto goal_msg = Fibonacci::Goal();
goal_msg.order = 10;

auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

// Feedback callback
send_goal_options.feedback_callback =
    [](auto, const std::shared_ptr<const Fibonacci::Feedback> feedback) {
        std::cout << "Current sequence: ";
        for (auto num : feedback->partial_sequence) {
            std::cout << num << " ";
        }
        std::cout << std::endl;
    };

// Send goal
auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
```

## When to Use Actions

**Use Actions When**:
- ✓ Task duration > 1 second
- ✓ Need progress updates
- ✓ User might want to cancel
- ✓ Task has clear goal and result
- ✓ Examples: Navigation, manipulation, processing

**Use Services When**:
- ✓ Quick request-response (<1 sec)
- ✓ No progress updates needed
- ✓ Cancellation not required
- ✓ Examples: Get parameter, compute sum, query database

**Use Topics When**:
- ✓ Continuous data stream
- ✓ One-to-many broadcasting
- ✓ No response needed
- ✓ Examples: Sensor readings, robot state, camera feed

## Common Pitfalls

### 1. Using Service Instead of Action
```cpp
// BAD: Service for long task blocks client
auto result = move_robot_service_->call(request);  // Blocked for 30 sec
// Can't monitor progress, can't cancel
```

```cpp
// GOOD: Action provides feedback and cancellation
auto goal_future = move_robot_action_->async_send_goal(goal);
// Can monitor via feedback callback, cancel if needed
```

### 2. Not Handling Cancellation
```cpp
// BAD: Ignores cancel requests
void execute(goal_handle) {
    for (int i = 0; i < 1000; ++i) {
        do_work();  // No cancel check
    }
}
```

```cpp
// GOOD: Checks for cancellation
void execute(goal_handle) {
    for (int i = 0; i < 1000; ++i) {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }
        do_work();
    }
}
```

### 3. Forgetting Feedback
```cpp
// BAD: No feedback, client has no idea of progress
void execute(goal_handle) {
    do_long_task();  // Silent for 60 seconds
    goal_handle->succeed(result);
}
```

```cpp
// GOOD: Regular feedback updates
void execute(goal_handle) {
    for (int i = 0; i < 100; ++i) {
        do_step();
        feedback->percent_complete = i;
        goal_handle->publish_feedback(feedback);
    }
    goal_handle->succeed(result);
}
```

## Best Practices

1. **Provide Regular Feedback**
   - Update at least every 0.5-1 second
   - Include meaningful progress information
   - Don't spam (too frequent = network overhead)

2. **Handle Cancellation Gracefully**
   - Check `is_canceling()` in loops
   - Clean up resources before returning
   - Return quickly after cancel detected

3. **Design Informative Messages**
   - Goal: Clear parameters for task
   - Feedback: Progress metrics (%, current state)
   - Result: Success status + detailed information

4. **Use Appropriate Timeouts**
   - Set realistic execution time limits
   - Abort if task takes too long
   - Consider edge cases (worst-case duration)

5. **Thread Safety**
   - Action callbacks run in separate threads
   - Protect shared data with mutexes
   - Be careful with member variable access

## Summary

Actions are ROS2's solution for long-running, goal-oriented tasks:

**Key Takeaways**:
- Actions = Goal + Feedback + Result
- Use for tasks taking >1 second
- Provide progress updates during execution
- Support cancellation
- Replace blocking services for long operations

**Three-Part Structure**:
1. **Goal**: What to do (client → server)
2. **Feedback**: Progress updates (server → client)
3. **Result**: Final outcome (server → client)

**Decision Guide**:
- Continuous data? → **Topic**
- Quick query? → **Service**
- Long task with progress? → **Action**

## What's Next?

Now that you understand what actions are and when to use them, the next lessons will show you how to implement them:

- **Lesson 2**: Build action servers that execute goals
- **Lesson 3**: Create action clients that send goals and monitor progress
- **Lesson 4**: Define custom action interfaces for your specific needs

In the next lesson, you'll write your first action server!

## Further Reading

- [ROS2 Actions Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [Action Design Patterns](https://design.ros2.org/articles/actions.html)
- [Navigation2 Actions](https://navigation.ros.org/plugins/index.html)

---

**Practice**: Before moving on, try to identify 3 tasks in a robot system (any robot) that would be good candidates for actions. Why would actions be better than services for each?

**Next Lesson**: [Lesson 2: Action Servers](02-action-servers.md)
