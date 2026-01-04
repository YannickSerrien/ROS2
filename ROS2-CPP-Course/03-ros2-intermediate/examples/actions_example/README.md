# Actions Example

Complete demonstration of ROS2 actions with servers, clients, and custom action definitions.

## What's Included

This package demonstrates ROS2 actions through three complete examples:

### 1. Fibonacci Action (Basic)
- **Server**: `fibonacci_server` - Computes Fibonacci sequences
- **Client**: `fibonacci_client` - Requests sequences with feedback
- **Action**: `Fibonacci.action` - Simple action definition

**Demonstrates**:
- Goal handling (accept/reject)
- Periodic feedback publishing
- Cancellation handling
- Final result returning

### 2. Robot Movement (Advanced)
- **Server**: `task_manager` - Simulates robot movement
- **Action**: `MoveRobot.action` - Complex action with multiple fields

**Demonstrates**:
- Realistic robotics scenario
- Progress calculation
- State management
- Input validation
- Detailed feedback

## Building

```bash
# From workspace root
colcon build --packages-select actions_example
source install/setup.bash
```

## Running the Examples

### Example 1: Fibonacci Server and Client

**Terminal 1** - Start the action server:
```bash
ros2 run actions_example fibonacci_server
```

**Terminal 2** - Start the action client:
```bash
ros2 run actions_example fibonacci_client
```

**Expected Output**:
- Server accepts goal and computes Fibonacci sequence
- Client receives periodic feedback showing partial sequences
- Final result shows complete sequence

### Example 2: Task Manager (Robot Movement)

**Terminal 1** - Start the task manager server:
```bash
ros2 run actions_example task_manager
```

**Terminal 2** - Send a goal via command line:
```bash
ros2 action send_goal /move_robot actions_example/action/MoveRobot "{target_x: 10.0, target_y: 5.0, target_theta: 1.57, speed: 1.0}"
```

**Expected Output**:
- Server simulates robot moving to target position
- Periodic feedback shows current position and progress percentage
- Final result indicates success and total distance traveled

## Testing Cancellation

**Terminal 1** - Start server:
```bash
ros2 run actions_example task_manager
```

**Terminal 2** - Send goal:
```bash
ros2 action send_goal /move_robot actions_example/action/MoveRobot "{target_x: 50.0, target_y: 50.0, target_theta: 0.0, speed: 0.5}"
```

**Terminal 3** - Cancel while in progress:
```bash
# Get goal ID from Terminal 2 output, then:
ros2 action cancel_goal /move_robot <goal_id>
```

## Monitoring Actions

### List available actions:
```bash
ros2 action list
```

**Output**:
```
/fibonacci
/move_robot
```

### Show action info:
```bash
ros2 action info /fibonacci
```

### View action type:
```bash
ros2 action type /fibonacci
```

**Output**: `actions_example/action/Fibonacci`

### Show action interface:
```bash
ros2 interface show actions_example/action/Fibonacci
```

## Action Definitions

### Fibonacci.action
```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

**Usage**: Request Fibonacci sequence up to specified order.

### MoveRobot.action
```
# Goal
float64 target_x
float64 target_y
float64 target_theta
float64 speed
---
# Result
bool success
string message
float64 final_x
float64 final_y
float64 final_theta
float64 total_distance
---
# Feedback
float64 current_x
float64 current_y
float64 current_theta
float64 distance_remaining
float64 percent_complete
```

**Usage**: Command robot to move to target position at specified speed.

## Key Concepts Demonstrated

### Action Server
- Creating action servers with `rclcpp_action::create_server`
- Three callbacks: `handle_goal`, `handle_cancel`, `handle_accepted`
- Executing goals in separate threads (non-blocking)
- Publishing feedback during execution
- Returning results (succeed/abort/cancel)

### Action Client
- Creating action clients with `rclcpp_action::create_client`
- Sending goals asynchronously
- Configuring callbacks for goal response, feedback, and result
- Canceling active goals

### Custom Actions
- Defining `.action` files with Goal/Result/Feedback structure
- Building custom interfaces with `rosidl_generate_interfaces`
- Linking generated code with `rosidl_get_typesupport_target`

## Code Highlights

### Server Goal Validation
```cpp
rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal)
{
    // Validate input
    if (goal->speed <= 0.0) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```

### Publishing Feedback
```cpp
void execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    auto feedback = std::make_shared<Feedback>();

    while (/* still executing */) {
        // Update feedback
        feedback->percent_complete = ...;
        goal_handle->publish_feedback(feedback);

        // Check for cancellation
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }
    }

    goal_handle->succeed(result);
}
```

### Client with Callbacks
```cpp
auto send_goal_options = Client::SendGoalOptions();
send_goal_options.goal_response_callback = /* ... */;
send_goal_options.feedback_callback = /* ... */;
send_goal_options.result_callback = /* ... */;

client_->async_send_goal(goal_msg, send_goal_options);
```

## Learning Path

1. **Start with Fibonacci** - Understand basic action concepts
2. **Explore Task Manager** - See realistic robotics application
3. **Modify Parameters** - Change Fibonacci order, robot speed
4. **Create Custom Action** - Define your own action type
5. **Implement New Server** - Build action server for different task

## Common Use Cases

Actions are ideal for:
- **Navigation**: Move robot to target pose
- **Manipulation**: Pick and place operations
- **Perception**: Long-running image processing
- **Mapping**: Build map of environment
- **Calibration**: Sensor/actuator calibration routines

## Related Lessons

- **Lesson 1**: Actions Overview
- **Lesson 2**: Action Servers
- **Lesson 3**: Action Clients
- **Lesson 4**: Custom Action Definitions

## Troubleshooting

**Server not responding**:
```bash
# Check if server is running
ros2 node list

# Check action server availability
ros2 action list
```

**Goal rejected**:
- Check server logs for rejection reason
- Verify goal parameters meet validation requirements

**Build errors**:
- Ensure `rosidl_default_generators` is in package.xml
- Check action file syntax (Goal/Result/Feedback structure)
- Verify `rosidl_generate_interfaces` in CMakeLists.txt

## Next Steps

- Complete Module 3 exercises on actions
- Combine with TF2 for coordinate-aware movement
- Use launch files to start multiple action servers
- Integrate actions into mini-project robot arm
