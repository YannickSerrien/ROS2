# Exercise 1: Action Server - Countdown Timer

## Objective

Build an action server and client for a countdown timer that provides periodic feedback and can be canceled.

## Learning Goals

- Create custom action definitions
- Implement action server with goal handling
- Publish periodic feedback
- Handle cancellation requests
- Build action client with callbacks

## Background

Actions are perfect for long-running tasks that need progress updates. A countdown timer is an ideal scenario:
- **Goal**: How long to count down (seconds)
- **Feedback**: Current remaining time
- **Result**: Whether countdown completed or was canceled

## Task Description

Create a countdown action system with:

1. **Custom Action** (`Countdown.action`):
   - Goal: `int32 duration` (seconds to count down)
   - Feedback: `int32 time_remaining` (seconds left)
   - Result: `bool completed`, `string message`

2. **Action Server** (`countdown_server`):
   - Accept countdown goal
   - Reject if duration <= 0 or > 300 (5 minutes)
   - Publish feedback every 1 second
   - Handle cancellation
   - Return success when countdown reaches 0

3. **Action Client** (`countdown_client`):
   - Send goal (e.g., 10 seconds)
   - Display feedback updates
   - Handle goal acceptance/rejection
   - Process final result

## Requirements

### Action Definition

Create `action/Countdown.action`:
```
# Goal: Duration in seconds
int32 duration
---
# Result: Success status and message
bool completed
string message
int32 elapsed_time
---
# Feedback: Time remaining
int32 time_remaining
```

### Server Requirements

1. Validate goal (1-300 seconds)
2. Publish feedback every 1 second
3. Check for cancellation each iteration
4. Log current countdown value
5. Return appropriate result

### Client Requirements

1. Send goal with configurable duration
2. Handle goal response (accepted/rejected)
3. Print feedback updates
4. Display final result
5. Show elapsed time

## Starter Code

The starter package includes:
- Package structure (package.xml, CMakeLists.txt)
- Action definition file
- Node skeletons with TODO comments
- Build configuration

Your tasks:
1. Complete the action server implementation
2. Complete the action client implementation
3. Test with various countdown durations
4. Test cancellation

## Testing

### Test 1: Basic Countdown

**Terminal 1**:
```bash
ros2 run action_countdown countdown_server
```

**Terminal 2**:
```bash
ros2 run action_countdown countdown_client
```

**Expected**: 10-second countdown with feedback every second

### Test 2: Long Countdown

**Terminal 2**:
```bash
ros2 action send_goal /countdown action_countdown/action/Countdown "{duration: 30}"
```

**Expected**: 30-second countdown

### Test 3: Cancellation

**Terminal 2**: Start long countdown
```bash
ros2 action send_goal /countdown action_countdown/action/Countdown "{duration: 60}"
```

**Terminal 3**: Cancel while running
```bash
ros2 action cancel_goal /countdown <goal_id>
```

**Expected**: Server stops and returns canceled result

### Test 4: Invalid Goal

```bash
ros2 action send_goal /countdown action_countdown/action/Countdown "{duration: -5}"
```

**Expected**: Goal rejected

## Hints

<details>
<summary>Hint 1: Action Server Structure</summary>

```cpp
class CountdownServer : public rclcpp::Node
{
    rclcpp_action::Server<Countdown>::SharedPtr action_server_;

    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        // Loop from duration down to 0
        for (int i = duration; i > 0; --i) {
            // Check cancellation
            // Publish feedback
            // Sleep 1 second
        }
        // Succeed
    }
};
```
</details>

<details>
<summary>Hint 2: Publishing Feedback</summary>

```cpp
auto feedback = std::make_shared<Countdown::Feedback>();
feedback->time_remaining = remaining;
goal_handle->publish_feedback(feedback);
```
</details>

<details>
<summary>Hint 3: Checking Cancellation</summary>

```cpp
if (goal_handle->is_canceling()) {
    result->completed = false;
    result->message = "Countdown canceled";
    goal_handle->canceled(result);
    return;
}
```
</details>

<details>
<summary>Hint 4: Client Callbacks</summary>

```cpp
auto send_goal_options = Client::SendGoalOptions();
send_goal_options.feedback_callback =
    [](auto, const auto feedback) {
        RCLCPP_INFO(logger, "Time remaining: %d", feedback->time_remaining);
    };
```
</details>

## Bonus Challenges

1. **Parameter Configuration**: Make countdown client duration configurable via parameter
2. **Multiple Clients**: Test multiple simultaneous countdown requests
3. **Pause/Resume**: Add ability to pause and resume countdown (advanced)
4. **Visual Display**: Show progress bar in terminal

## Success Criteria

- [ ] Custom action definition compiles
- [ ] Server accepts valid goals (1-300 seconds)
- [ ] Server rejects invalid goals
- [ ] Feedback published every second
- [ ] Countdown reaches zero successfully
- [ ] Cancellation works correctly
- [ ] Client displays all feedback
- [ ] Client handles all result codes

## Common Issues

**Build Error**: "Could not find action 'Countdown'"
- Ensure action file in `action/` directory
- Check `rosidl_generate_interfaces` in CMakeLists.txt
- Rebuild package

**Server Not Responding**:
- Check action name matches (`/countdown`)
- Verify server is running (`ros2 action list`)

**No Feedback**:
- Ensure `publish_feedback` called in loop
- Check client has feedback callback

## Related Lessons

- Lesson 1: Actions Overview
- Lesson 2: Action Servers
- Lesson 3: Action Clients
- Lesson 4: Custom Action Definitions

## Next Steps

After completing this exercise:
- Complete Exercise 2 (TF2 + Launch)
- Review actions_example package
- Integrate actions into mini-project
