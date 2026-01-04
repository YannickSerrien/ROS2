# Lesson 4: Custom Action Definitions

## Learning Objectives

By the end of this lesson, you will be able to:
- Create custom `.action` files for your specific needs
- Understand the three-part structure of action definitions
- Choose appropriate message types for goals, feedback, and results
- Build and use custom actions in your packages
- Follow naming and design best practices for actions

## Introduction

The Fibonacci action was useful for learning, but real robots need domain-specific actions like:
- `MoveToPosition` for navigation
- `GraspObject` for manipulation
- `ProcessImage` for vision
- `ChargeB

attery` for power management

**Custom actions** let you define exactly what your robot needs.

## Action File Structure

Action files (`.action`) have three sections separated by `---`:

```
# Request (Goal)
<goal fields>
---
# Response (Result)
<result fields>
---
# Feedback
<feedback fields>
```

### Basic Example: Move to Position

```
# File: MoveToPosition.action

# Goal: Where to move
float64 target_x
float64 target_y
float64 max_speed
---
# Result: Final status
bool success
float64 final_x
float64 final_y
string message
---
# Feedback: Progress updates
float64 current_x
float64 current_y
float64 distance_remaining
float64 percent_complete
```

## Creating Custom Actions

### Step 1: Create action Directory

```bash
cd ~/ros2_ws/src/my_package
mkdir action
```

### Step 2: Define Action File

Create `action/ProcessImage.action`:

```
# Goal: Image to process and what to find
sensor_msgs/Image image
string[] object_types_to_detect
float32 confidence_threshold
---
# Result: What was found
string[] detected_objects
geometry_msgs/Point[] object_locations
float32[] confidences
bool success
string error_message
---
# Feedback: Processing progress
int32 regions_processed
int32 total_regions
float32 percent_complete
string current_stage
```

### Step 3: Update package.xml

Add rosidl dependencies:

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

<!-- Also add any message dependencies -->
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
```

### Step 4: Update CMakeLists.txt

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ProcessImage.action"
  DEPENDENCIES sensor_msgs geometry_msgs
)
```

### Step 5: Build

```bash
cd ~/ros2_ws
colcon build --packages-select my_package
source install/setup.bash
```

### Step 6: Verify

```bash
# Check action is available
ros2 interface list | grep ProcessImage

# Show action definition
ros2 interface show my_package/action/ProcessImage
```

## Design Patterns

### Navigation Action

```
# Goal: Where to navigate
geometry_msgs/PoseStamped target_pose
float32 tolerance  # How close is "reached"
---
# Result: How it went
bool reached_goal
geometry_msgs/Pose final_pose
float32 distance_from_target
float32 time_elapsed
string termination_reason
---
# Feedback: Current progress
geometry_msgs/Pose current_pose
float32 distance_remaining
float32 estimated_time_remaining
float32 percent_complete
int32 navigation_stage  # 0=planning, 1=executing, 2=recovering
```

### Manipulation Action

```
# Goal: Object to grasp
geometry_msgs/Pose object_pose
string grasp_type  # "top", "side", "pinch"
float32 force_limit
---
# Result: Success or failure
bool grasped
geometry_msgs/Pose final_gripper_pose
float32 force_applied
string failure_reason
---
# Feedback: Execution stages
string current_stage  # "approaching", "grasping", "lifting"
float32 distance_to_object
float32 gripper_opening
float32 percent_complete
```

### Data Processing Action

```
# Goal: Processing parameters
string input_file_path
string output_format  # "csv", "json", "xml"
int32 chunk_size
---
# Result: Processing outcome
bool success
string output_file_path
int64 records_processed
float64 processing_time_seconds
string error_details
---
# Feedback: Progress
int64 records_completed
int64 total_records
float32 percent_complete
float64 estimated_time_remaining
string current_operation
```

## Using Custom Actions in Code

### Server Example

```cpp
#include "my_package/action/process_image.hpp"

using ProcessImage = my_package::action::ProcessImage;

class ImageProcessorServer : public rclcpp::Node
{
public:
    ImageProcessorServer() : Node("image_processor_server")
    {
        action_server_ = rclcpp_action::create_server<ProcessImage>(
            this,
            "process_image",
            std::bind(&ImageProcessorServer::handle_goal, this, _1, _2),
            std::bind(&ImageProcessorServer::handle_cancel, this, _1),
            std::bind(&ImageProcessorServer::handle_accepted, this, _1));
    }

private:
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ProcessImage::Feedback>();
        auto result = std::make_shared<ProcessImage::Result>();

        // Access custom fields
        auto image = goal->image;
        auto threshold = goal->confidence_threshold;

        // Process in stages
        for (int region = 0; region < 100; ++region) {
            // Check cancellation
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->error_message = "Processing cancelled";
                goal_handle->canceled(result);
                return;
            }

            // Update feedback with custom fields
            feedback->regions_processed = region;
            feedback->total_regions = 100;
            feedback->percent_complete = (region / 100.0) * 100.0;
            feedback->current_stage = "object_detection";
            goal_handle->publish_feedback(feedback);

            // Simulate work
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Set custom result fields
        result->success = true;
        result->detected_objects = {"person", "car", "bicycle"};
        // ... set other fields
        goal_handle->succeed(result);
    }

    rclcpp_action::Server<ProcessImage>::SharedPtr action_server_;
};
```

### Client Example

```cpp
void send_processing_goal()
{
    auto goal = ProcessImage::Goal();

    // Set custom goal fields
    goal.image = current_image_;
    goal.object_types_to_detect = {"person", "vehicle"};
    goal.confidence_threshold = 0.8;

    auto options = SendGoalOptions();

    options.feedback_callback = [this](auto, const auto & feedback) {
        RCLCPP_INFO(this->get_logger(),
                   "Processing: %d/%d regions (%.1f%%) - Stage: %s",
                   feedback->regions_processed,
                   feedback->total_regions,
                   feedback->percent_complete,
                   feedback->current_stage.c_str());
    };

    options.result_callback = [this](const auto & result) {
        if (result.result->success) {
            RCLCPP_INFO(this->get_logger(), "Found %zu objects",
                       result.result->detected_objects.size());
            for (const auto & obj : result.result->detected_objects) {
                RCLCPP_INFO(this->get_logger(), "  - %s", obj.c_str());
            }
        }
    };

    client_->async_send_goal(goal, options);
}
```

## Field Type Guidelines

### Goal Fields

Choose types that fully specify the task:
```
# Good: Specific, complete
geometry_msgs/PoseStamped target
float32 speed_limit
string approach_direction

# Bad: Vague, incomplete
float32 x  # Missing y, z, orientation
int32 mode  # What do the numbers mean?
```

### Result Fields

Include both success status and details:
```
# Good: Complete information
bool success
string error_code  # For debugging
float64 actual_value
float64 time_taken

# Bad: Minimal info
bool done  # No details on failure
```

### Feedback Fields

Provide meaningful progress indicators:
```
# Good: Informative
float32 percent_complete
string current_stage
float32 estimated_time_remaining

# Bad: Uninformative
int32 step  # What does step number mean?
```

## Common Pitfalls

### 1. Forgetting Dependencies in CMakeLists.txt

```cmake
# BAD: Missing dependencies
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MyAction.action"
)
# Fails if MyAction uses geometry_msgs!

# GOOD: Include all dependencies
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MyAction.action"
  DEPENDENCIES geometry_msgs std_msgs
)
```

### 2. Using Non-Standard Message Types

```
# BAD: Custom struct (not supported)
MyCustomStruct data
---
# GOOD: Use ROS message types
geometry_msgs/Pose pose
```

### 3. Overly Complex Actions

```
# BAD: Too many fields, unclear purpose
int32 mode1
int32 mode2
float64[] data1
float64[] data2
string[] options
# ... 20 more fields

# GOOD: Focused, clear purpose
geometry_msgs/Pose target
float32 speed
string strategy
```

## Best Practices

1. **Use Existing Message Types**
   - Prefer `geometry_msgs`, `sensor_msgs`, `std_msgs`
   - Only create custom messages if truly needed

2. **Provide Clear Field Names**
   ```
   # Good
   float32 target_temperature_celsius

   # Bad
   float32 temp  # Ambiguous units, purpose
   ```

3. **Include Timestamps Where Relevant**
   ```
   builtin_interfaces/Time start_time
   builtin_interfaces/Time end_time
   ```

4. **Add Comments**
   ```
   float32 tolerance  # Maximum acceptable error in meters
   int32 max_attempts  # Retry limit before aborting
   ```

5. **Design for Extensibility**
   ```
   # Future-proof: can add modes without breaking
   string operation_mode  # "fast", "accurate", "balanced"

   # Rigid: new modes break existing code
   bool use_fast_mode
   bool use_accurate_mode
   ```

## Organizing Multiple Actions

For packages with many actions:

```
my_package/
├── action/
│   ├── Navigate.action
│   ├── Grasp.action
│   ├── Pick.action
│   └── Place.action
├── CMakeLists.txt
└── package.xml
```

In CMakeLists.txt:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Navigate.action"
  "action/Grasp.action"
  "action/Pick.action"
  "action/Place.action"
  DEPENDENCIES geometry_msgs sensor_msgs
)
```

## Summary

Custom actions let you define robot-specific behaviors:

**Key Steps**:
1. Create `.action` file with goal, result, feedback
2. Update `package.xml` with rosidl dependencies
3. Update `CMakeLists.txt` to generate interfaces
4. Build and verify

**Design Guidelines**:
- Clear, descriptive field names
- Use standard message types
- Include success status in result
- Provide meaningful feedback
- Add comments for clarity

**Structure**:
```
Goal: What to do
Result: What happened
Feedback: Progress updates
```

## What's Next?

You've completed the Actions module! You can now:
- ✓ Understand what actions are
- ✓ Create action servers
- ✓ Build action clients
- ✓ Define custom actions

Next, you'll learn about **TF2** for coordinate transformations:
- Lesson 6: Broadcasting Transformations
- Lesson 7: Listening to Transformations
- Lesson 8: TF2 Advanced Topics

## Further Reading

- [Creating Custom Action Interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Interface Definition](https://design.ros2.org/articles/interface_definition.html)
- [Standard Messages](http://wiki.ros.org/common_msgs)

---

**Exercise**: Create a `ChargeRobot.action` with:
- Goal: target charge percentage, charging strategy
- Result: success, final battery level, time taken
- Feedback: current battery level, estimated time remaining

**Next Lesson**: [Lesson 6: Broadcasting Transformations](06-broadcasting-transformations.md)
