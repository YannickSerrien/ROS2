# Exercise 3: Parameters and Configuration

## Objective
Create a configurable robot simulation node that uses parameters for configuration and can be reconfigured at runtime.

## Learning Goals
- Declare and use parameters
- Load parameters from YAML files
- Implement parameter validation
- Handle runtime parameter changes
- Use different parameter types

## Task Description

Create a node that simulates a robot with configurable properties:

### RobotSimNode
- Configurable parameters: robot name, speed, operating mode
- Validates parameter values
- Responds to parameter changes at runtime
- Publishes robot status periodically

## Requirements

### Parameters to Implement
1. **robot_name** (string) - Name of the robot
   - Default: "DefaultRobot"
   - Validation: Non-empty string

2. **max_speed** (double) - Maximum speed in m/s
   - Default: 1.0
   - Validation: Range [0.1, 5.0]

3. **operating_mode** (string) - Current operating mode
   - Default: "manual"
   - Validation: One of ["manual", "auto", "standby"]

4. **enable_sensors** (bool) - Whether sensors are enabled
   - Default: true

### Behavior
- Log all parameters on startup
- Publish status message every 2 seconds containing current configuration
- Validate parameter changes and reject invalid values
- Log when parameters are successfully changed

## Files to Complete

**Starter code**: `starter/`
- `src/robot_sim_node.cpp` - Complete the TODOs
- `config/robot_params.yaml` - Parameter file (provided)
- `CMakeLists.txt` - Already configured
- `package.xml` - Already configured

**Solution**: `solution/` (for reference after attempting)

## Build Instructions

```bash
cd ROS2-CPP-Course/02-ros2-fundamentals/exercises/03-parameters-exercise/starter
colcon build --packages-select parameters_exercise
source install/setup.bash
```

## Running the Exercise

Run with default parameters:
```bash
ros2 run parameters_exercise robot_sim_node
```

Run with custom parameters:
```bash
ros2 run parameters_exercise robot_sim_node --ros-args -p robot_name:="TestBot" -p max_speed:=2.5
```

Run with parameter file:
```bash
ros2 run parameters_exercise robot_sim_node --ros-args --params-file src/parameters_exercise/config/robot_params.yaml
```

Change parameters at runtime:
```bash
ros2 param set /robot_sim robot_name "NewName"
ros2 param set /robot_sim max_speed 3.0
ros2 param set /robot_sim operating_mode "auto"
```

List current parameters:
```bash
ros2 param list /robot_sim
ros2 param get /robot_sim max_speed
```

## Expected Output

**Startup:**
```
[INFO] [robot_sim]: Robot Simulation Node started
[INFO] [robot_sim]: Configuration:
[INFO] [robot_sim]:   Robot Name: DefaultRobot
[INFO] [robot_sim]:   Max Speed: 1.00 m/s
[INFO] [robot_sim]:   Operating Mode: manual
[INFO] [robot_sim]:   Sensors Enabled: true
```

**Status updates:**
```
[INFO] [robot_sim]: Status - Name: DefaultRobot, Speed: 1.00 m/s, Mode: manual, Sensors: ON
```

**Parameter change (valid):**
```
[INFO] [robot_sim]: Parameter 'max_speed' changed to: 2.50
```

**Parameter change (invalid):**
```
[WARN] [robot_sim]: Invalid max_speed: 10.00 (must be between 0.1 and 5.0)
```

## Testing Your Solution

1. Node starts with default parameters
2. Parameters can be loaded from YAML file
3. Parameters can be set via command line
4. Invalid values are rejected (e.g., max_speed = 10.0)
5. Runtime parameter changes work correctly
6. Status messages show current configuration

## Hints

1. Declaring parameters:
   ```cpp
   this->declare_parameter("param_name", default_value);
   ```

2. Getting parameter values:
   ```cpp
   std::string name = this->get_parameter("robot_name").as_string();
   double speed = this->get_parameter("max_speed").as_double();
   bool enabled = this->get_parameter("enable_sensors").as_bool();
   ```

3. Parameter change callback:
   ```cpp
   param_callback_handle_ = this->add_on_set_parameters_callback(
       std::bind(&RobotSimNode::parameters_callback, this, std::placeholders::_1));
   ```

4. Validating parameters in callback:
   ```cpp
   rcl_interfaces::msg::SetParametersResult result;
   result.successful = true;  // or false if invalid
   result.reason = "Validation message";
   return result;
   ```

## Verification Questions

After completing this exercise, you should be able to answer:
1. What are the different ways to set parameters in ROS2?
2. How do you validate parameter values?
3. What happens when you try to set an invalid parameter value?
4. Why is parameter validation important in robotics applications?
5. How can you list all parameters of a running node?

## Common Mistakes

- Forgetting to declare parameters before using them
- Not handling the case when parameter doesn't exist
- Incorrect parameter type conversion
- Not validating parameter values
- Forgetting to return result in parameter callback
- Not checking `result.successful` before applying changes

## Related Lessons

- [Lesson 6: Parameters](../../lessons/06-parameters.md)
- [Lesson 8: Launch Files](../../lessons/08-launch-files.md)

## Extension Challenges

Once you've completed the basic exercise:
1. Add a service to reset parameters to defaults
2. Save current parameters to a file
3. Add parameters for position (x, y coordinates)
4. Implement parameter groups/namespaces
5. Add parameter descriptions using parameter descriptors

## Next Steps

Once you've completed this exercise, move on to:
- **Exercise 4**: Integration exercise (multi-node system)
