# Module 2 Mini-Project: Temperature Monitoring System

## Project Overview

Build a complete temperature monitoring and alert system that demonstrates all fundamental ROS2 concepts learned in Module 2.

**Estimated Time**: 3-4 hours

## Learning Objectives

By completing this project, you will:
- Design and implement a multi-node ROS2 application
- Use topics for continuous data streams
- Implement services for queries and commands
- Configure nodes using parameters and YAML files
- Create custom message and service types
- Implement proper logging and error handling
- Build a launch file for the complete system

## Project Description

You will build a temperature monitoring system for a building with multiple sensors. The system will:
- Simulate multiple temperature sensors in different rooms
- Monitor temperatures and detect anomalies
- Provide alerts when thresholds are exceeded
- Allow querying of statistics
- Support runtime reconfiguration

## System Architecture

```
┌─────────────┐     /temperatures      ┌──────────────┐     /alerts        ┌─────────────┐
│  Sensor     │─────topic─────────────>│  Monitor     │─────topic────────>│  Alerter    │
│  Nodes      │                        │  Node        │                    │  Node       │
│  (3 rooms)  │                        │              │                    │             │
└─────────────┘                        │              │<───service─────────│  Dashboard  │
                                       │              │  /get_stats        │  Node       │
                                       └──────────────┘                    └─────────────┘
                                              ^
                                              |
                                         /reset_stats
                                          (service)
```

## Requirements

### Part 1: Custom Message Types

Create custom messages and services:

**msg/TemperatureReading.msg**
```
string sensor_id
string room_name
float64 temperature
float64 humidity
int64 timestamp
```

**msg/Alert.msg**
```
string sensor_id
string room_name
string alert_type  # "HIGH", "LOW", "CRITICAL"
float64 temperature
string message
int64 timestamp
```

**srv/GetStats.srv**
```
string room_name  # Empty string for all rooms
---
bool success
string[] rooms
float64[] avg_temperatures
float64[] min_temperatures
float64[] max_temperatures
int32[] sample_counts
string message
```

### Part 2: Sensor Nodes (3 instances)

Create a `TempSensorNode` that can be run multiple times:

**Node name**: `temp_sensor_<room_name>`

**Parameters**:
- `room_name` (string) - Room identifier
- `sensor_id` (string) - Unique sensor ID
- `base_temperature` (double, default: 20.0) - Base temperature in °C
- `temperature_variance` (double, default: 5.0) - Random variance
- `humidity_base` (double, default: 50.0) - Base humidity %
- `publish_rate` (double, default: 1.0) - Publishing frequency in Hz

**Topics**:
- Publishes: `/temperatures` (TemperatureReading)

**Behavior**:
- Generate realistic temperature readings with random variations
- Simulate gradual temperature changes (not random jumps)
- Include timestamp in each reading
- Log sensor readings periodically

### Part 3: Monitor Node

**Node name**: `temp_monitor`

**Parameters**:
- `high_threshold` (double, default: 25.0) - High temperature alert threshold
- `low_threshold` (double, default: 15.0) - Low temperature alert threshold
- `critical_threshold` (double, default: 30.0) - Critical temperature threshold

**Topics**:
- Subscribes: `/temperatures` (TemperatureReading)
- Publishes: `/alerts` (Alert)

**Services**:
- Provides: `/get_stats` (GetStats)
- Provides: `/reset_stats` (std_srvs/Trigger)

**Behavior**:
- Track statistics for each room (count, min, max, average)
- Generate alerts when thresholds are exceeded
- Provide statistics via service
- Allow resetting statistics

### Part 4: Alerter Node

**Node name**: `alerter`

**Parameters**:
- `log_file` (string, optional) - Path to log alerts to file
- `enable_sound` (bool, default: false) - Enable sound alerts (simulated)

**Topics**:
- Subscribes: `/alerts` (Alert)

**Behavior**:
- Display alerts with color-coded severity
- Log alerts to file if configured
- Track alert history
- Provide alert summary periodically

### Part 5: Dashboard Node

**Node name**: `dashboard`

**Parameters**:
- `update_interval` (double, default: 5.0) - How often to query stats (seconds)

**Services**:
- Calls: `/get_stats`

**Behavior**:
- Periodically request and display statistics for all rooms
- Format output in a table-like structure
- Calculate building-wide statistics

## File Structure

```
temperature_monitor/
├── README.md
├── package.xml
├── CMakeLists.txt
├── msg/
│   ├── TemperatureReading.msg
│   └── Alert.msg
├── srv/
│   └── GetStats.srv
├── src/
│   ├── temp_sensor_node.cpp
│   ├── monitor_node.cpp
│   ├── alerter_node.cpp
│   └── dashboard_node.cpp
├── config/
│   ├── sensors.yaml
│   └── monitor.yaml
└── launch/
    └── temperature_system.launch.py
```

## Implementation Steps

### Step 1: Project Setup (30 min)
1. Create package with dependencies
2. Define custom messages and services
3. Update CMakeLists.txt and package.xml
4. Verify interfaces build correctly

### Step 2: Sensor Node (45 min)
1. Implement temperature simulation logic
2. Add parameter handling
3. Implement publishing
4. Test with `ros2 topic echo`

### Step 3: Monitor Node (60 min)
1. Implement statistics tracking (per room)
2. Add alert generation logic
3. Implement both services
4. Test with multiple sensors

### Step 4: Alerter Node (30 min)
1. Implement alert subscriber
2. Add colored console output
3. Implement optional file logging
4. Test with monitor

### Step 5: Dashboard Node (30 min)
1. Implement stats service client
2. Format statistics display
3. Add periodic updates
4. Test complete system

### Step 6: Launch File (30 min)
1. Create launch file for all nodes
2. Configure parameters via YAML
3. Test full system startup
4. Document usage

## Sample Configuration Files

**config/sensors.yaml**
```yaml
temp_sensor_living_room:
  ros__parameters:
    room_name: "Living Room"
    sensor_id: "SENSOR_LR_01"
    base_temperature: 22.0
    temperature_variance: 3.0
    publish_rate: 1.0

temp_sensor_bedroom:
  ros__parameters:
    room_name: "Bedroom"
    sensor_id: "SENSOR_BR_01"
    base_temperature: 20.0
    temperature_variance: 2.0
    publish_rate: 1.0

temp_sensor_kitchen:
  ros__parameters:
    room_name: "Kitchen"
    sensor_id: "SENSOR_KT_01"
    base_temperature: 24.0
    temperature_variance: 5.0
    publish_rate: 1.0
```

**config/monitor.yaml**
```yaml
temp_monitor:
  ros__parameters:
    high_threshold: 25.0
    low_threshold: 15.0
    critical_threshold: 30.0

alerter:
  ros__parameters:
    log_file: "/tmp/temperature_alerts.log"
    enable_sound: false

dashboard:
  ros__parameters:
    update_interval: 5.0
```

## Testing Your Implementation

### Basic Functionality
```bash
# Terminal 1: Launch entire system
ros2 launch temperature_monitor temperature_system.launch.py

# Terminal 2: Monitor topics
ros2 topic list
ros2 topic hz /temperatures
ros2 topic echo /alerts

# Terminal 3: Call services
ros2 service call /get_stats temperature_monitor/srv/GetStats "{room_name: ''}"
ros2 service call /reset_stats std_srvs/srv/Trigger "{}"

# Terminal 4: Check parameters
ros2 param list
ros2 param get /temp_monitor high_threshold
ros2 param set /temp_monitor high_threshold 28.0
```

### Expected Output

**Dashboard (every 5 seconds)**:
```
╔═══════════════════════════════════════════════════════════╗
║              TEMPERATURE MONITORING DASHBOARD             ║
╠═══════════════════════════════════════════════════════════╣
║ Room           │ Samples │ Current │ Avg    │ Min  │ Max  ║
╠════════════════╪═════════╪═════════╪════════╪══════╪══════╣
║ Living Room    │   45    │  22.3°C │ 22.1°C │ 19°C │ 25°C ║
║ Bedroom        │   45    │  20.1°C │ 20.3°C │ 18°C │ 23°C ║
║ Kitchen        │   45    │  26.8°C │ 24.5°C │ 20°C │ 29°C ║
╠════════════════╪═════════╪═════════╪════════╪══════╪══════╣
║ Building Avg   │  135    │   --    │ 22.3°C │ 18°C │ 29°C ║
╚═══════════════════════════════════════════════════════════╝
```

**Alerter**:
```
[WARN] [alerter]: ⚠️  HIGH TEMP ALERT - Kitchen (SENSOR_KT_01): 26.5°C
[ERROR] [alerter]: 🚨 CRITICAL ALERT - Kitchen (SENSOR_KT_01): 30.2°C
```

## Validation Criteria

Your project should:
- [ ] All four nodes compile and run without errors
- [ ] Sensors publish temperature readings at configured rates
- [ ] Monitor detects and publishes alerts correctly
- [ ] Statistics service returns accurate data
- [ ] Reset service clears statistics
- [ ] Dashboard displays formatted statistics
- [ ] Launch file starts all nodes with correct parameters
- [ ] Parameters can be changed at runtime
- [ ] System handles multiple sensors correctly
- [ ] Proper logging at appropriate levels

## Extension Challenges

Once you've completed the basic project:

1. **Advanced Features**:
   - Add action server for calibrating sensors
   - Implement moving average filtering
   - Add data persistence (save to file)
   - Create CSV export service

2. **Visualization**:
   - Use rqt_plot for real-time graphing
   - Create custom rqt plugin
   - Generate summary reports

3. **Robustness**:
   - Handle sensor timeouts/failures
   - Implement sensor health monitoring
   - Add automatic threshold adjustment

4. **Scalability**:
   - Support dynamic sensor addition/removal
   - Implement room grouping (zones)
   - Add multi-building support

## Deliverables

Submit the following:
1. Complete source code for all nodes
2. Custom message and service definitions
3. Configuration YAML files
4. Launch file
5. README with build and run instructions
6. Brief report (1-2 pages) describing:
   - Design decisions
   - Challenges faced
   - How you tested the system
   - Potential improvements

## Evaluation Rubric

| Criteria | Points |
|----------|--------|
| Custom messages/services defined correctly | 10 |
| Sensor nodes work correctly | 20 |
| Monitor node tracks stats and generates alerts | 25 |
| Alerter displays alerts properly | 15 |
| Dashboard queries and displays stats | 15 |
| Launch file and configuration | 10 |
| Code quality and documentation | 5 |
| **Total** | **100** |

## Tips for Success

1. **Start Simple**: Get one sensor working before adding multiple
2. **Test Incrementally**: Test each node independently before integration
3. **Use ROS2 Tools**: Leverage `ros2 topic echo`, `ros2 service call`, etc.
4. **Log Everything**: Good logging helps debugging
5. **Handle Edge Cases**: What if no data received? Division by zero?
6. **Follow ROS2 Conventions**: Node names, topic names, parameter names
7. **Document As You Go**: Don't leave documentation for the end

## Resources

- [ROS2 Documentation](https://docs.ros.org/)
- [Creating Custom Interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- Module 2 Lessons and Examples

## Getting Help

If you're stuck:
1. Review Module 2 lessons and examples
2. Check the custom message/service example code
3. Test each component independently
4. Use `ros2 doctor` to check system health
5. Review error messages carefully

Good luck! This project brings together everything you've learned in Module 2!
