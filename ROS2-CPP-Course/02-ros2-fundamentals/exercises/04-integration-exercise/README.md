# Exercise 4: Multi-Node Integration

## Objective
Build a complete multi-node system that integrates topics, services, and parameters to create a simple data processing pipeline.

## Learning Goals
- Integrate multiple ROS2 concepts (topics, services, parameters)
- Design a multi-node system architecture
- Implement data flow between nodes
- Use different communication patterns appropriately
- Build a cohesive application from individual components

## Task Description

Create a data processing system with three nodes:

### System Architecture
```
[DataGenerator] --> /raw_data --> [DataProcessor] --> /processed_data --> [DataMonitor]
                                        ^
                                        |
                                  /process_stats (service)
```

### 1. DataGenerator Node
- Publishes random sensor data to `/raw_data` topic
- Uses Float64 messages
- Publishing rate configurable via parameter (default: 1 Hz)
- Data range configurable via parameters (min/max)

### 2. DataProcessor Node
- Subscribes to `/raw_data`
- Processes data (scaling, filtering, etc.)
- Publishes results to `/processed_data`
- Provides `/process_stats` service for statistics
- Processing mode configurable via parameter

### 3. DataMonitor Node
- Subscribes to `/processed_data`
- Displays received values
- Periodically calls `/process_stats` service
- Alert threshold configurable via parameter

## Requirements

### DataGenerator
**Parameters:**
- `publish_rate` (double, default: 1.0) - Publishing frequency in Hz
- `data_min` (double, default: 0.0) - Minimum random value
- `data_max` (double, default: 100.0) - Maximum random value

**Topics:**
- Publishes: `/raw_data` (std_msgs/Float64)

### DataProcessor
**Parameters:**
- `scale_factor` (double, default: 1.0) - Multiply data by this factor
- `enable_filtering` (bool, default: false) - Enable/disable filtering

**Topics:**
- Subscribes: `/raw_data` (std_msgs/Float64)
- Publishes: `/processed_data` (std_msgs/Float64)

**Services:**
- Provides: `/process_stats` (std_srvs/Trigger)
  - Returns: count, average, min, max of processed data

### DataMonitor
**Parameters:**
- `alert_threshold` (double, default: 50.0) - Threshold for alerts
- `stats_interval` (double, default: 5.0) - How often to request stats (seconds)

**Topics:**
- Subscribes: `/processed_data` (std_msgs/Float64)

**Services:**
- Calls: `/process_stats`

## Files to Complete

**Starter code**: `starter/`
- `src/data_generator.cpp` - Complete the TODOs
- `src/data_processor.cpp` - Complete the TODOs
- `src/data_monitor.cpp` - Complete the TODOs
- `CMakeLists.txt` - Already configured
- `package.xml` - Already configured

**Solution**: `solution/` (for reference after attempting)

## Build Instructions

```bash
cd ROS2-CPP-Course/02-ros2-fundamentals/exercises/04-integration-exercise/starter
colcon build --packages-select integration_exercise
source install/setup.bash
```

## Running the Exercise

Terminal 1 (Generator):
```bash
ros2 run integration_exercise data_generator
```

Terminal 2 (Processor):
```bash
ros2 run integration_exercise data_processor --ros-args -p scale_factor:=2.0
```

Terminal 3 (Monitor):
```bash
ros2 run integration_exercise data_monitor --ros-args -p alert_threshold:=75.0
```

## Expected Output

**data_generator:**
```
[INFO] [data_generator]: Data Generator started (rate: 1.00 Hz, range: [0.00, 100.00])
[INFO] [data_generator]: Publishing: 42.5
[INFO] [data_generator]: Publishing: 67.3
```

**data_processor:**
```
[INFO] [data_processor]: Data Processor started (scale: 2.00, filtering: disabled)
[INFO] [data_processor]: Received: 42.5 -> Processed: 85.0
[INFO] [data_processor]: Received: 67.3 -> Processed: 134.6
[INFO] [data_processor]: Stats requested - Count: 10, Avg: 112.5, Min: 50.0, Max: 180.0
```

**data_monitor:**
```
[INFO] [data_monitor]: Data Monitor started (threshold: 75.0, stats interval: 5.0s)
[INFO] [data_monitor]: Received: 85.0 [ALERT: Above threshold!]
[INFO] [data_monitor]: Received: 134.6 [ALERT: Above threshold!]
[INFO] [data_monitor]: Statistics: Count: 10, Avg: 112.5
```

## Testing Your Solution

1. All three nodes run without errors
2. Data flows from generator → processor → monitor
3. Processor correctly scales data
4. Monitor detects values above threshold
5. Stats service returns correct information
6. Parameters can be changed to affect behavior

## Testing Commands

Monitor topics:
```bash
ros2 topic list
ros2 topic echo /raw_data
ros2 topic echo /processed_data
ros2 topic hz /raw_data
```

Call service manually:
```bash
ros2 service call /process_stats std_srvs/srv/Trigger
```

Change parameters:
```bash
ros2 param set /data_generator publish_rate 2.0
ros2 param set /data_processor scale_factor 3.0
ros2 param set /data_monitor alert_threshold 100.0
```

Visualize graph:
```bash
rqt_graph
```

## Hints

1. Random number generation:
   ```cpp
   #include <random>
   std::random_device rd;
   std::mt19937 gen(rd());
   std::uniform_real_distribution<> dist(min, max);
   double value = dist(gen);
   ```

2. Convert parameter to duration:
   ```cpp
   double rate_hz = this->get_parameter("publish_rate").as_double();
   auto period = std::chrono::duration<double>(1.0 / rate_hz);
   timer_ = this->create_wall_timer(period, callback);
   ```

3. Track statistics:
   ```cpp
   count_++;
   sum_ += value;
   min_ = std::min(min_, value);
   max_ = std::max(max_, value);
   double avg = sum_ / count_;
   ```

## Verification Questions

After completing this exercise, you should be able to answer:
1. Why use topics for continuous data and services for occasional requests?
2. How can you verify data is flowing between nodes?
3. What happens if the processor crashes while generator is still running?
4. How would you add a fourth node to this system?
5. What are the advantages of separating functionality into multiple nodes?

## Common Mistakes

- Topic names not matching between publisher and subscriber
- Forgetting to wait for service availability
- Not initializing statistics variables (min, max, count, sum)
- Incorrect parameter types or default values
- Service callback not returning a result
- Not handling the case when no data has been received yet

## System Diagram

```
┌──────────────┐         ┌───────────────┐         ┌──────────────┐
│ Generator    │         │  Processor    │         │  Monitor     │
│              │─topic──>│               │─topic──>│              │
│ /raw_data    │         │  /processed_  │         │ Display +    │
│              │         │   data        │<─srv────│ Stats Query  │
└──────────────┘         └───────────────┘         └──────────────┘
  Parameters:              Parameters:               Parameters:
  - publish_rate           - scale_factor            - alert_threshold
  - data_min/max           - enable_filtering        - stats_interval
```

## Related Lessons

- [Lesson 3: Topics](../../lessons/03-topics.md)
- [Lesson 4: Services](../../lessons/04-services.md)
- [Lesson 6: Parameters](../../lessons/06-parameters.md)
- [Lesson 7: ROS2 Tools](../../lessons/07-ros2-tools.md)

## Extension Challenges

Once you've completed the basic exercise:
1. Add data filtering (moving average) in processor
2. Create a launch file to start all nodes
3. Add a reset service to clear statistics
4. Implement parameter validation in all nodes
5. Add custom message type with timestamp and data
6. Create a logging node that saves data to file
7. Add visualization using rqt_plot

## Next Steps

Once you've completed this exercise:
- Review all Module 2 exercises
- Attempt the **Module 2 Mini-Project: Temperature Monitor**
- Move on to Module 3: ROS2 Intermediate
