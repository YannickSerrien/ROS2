# Module 2: ROS2 Fundamentals - Code Examples

This directory contains complete, working ROS2 C++ packages demonstrating the concepts taught in Module 2.

## 📦 Example Packages

### 1. **pubsub_example** - Publisher and Subscriber Communication
Demonstrates basic topic communication with publishers and subscribers.

**Files:**
- `talker_node.cpp` - Publisher that sends messages periodically
- `listener_node.cpp` - Subscriber that receives and processes messages
- `parameter_talker.cpp` - Configurable publisher using parameters

**Concepts:**
- Creating publishers and subscribers
- Timer-based periodic publishing
- Message handling in callbacks
- Parameters for configuration
- QoS settings

**Build and run:**
```bash
cd ~/ros2_ws/src
cp -r ROS2-CPP-Course/02-ros2-fundamentals/examples/pubsub_example .
cd ~/ros2_ws
colcon build --packages-select pubsub_example
source install/setup.bash

# Terminal 1
ros2 run pubsub_example talker

# Terminal 2
ros2 run pubsub_example listener

# With parameters
ros2 run pubsub_example parameter_talker --ros-args -p topic:=custom_topic -p rate:=2.0
```

### 2. **service_example** - Service Server and Client
Complete service implementation with servers and clients.

**Files:**
- `add_server.cpp` - Service server that adds two numbers
- `add_client.cpp` - Service client
- `calculator_server.cpp` - Multi-service calculator
- `robot_controller_server.cpp` - Real-world service example

**Concepts:**
- Creating service servers
- Calling services from clients
- Timeout handling
- Multiple services in one node
- Error handling and validation

**Build and run:**
```bash
colcon build --packages-select service_example
source install/setup.bash

# Terminal 1
ros2 run service_example add_server

# Terminal 2
ros2 run service_example add_client 5 3
```

### 3. **parameters_example** - Parameter Configuration
Parameter usage, validation, and dynamic reconfiguration.

**Files:**
- `parameter_node.cpp` - Basic parameter usage
- `configurable_robot.cpp` - Robot with parameter-based configuration
- `dynamic_parameters.cpp` - Dynamic parameter updates with callbacks

**Concepts:**
- Declaring parameters
- Getting parameter values
- Loading from YAML files
- Parameter callbacks
- Validation

**Build and run:**
```bash
colcon build --packages-select parameters_example
source install/setup.bash

ros2 run parameters_example configurable_robot --ros-args \
    -p max_speed:=2.0 \
    -p robot_name:=alpha

# Or with YAML
ros2 run parameters_example configurable_robot --ros-args \
    --params-file config/robot_params.yaml
```

### 4. **timer_logging_example** - Timers and Logging
Advanced timer usage and professional logging practices.

**Files:**
- `periodic_publisher.cpp` - Timer-based publisher
- `heartbeat_node.cpp` - Heartbeat with logging levels
- `multi_timer_node.cpp` - Multiple timers
- `watchdog_node.cpp` - Timeout monitoring

**Concepts:**
- Creating timers
- Periodic callbacks
- Logging levels
- Throttled logging
- Conditional logging

**Build and run:**
```bash
colcon build --packages-select timer_logging_example
source install/setup.bash

ros2 run timer_logging_example heartbeat --ros-args --log-level debug
```

### 5. **multi_node_communication** - Complex Multi-Node System
Complete system with multiple nodes communicating.

**Files:**
- `sensor_node.cpp` - Simulated sensor publisher
- `processor_node.cpp` - Data processor (sub + pub)
- `monitor_node.cpp` - Monitoring with services
- `coordinator_node.cpp` - System coordinator

**Concepts:**
- Multi-node architecture
- Data processing pipelines
- Service-based control
- Parameter-based configuration
- System integration

**Build and run:**
```bash
colcon build --packages-select multi_node_communication
source install/setup.bash

# Run all nodes
ros2 run multi_node_communication sensor_node &
ros2 run multi_node_communication processor_node &
ros2 run multi_node_communication monitor_node &
ros2 run multi_node_communication coordinator_node

# Or use launch file (Module 3)
```

### 6. **executor_example** - Executors and Thread Safety
Multi-threaded execution and callback groups.

**Files:**
- `single_threaded_node.cpp` - Default execution
- `multi_threaded_node.cpp` - Multi-threaded executor
- `callback_groups_node.cpp` - Callback group demonstration
- `thread_safe_counter.cpp` - Thread safety patterns

**Concepts:**
- Single-threaded vs multi-threaded executors
- Callback groups
- Thread safety (mutex, atomic)
- Concurrent callbacks
- Performance optimization

**Build and run:**
```bash
colcon build --packages-select executor_example
source install/setup.bash

ros2 run executor_example multi_threaded_node
```

## 🔨 Building All Examples

```bash
# From workspace root
cd ~/ros2_ws

# Copy all examples
cp -r ROS2-CPP-Course/02-ros2-fundamentals/examples/* src/

# Build all
colcon build

# Source
source install/setup.bash
```

## 📝 Package Structure

Each example package contains:

```
package_name/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── src/                    # Source code
│   ├── node1.cpp
│   ├── node2.cpp
│   └── ...
├── include/                # Headers (if needed)
│   └── package_name/
├── config/                 # YAML config files (if needed)
│   └── params.yaml
└── README.md               # Package-specific docs
```

## 🎯 Learning Path

**Recommended order:**

1. **pubsub_example** - Start here! Basic communication
2. **service_example** - Request-response pattern
3. **parameters_example** - Node configuration
4. **timer_logging_example** - Periodic tasks and logging
5. **multi_node_communication** - System integration
6. **executor_example** - Advanced (multi-threading)

## 💡 Tips

- Read the lesson first, then run the corresponding example
- Modify the examples and rebuild to experiment
- Use `ros2 topic`, `ros2 service`, `ros2 param` commands to inspect
- Check logs for understanding program flow
- Try different parameter values
- Use `rqt_graph` to visualize node connections

## 🐛 Troubleshooting

**Build fails:**
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --packages-select package_name
```

**Can't find executable:**
```bash
# Source workspace
source install/setup.bash

# Check if built
ros2 pkg executables package_name
```

**Dependencies missing:**
```bash
# Install ROS2 dependencies
rosdep install --from-paths src -y --ignore-src
```

## 📚 Related Lessons

- [Lesson 3: First Node](../lessons/03-first-node.md)
- [Lesson 4: Publishers](../lessons/04-topics-publishers.md)
- [Lesson 5: Subscribers](../lessons/05-topics-subscribers.md)
- [Lesson 7: Service Servers](../lessons/07-service-servers.md)
- [Lesson 8: Service Clients](../lessons/08-service-clients.md)
- [Lesson 9: Parameters](../lessons/09-parameters.md)
- [Lesson 10: Logging](../lessons/10-logging.md)
- [Lesson 11: Timers](../lessons/11-timers.md)
- [Lesson 12: Executors](../lessons/12-executors.md)

---

**Practice with these examples to solidify your ROS2 knowledge!**
