# Multi-Node Communication Example

Complete multi-node system demonstrating data pipeline architecture.

## Architecture

```
sensor_node → /raw_data → processor_node → /processed_data → monitor_node
                                                                    ↓
coordinator_node ←─── /get_stats service ─────────────────────────┘
```

## Nodes

1. **sensor_node** - Simulated sensor publishing raw data (10 Hz)
2. **processor_node** - Processes raw data (moving average filter)
3. **monitor_node** - Monitors processed data, provides statistics service
4. **coordinator_node** - Periodically requests statistics

## Quick Start

```bash
# Build
colcon build --packages-select multi_node_communication
source install/setup.bash

# Run all nodes (4 terminals)
ros2 run multi_node_communication sensor_node
ros2 run multi_node_communication processor_node
ros2 run multi_node_communication monitor_node
ros2 run multi_node_communication coordinator_node
```

## Key Concepts

- Data processing pipeline (pub/sub chain)
- Service-based monitoring
- Multi-node system integration
- Timer-based coordination

## Related Lessons

- Lessons 4-5: Topics
- Lessons 7-8: Services
- All ROS2 Fundamentals concepts integrated
