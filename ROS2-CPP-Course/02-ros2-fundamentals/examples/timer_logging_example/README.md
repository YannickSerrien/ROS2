# Timer and Logging Example

This package demonstrates ROS2 timers and professional logging practices.

## Nodes

1. **periodic_publisher** - Multiple timers with different frequencies
2. **heartbeat** - All logging levels and conditional logging
3. **multi_timer** - Timer management (cancel, reset, one-shot)
4. **watchdog** - Timeout monitoring pattern

## Key Concepts

- Creating wall timers with `create_wall_timer()`
- Logging levels: DEBUG, INFO, WARN, ERROR, FATAL
- Conditional logging: ONCE, SKIPFIRST, THROTTLE, EXPRESSION
- Timer cancel and reset
- Watchdog pattern for timeout detection

## Quick Start

```bash
# Build
colcon build --packages-select timer_logging_example
source install/setup.bash

# Run examples
ros2 run timer_logging_example periodic_publisher
ros2 run timer_logging_example heartbeat --ros-args --log-level debug
ros2 run timer_logging_example multi_timer
ros2 run timer_logging_example watchdog
```

## Related Lessons

- [Lesson 10: Logging](../../lessons/10-logging.md)
- [Lesson 11: Timers](../../lessons/11-timers.md)
