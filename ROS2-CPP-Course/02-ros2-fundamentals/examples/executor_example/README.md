# Executor Example

Demonstrating ROS2 executors and thread safety patterns.

## Nodes

1. **single_threaded** - Default single-threaded executor (callbacks sequential)
2. **multi_threaded** - Multi-threaded executor (callbacks concurrent)
3. **callback_groups** - Callback groups (MutuallyExclusive vs Reentrant)
4. **thread_safe_counter** - Thread safety (unsafe, mutex, atomic comparison)

## Key Concepts

- Single-threaded executor (default)
- Multi-threaded executor with configurable thread count
- Callback groups for controlling concurrency
- Thread safety with mutex and atomic variables
- Race condition demonstration

## Quick Start

```bash
# Build
colcon build --packages-select executor_example
source install/setup.bash

# Run examples (compare behavior)
ros2 run executor_example single_threaded
ros2 run executor_example multi_threaded
ros2 run executor_example callback_groups
ros2 run executor_example thread_safe_counter
```

## Observations

**single_threaded**: Callbacks wait for each other
**multi_threaded**: Callbacks execute concurrently on different threads
**callback_groups**: Exclusive group serializes, reentrant allows concurrency
**thread_safe_counter**: Watch for race conditions in unsafe counter!

## Related Lessons

- [Lesson 12: Executors](../../lessons/12-executors.md)
