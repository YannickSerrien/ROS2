# Exercise 3: Node Composition and Performance

## Objective

Convert regular ROS2 nodes to component nodes and compare performance between composed and separate execution.

## Learning Goals

- Convert regular nodes to components
- Create ComposableNodeContainer launch files
- Measure performance differences
- Understand zero-copy benefits
- Use component registration macros

## Background

Node composition enables:
- **Zero-copy messaging**: Messages passed by pointer (no serialization)
- **Lower latency**: No inter-process communication overhead
- **Reduced CPU**: No serialization/deserialization
- **Lower memory**: Shared process space

Perfect for high-frequency, large-message pipelines.

## Task Description

Convert a data processing pipeline to use composition and measure the performance improvement.

### System Architecture

**Pipeline**: Producer → Processor → Consumer
- **Producer**: Generates large data (simulated sensor)
- **Processor**: Processes data (simple transformation)
- **Consumer**: Receives and validates data

### Requirements

1. **Convert Three Nodes to Components**:
   - `DataProducer` → `DataProducerComponent`
   - `DataProcessor` → `DataProcessorComponent`
   - `DataConsumer` → `DataConsumerComponent`

2. **Create Launch Files**:
   - `composed.launch.py`: All nodes in ONE container (fast)
   - `separate.launch.py`: Each node separate process (baseline)

3. **Measure Performance**:
   - Record latency (producer timestamp → consumer receipt)
   - Monitor CPU usage
   - Compare composed vs separate

### Component Requirements

**Each component must**:
- Accept `NodeOptions` in constructor
- Pass options to base Node class
- Use `RCLCPP_COMPONENTS_REGISTER_NODE` macro
- Publish with `std::move(unique_ptr)` for zero-copy

**Data Producer**:
- Publish large messages (configurable size)
- Include timestamp in message
- Configurable publish rate

**Data Processor**:
- Subscribe to producer
- Process data (simple transformation)
- Republish processed data

**Data Consumer**:
- Subscribe to processor
- Calculate latency from timestamp
- Log statistics

### CMakeLists.txt Requirements

- Build components as shared libraries
- Use `rclcpp_components_register_node`
- Generate standalone executables
- Install properly

## Starter Code

Provides:
- Regular node implementations
- TODO comments for conversion
- Launch file templates
- Custom message definition

Your tasks:
1. Add `NodeOptions` parameters
2. Add component registration
3. Update CMakeLists.txt
4. Create launch files
5. Test and compare performance

## Testing

### Test 1: Composed System (Fast)

```bash
ros2 launch node_composition composed.launch.py
```

**Monitor performance**:
```bash
# Terminal 2: Check latency logs
# Consumer will print latency statistics

# Terminal 3: Monitor CPU
top -p $(pgrep -f component_container)
```

### Test 2: Separate Processes (Baseline)

```bash
ros2 launch node_composition separate.launch.py
```

**Monitor**:
```bash
# Check CPU usage of all processes
ps aux | grep node_composition
```

### Test 3: Performance Comparison

Run both and compare:
- **Latency**: Composed should be 5-10x faster
- **CPU**: Composed should use 30-50% less
- **Memory**: Composed should use less total RAM

### Test 4: Different Message Sizes

Modify data size parameter:
```bash
# Small messages (1 KB) - less difference
ros2 launch node_composition composed.launch.py data_size:=1000

# Large messages (1 MB) - huge difference
ros2 launch node_composition composed.launch.py data_size:=1000000
```

## Hints

<details>
<summary>Hint 1: Component Conversion</summary>

```cpp
// Before (regular node)
class MyNode : public rclcpp::Node {
    MyNode() : Node("my_node") {}
};

// After (component)
class MyComponent : public rclcpp::Node {
    explicit MyComponent(const rclcpp::NodeOptions & options)
        : Node("my_component", options) {}
};

RCLCPP_COMPONENTS_REGISTER_NODE(MyComponent)
```
</details>

<details>
<summary>Hint 2: CMakeLists.txt for Components</summary>

```cmake
add_library(my_component SHARED src/my_component.cpp)
ament_target_dependencies(my_component rclcpp rclcpp_components)

rclcpp_components_register_node(my_component
    PLUGIN "MyComponent"
    EXECUTABLE my_node
)

install(TARGETS my_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
```
</details>

<details>
<summary>Hint 3: Composed Launch File</summary>

```python
ComposableNodeContainer(
    name='pipeline_container',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='node_composition',
            plugin='DataProducerComponent',
            extra_arguments=[{'use_intra_process_comms': True}]
        ),
        # ... more components
    ]
)
```
</details>

<details>
<summary>Hint 4: Zero-Copy Publishing</summary>

```cpp
// Use unique_ptr for zero-copy
auto msg = std::make_unique<MyMsg>();
publisher_->publish(std::move(msg));  // Transfers ownership
```
</details>

## Performance Metrics

Measure and record:

| Metric | Separate | Composed | Improvement |
|--------|----------|----------|-------------|
| Latency (ms) | ___ | ___ | ___x faster |
| CPU (%) | ___ | ___ | ___% reduction |
| Memory (MB) | ___ | ___ | ___ MB saved |

**Expected Results** (1 MB messages @ 10 Hz):
- Latency: 50ms → 5ms (10x faster)
- CPU: 40% → 15% (60% reduction)
- Memory: 150MB → 80MB (47% reduction)

## Bonus Challenges

1. **Variable Load**: Test with different publish rates (1 Hz, 10 Hz, 100 Hz)
2. **Pipeline Length**: Add more processing stages
3. **Mixed Mode**: Some composed, some separate
4. **Memory Pooling**: Pre-allocate message buffers

## Success Criteria

- [ ] All three nodes converted to components
- [ ] Components compile and register correctly
- [ ] Composed launch file works
- [ ] Separate launch file works
- [ ] Performance measurements recorded
- [ ] Composed version shows improvement
- [ ] Zero-copy verified (low latency)

## Common Issues

**Component Not Found**:
```bash
# Rebuild and check registration
colcon build --packages-select node_composition
ros2 component types node_composition
```

**Compilation Error**:
- Ensure `NodeOptions` parameter exists
- Check `RCLCPP_COMPONENTS_REGISTER_NODE` placement
- Verify CMakeLists.txt library type (SHARED)

**No Performance Improvement**:
- Check `use_intra_process_comms: True` in launch file
- Verify all nodes in SAME container
- Ensure using `std::move` when publishing

**Segmentation Fault**:
- Pass `options` to base class: `Node("name", options)`
- Check all shared_ptr initialized properly

## Verification

**Check component registration**:
```bash
ros2 component types | grep node_composition
```

**Expected output**:
```
node_composition::DataProducerComponent
node_composition::DataProcessorComponent
node_composition::DataConsumerComponent
```

**Check container**:
```bash
ros2 component list /pipeline_container
```

## Related Lessons

- Lesson 11: Composition Basics
- Lesson 12: Component Nodes
- Example: composition_example package

## Next Steps

- Complete all Module 3 exercises
- Apply composition to mini-project
- Understand when to use composition vs separate nodes
- Explore lifecycle composition (Module 6)
