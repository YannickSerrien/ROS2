# Lesson 1: ROS2 Architecture

## Learning Objectives

- Understand ROS2 vs ROS1 differences
- Learn about DDS (Data Distribution Service)
- Understand the ROS2 computational graph
- Know the key ROS2 concepts (nodes, topics, services, actions)
- Understand Quality of Service (QoS)

## What is ROS2?

**ROS2 (Robot Operating System 2)** is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify creating complex and robust robot behavior across a wide variety of robotic platforms.

### ROS1 vs ROS2

| Feature | ROS1 | ROS2 |
|---------|------|------|
| **Communication** | Custom protocol (TCPROS) | DDS (standard) |
| **Real-time** | Not supported | Supported |
| **Multi-robot** | Difficult | Native support |
| **Security** | None | Built-in encryption |
| **Platform** | Primarily Linux | Linux, Windows, macOS |
| **Python** | Python 2 | Python 3 |
| **C++ Standard** | C++03/11 | C++17 |
| **Master** | Required (roscore) | No master needed |
| **Lifecycle** | Basic | Advanced lifecycle management |

### Why ROS2?

- **Production-ready**: Designed for commercial products
- **Real-time capable**: Support for real-time systems
- **Secure**: Built-in security features (SROS2)
- **Multi-platform**: Works on Linux, Windows, macOS
- **Standard-based**: Uses DDS, a proven industry standard
- **Better tooling**: Improved debugging and development tools

## DDS: The Communication Layer

ROS2 uses **DDS (Data Distribution Service)** for communication instead of a custom protocol.

### What is DDS?

- **Industry standard** (OMG specification)
- **Publish-subscribe** model
- **Discovery protocol** (automatic peer detection)
- **Quality of Service** (configurable reliability, durability, etc.)
- **No central broker** (peer-to-peer)

### DDS Implementations

ROS2 supports multiple DDS implementations:
- **Fast DDS** (eProsima) - Default
- **CycloneDDS** (Eclipse)
- **Connext DDS** (RTI) - Commercial

```bash
# Check which DDS you're using
echo $RMW_IMPLEMENTATION

# Switch DDS implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### No Master Node!

Unlike ROS1 (which requires `roscore`), ROS2 nodes discover each other automatically through DDS:

```
ROS1:                          ROS2:
┌─────────┐                   ┌────────┐    ┌────────┐
│ roscore │                   │ Node 1 │◄──►│ Node 2 │
└────┬────┘                   └───┬────┘    └───┬────┘
     │                            │             │
┌────┴────┬────────┐             └──────┬──────┘
│  Node 1 │ Node 2 │                    │
└─────────┴────────┘               ┌────▼────┐
                                   │ Node 3  │
                                   └─────────┘
```

## ROS2 Computational Graph

The ROS2 system consists of **nodes** connected through a **communication graph**.

### Graph Concepts

```
┌──────────────────────────────────────────────────────┐
│                 ROS2 Graph                            │
│                                                       │
│  ┌────────┐                        ┌────────┐        │
│  │ Node A │────topic───────────────▶│ Node B │       │
│  └────────┘                        └────────┘        │
│      │                                  ▲             │
│      │                                  │             │
│      │service                      action│            │
│      │                                  │             │
│      ▼                                  │             │
│  ┌────────┐                        ┌────────┐        │
│  │ Node C │◄────parameter──────────│ Node D │        │
│  └────────┘                        └────────┘        │
│                                                       │
└──────────────────────────────────────────────────────┘
```

### Key Concepts

#### 1. Nodes
- **Independent processes** that perform computation
- **Single purpose**: Each node does one thing well
- **Communicate** via topics, services, actions
- **Example**: camera_driver, object_detector, motion_controller

#### 2. Topics
- **Named buses** for asynchronous message passing
- **Publish-subscribe** pattern (many-to-many)
- **Unidirectional** data flow
- **Best for**: Continuous data streams (sensor data, commands)

```cpp
Publisher ──message──▶ Topic ──message──▶ Subscriber(s)
```

#### 3. Services
- **Synchronous request-response** communication
- **Client-server** pattern (one-to-one)
- **Blocking** until response received
- **Best for**: Remote procedure calls, configuration

```cpp
Client ──request──▶ Service ──response──▶ Client
```

#### 4. Actions
- **Long-running tasks** with feedback
- **Goal-feedback-result** pattern
- **Cancellable** and **preemptable**
- **Best for**: Navigation, manipulation, complex behaviors

```cpp
Client ──goal──▶ Action Server
Client ◀──feedback── Action Server
Client ◀──result──── Action Server
```

#### 5. Parameters
- **Node configuration values**
- **Can be changed at runtime**
- **Typed** (int, double, string, bool, arrays)
- **Best for**: Tuning, configuration

## ROS2 Node Lifecycle

ROS2 supports **managed nodes** with defined lifecycle states:

```
      ┌─────────────┐
      │ Unconfigured│
      └──────┬──────┘
             │configure
      ┌──────▼──────┐
      │  Inactive   │
      └──────┬──────┘
             │activate
      ┌──────▼──────┐
      │   Active    │◄── Normal operation
      └──────┬──────┘
             │deactivate
      ┌──────▼──────┐
      │  Inactive   │
      └──────┬──────┘
             │cleanup
      ┌──────▼──────┐
      │  Finalized  │
      └─────────────┘
```

## Quality of Service (QoS)

ROS2 allows fine-grained control over communication reliability:

### QoS Policies

| Policy | Options | Use Case |
|--------|---------|----------|
| **Reliability** | Reliable, Best Effort | Sensor data vs commands |
| **Durability** | Transient Local, Volatile | Late-joining subscribers |
| **History** | Keep Last N, Keep All | Buffer size |
| **Deadline** | Duration | Real-time requirements |
| **Lifespan** | Duration | Message expiration |
| **Liveliness** | Automatic, Manual | Node health monitoring |

### Example QoS Profiles

```cpp
// Sensor data (best effort, volatile)
auto sensor_qos = rclcpp::SensorDataQoS();

// System defaults (reliable, volatile)
auto default_qos = rclcpp::QoS(10);

// Custom QoS
auto custom_qos = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
```

## ROS2 Distributions

ROS2 releases follow a regular schedule:

| Distribution | Release | EOL | Support |
|--------------|---------|-----|---------|
| **Humble Hawksbill** | May 2022 | May 2027 | LTS ⭐ |
| Iron Irwini | May 2023 | Nov 2024 | Standard |
| Jazzy Jalisco | May 2024 | May 2026 | Standard |

**LTS (Long Term Support)** releases are recommended for production systems.

## Key ROS2 Commands

```bash
# Node management
ros2 node list              # List running nodes
ros2 node info /node_name   # Get node info

# Topic management
ros2 topic list             # List topics
ros2 topic echo /topic      # Show messages
ros2 topic hz /topic        # Message frequency
ros2 topic pub /topic ...   # Publish message

# Service management
ros2 service list           # List services
ros2 service call /srv ...  # Call service

# Parameter management
ros2 param list /node       # List parameters
ros2 param get /node param  # Get parameter
ros2 param set /node param value  # Set parameter

# Launch files
ros2 launch pkg file.py     # Launch multiple nodes

# Package management
ros2 pkg list               # List packages
ros2 pkg executables pkg    # List executables
```

## ROS2 Ecosystem

### Core Tools
- **rviz2**: 3D visualization
- **rqt**: Qt-based GUI tools
- **ros2bag**: Record and replay data
- **colcon**: Build tool

### Common Packages
- **tf2**: Coordinate transforms
- **nav2**: Navigation stack
- **MoveIt2**: Motion planning
- **ros2_control**: Control framework
- **image_pipeline**: Image processing

## Python vs C++

Both languages are first-class citizens in ROS2:

| Aspect | Python (rclpy) | C++ (rclcpp) |
|--------|----------------|--------------|
| **Performance** | Slower | Faster |
| **Development** | Faster | Slower |
| **Best for** | Prototyping, scripting | Production, real-time |
| **Learning curve** | Gentle | Steeper |

**This course focuses on C++ (rclcpp)** for production-ready systems.

## Summary

**Key Takeaways:**
- ROS2 is built on DDS (no master node needed)
- Nodes communicate via topics, services, and actions
- QoS provides fine-grained communication control
- Lifecycle management for production systems
- Multiple platforms and languages supported
- Humble (LTS) is recommended for production

**Critical Concepts:**
- **Nodes**: Independent processes
- **Topics**: Pub/sub messaging
- **Services**: Request/response
- **Actions**: Long-running tasks
- **Parameters**: Configuration
- **QoS**: Communication quality control

## What's Next?

- **Next Lesson**: [Creating Workspaces](02-creating-workspace.md)
- **Start Coding**: [First Node](03-first-node.md)

---

**You now understand ROS2 architecture!** Time to start building nodes.
