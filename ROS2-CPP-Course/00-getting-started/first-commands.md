# ROS2 First Commands - Cheat Sheet

Quick reference guide for essential ROS2 command-line interface (CLI) commands.

## Table of Contents

- [Getting Help](#getting-help)
- [Node Commands](#node-commands)
- [Topic Commands](#topic-commands)
- [Service Commands](#service-commands)
- [Parameter Commands](#parameter-commands)
- [Action Commands](#action-commands)
- [Package Commands](#package-commands)
- [Launch Commands](#launch-commands)
- [Bag Commands](#bag-commands)
- [Introspection Tools](#introspection-tools)
- [Daemon Commands](#daemon-commands)

---

## Getting Help

```bash
# General help
ros2 --help

# Help for specific command
ros2 <command> --help

# Example:
ros2 node --help

# Get ROS2 version
ros2 --version

# List all available commands
ros2
```

---

## Node Commands

Nodes are executable processes that perform computation.

### List and Inspect Nodes

```bash
# List all running nodes
ros2 node list

# Get info about a node
ros2 node info /node_name

# Example with turtlesim
ros2 run turtlesim turtlesim_node  # Run in terminal 1
ros2 node list                      # In terminal 2
ros2 node info /turtlesim
```

### Run Nodes

```bash
# Run a node
ros2 run <package_name> <executable_name>

# Example:
ros2 run demo_nodes_cpp talker

# Run with custom node name
ros2 run <package> <executable> --ros-args --remap __node:=<new_name>

# Example:
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

### Node Information Details

```bash
ros2 node info /node_name
```

Output includes:
- Subscribers
- Publishers
- Services
- Actions
- Clients

---

## Topic Commands

Topics are named channels for message passing (pub/sub pattern).

### List and Info

```bash
# List all topics
ros2 topic list

# List with message types
ros2 topic list -t

# Get topic info
ros2 topic info /topic_name

# Get topic type
ros2 topic type /topic_name

# Find topics by type
ros2 topic find <message_type>
```

### Publish and Subscribe

```bash
# Echo messages from a topic
ros2 topic echo /topic_name

# Echo with limited messages
ros2 topic echo /topic_name --once
ros2 topic echo /topic_name --max-count 10

# Publish to a topic
ros2 topic pub /topic_name <message_type> '<message_data>'

# Example - publish once
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# Example - publish continuously at 1 Hz
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 2.0}, angular: {z: 1.8}}"
```

### Topic Statistics

```bash
# Get topic frequency
ros2 topic hz /topic_name

# Get topic bandwidth
ros2 topic bw /topic_name

# Delay measurement (requires header)
ros2 topic delay /topic_name
```

### Topic Type Details

```bash
# Show message structure
ros2 interface show <message_type>

# Example:
ros2 interface show geometry_msgs/msg/Twist

# Show all message types
ros2 interface list
```

---

## Service Commands

Services provide request/response communication (client/server pattern).

### List and Info

```bash
# List all services
ros2 service list

# List with service types
ros2 service list -t

# Get service type
ros2 service type /service_name

# Find services by type
ros2 service find <service_type>
```

### Call Services

```bash
# Call a service
ros2 service call /service_name <service_type> '<arguments>'

# Example - spawn a turtle
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"

# Example - clear turtlesim
ros2 service call /clear std_srvs/srv/Empty

# Example - add two integers
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

### Service Type Details

```bash
# Show service structure
ros2 interface show <service_type>

# Example:
ros2 interface show turtlesim/srv/Spawn
```

---

## Parameter Commands

Parameters are node configuration values.

### List and Get

```bash
# List all parameters for a node
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name parameter_name

# Example:
ros2 param get /turtlesim background_r

# Dump all parameters
ros2 param dump /node_name

# Save to file
ros2 param dump /node_name > params.yaml
```

### Set Parameters

```bash
# Set parameter value
ros2 param set /node_name parameter_name value

# Example:
ros2 param set /turtlesim background_r 150

# Load parameters from file
ros2 param load /node_name params.yaml
```

### Parameter Types

```bash
# Describe parameter
ros2 param describe /node_name parameter_name
```

---

## Action Commands

Actions provide goal-oriented communication with feedback and cancellation.

### List and Info

```bash
# List all actions
ros2 action list

# List with action types
ros2 action list -t

# Get action info
ros2 action info /action_name
```

### Send Goals

```bash
# Send action goal
ros2 action send_goal /action_name <action_type> '<goal>'

# Example - navigate turtle
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"

# Send goal with feedback
ros2 action send_goal --feedback /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}"
```

### Action Type Details

```bash
# Show action structure
ros2 interface show <action_type>

# Example:
ros2 interface show turtlesim/action/RotateAbsolute
```

---

## Package Commands

Packages are the organizational unit of ROS2 code.

### List and Find

```bash
# List all packages
ros2 pkg list

# Find package path
ros2 pkg prefix <package_name>

# Example:
ros2 pkg prefix turtlesim

# List package executables
ros2 pkg executables <package_name>

# List all executables
ros2 pkg executables
```

### Create Packages

```bash
# Create C++ package
ros2 pkg create --build-type ament_cmake <package_name> \
    --dependencies <dep1> <dep2>

# Example:
ros2 pkg create --build-type ament_cmake my_robot_controller \
    --dependencies rclcpp std_msgs geometry_msgs

# Create Python package
ros2 pkg create --build-type ament_python <package_name> \
    --dependencies <dep1> <dep2>

# Example:
ros2 pkg create --build-type ament_python my_py_pkg \
    --dependencies rclpy std_msgs
```

---

## Launch Commands

Launch files start multiple nodes with configuration.

### Run Launch Files

```bash
# Launch a launch file
ros2 launch <package_name> <launch_file>

# Example:
ros2 launch turtlesim multisim.launch.py

# Launch with arguments
ros2 launch <package> <launch_file> <arg>:=<value>

# Example:
ros2 launch my_robot robot.launch.py use_sim:=true

# Launch file directly (not from package)
ros2 launch /path/to/launch/file.launch.py
```

---

## Bag Commands

Bags record and replay ROS2 data.

### Record Data

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /topic1 /topic2

# Record to specific directory
ros2 bag record -o my_bag /topic_name

# Example:
ros2 bag record -o turtle_recording /turtle1/cmd_vel /turtle1/pose
```

### Play and Inspect

```bash
# Get bag info
ros2 bag info <bag_directory>

# Example:
ros2 bag info turtle_recording

# Play bag
ros2 bag play <bag_directory>

# Play at different rate
ros2 bag play --rate 2.0 <bag_directory>  # 2x speed

# Play in loop
ros2 bag play --loop <bag_directory>

# Play single message
ros2 bag play --max-count 1 <bag_directory>
```

---

## Introspection Tools

### RQt Tools

```bash
# RQt main window (plugin container)
rqt

# RQt graph (node/topic visualization)
rqt_graph

# RQt console (log viewer)
rqt_console

# RQt topic viewer
rqt_topic

# RQt service caller
rqt_service_caller

# RQt parameter reconfigure
rqt_reconfigure

# RQt plot
rqt_plot /topic/field

# Example:
rqt_plot /turtle1/pose/x /turtle1/pose/y
```

### TF2 Tools

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo transform
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>

# Example:
ros2 run tf2_ros tf2_echo world base_link
```

### Visualization

```bash
# RViz2
rviz2

# RViz2 with config
rviz2 -d /path/to/config.rviz
```

---

## Daemon Commands

The ROS2 daemon provides discovery and caching.

```bash
# Start daemon
ros2 daemon start

# Stop daemon
ros2 daemon stop

# Check daemon status
ros2 daemon status

# Restart daemon (useful when nodes don't appear)
ros2 daemon stop
ros2 daemon start
```

---

## Useful Command Combinations

### Debug a Running System

```bash
# In separate terminals:

# 1. List running nodes
ros2 node list

# 2. Inspect a node
ros2 node info /node_name

# 3. Check topics
ros2 topic list
ros2 topic hz /topic_name
ros2 topic echo /topic_name

# 4. Visualize graph
rqt_graph
```

### Test Communication

```bash
# Terminal 1: Publisher
ros2 topic pub /test std_msgs/msg/String "data: 'Hello ROS2'"

# Terminal 2: Subscriber
ros2 topic echo /test
```

### Record and Replay

```bash
# Terminal 1: Record
ros2 bag record -a

# ... do stuff ...

# Stop recording (Ctrl+C)

# Terminal 2: Replay
ros2 bag play <bag_name>
```

---

## Environment Variables

Useful ROS2 environment variables:

```bash
# ROS2 distribution
echo $ROS_DISTRO

# Domain ID (for multi-robot)
export ROS_DOMAIN_ID=42

# Localhost only (security/performance)
export ROS_LOCALHOST_ONLY=1

# Log level
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

# Colored output
export RCUTILS_COLORIZED_OUTPUT=1
```

---

## Pro Tips

### Tab Completion

Enable auto-complete:
```bash
# Already set up if you followed installation guide
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

Then use TAB to complete:
```bash
ros2 <TAB><TAB>
ros2 topic <TAB><TAB>
ros2 topic echo /<TAB><TAB>
```

### Aliases

Add to `~/.bashrc`:

```bash
# ROS2 aliases
alias r2='ros2'
alias r2n='ros2 node'
alias r2t='ros2 topic'
alias r2s='ros2 service'
alias r2run='ros2 run'
alias r2launch='ros2 launch'

# Shortcuts
alias tl='ros2 topic list'
alias nl='ros2 node list'
alias sl='ros2 service list'

# Source workspace
alias sw='source ~/ros2_ws/install/setup.bash'
```

### Quick Scripts

Create `~/bin/ros2_info.sh`:

```bash
#!/bin/bash
echo "=== Nodes ==="
ros2 node list

echo -e "\n=== Topics ==="
ros2 topic list

echo -e "\n=== Services ==="
ros2 service list
```

Make executable:
```bash
chmod +x ~/bin/ros2_info.sh
```

---

## Common Workflows

### Starting a Project

```bash
# 1. Source ROS2
source /opt/ros/humble/setup.bash

# 2. Source workspace
source ~/ros2_ws/install/setup.bash

# 3. Run your nodes
ros2 run my_package my_node
```

### Debugging Issues

```bash
# Check if nodes are running
ros2 node list

# Check if topics are publishing
ros2 topic hz /topic_name

# See messages
ros2 topic echo /topic_name

# Visualize
rqt_graph

# Check logs
rqt_console
```

---

## Quick Reference Card

Print this section!

```
╔══════════════════════════════════════════════════════════╗
║              ROS2 QUICK REFERENCE                         ║
╠══════════════════════════════════════════════════════════╣
║ NODES                                                     ║
║  ros2 node list             List nodes                   ║
║  ros2 node info /node       Node details                 ║
║  ros2 run pkg exe           Run node                     ║
╠══════════════════════════════════════════════════════════╣
║ TOPICS                                                    ║
║  ros2 topic list            List topics                  ║
║  ros2 topic echo /topic     Show messages                ║
║  ros2 topic hz /topic       Message rate                 ║
║  ros2 topic pub /topic ...  Publish message              ║
╠══════════════════════════════════════════════════════════╣
║ SERVICES                                                  ║
║  ros2 service list          List services                ║
║  ros2 service call /srv ... Call service                 ║
╠══════════════════════════════════════════════════════════╣
║ PARAMETERS                                                ║
║  ros2 param list /node      List params                  ║
║  ros2 param get /node p     Get param                    ║
║  ros2 param set /node p v   Set param                    ║
╠══════════════════════════════════════════════════════════╣
║ LAUNCH & BUILD                                            ║
║  ros2 launch pkg file.py    Run launch file              ║
║  colcon build               Build workspace              ║
║  source install/setup.bash  Source workspace             ║
╠══════════════════════════════════════════════════════════╣
║ TOOLS                                                     ║
║  rqt_graph                  Visualize nodes/topics       ║
║  rviz2                      Visualization                ║
║  rqt_console                Log viewer                   ║
╚══════════════════════════════════════════════════════════╝
```

---

## Next Steps

Familiar with commands? Excellent!

1. ✓ Know essential ROS2 commands
2. ✓ Can inspect running systems
3. → Start learning C++: [Module 1: C++ Refresher](../01-cpp-refresher/)
4. → Bookmark this page for quick reference!

---

**Keep this cheat sheet handy** - you'll use these commands throughout your ROS2 journey!
