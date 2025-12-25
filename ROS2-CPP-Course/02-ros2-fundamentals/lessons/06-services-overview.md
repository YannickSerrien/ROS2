# Lesson 6: Services - Overview

## Learning Objectives

- Understand the request-response pattern
- Compare services vs topics
- Learn service architecture
- Understand service types and definitions
- Know when to use services
- Explore common ROS2 services

## What are Services?

**Services** implement **request-response** communication in ROS2:

```
Client ──request──▶ Service Server ──response──▶ Client
       (blocking)                    (result)
```

### Key Characteristics

- **Synchronous**: Client waits for response
- **One-to-one**: Single client, single server
- **Blocking**: Client blocks until response (or timeout)
- **Bidirectional**: Request goes out, response comes back
- **Reliable**: Guaranteed delivery (QoS)

## Services vs Topics

### Comparison

| Feature | Topics | Services |
|---------|--------|----------|
| **Pattern** | Publish-Subscribe | Request-Response |
| **Communication** | Asynchronous | Synchronous |
| **Direction** | One-way | Two-way |
| **Cardinality** | Many-to-many | One-to-one |
| **Blocking** | Non-blocking | Blocking |
| **Use case** | Continuous data | On-demand queries |

### When to Use Services

✓ **Use Services for:**
- Configuration queries ("What's your current mode?")
- State changes ("Set motor speed to 100")
- Computations ("Calculate path from A to B")
- Remote procedure calls ("Reset odometry")
- One-time operations ("Take a picture")

✗ **Don't Use Services for:**
- High-frequency data (use topics)
- Sensor streams (use topics)
- Continuous commands (use topics)
- Many clients to one server (use topics)

## Service Architecture

### Client-Server Model

```
┌─────────────────────────────────────────────────┐
│                 ROS2 Network                     │
│                                                  │
│  ┌───────────┐                  ┌─────────────┐ │
│  │  Client   │                  │   Service   │ │
│  │           │─────request─────▶│   Server    │ │
│  │  (caller) │                  │  (handler)  │ │
│  │           │◀────response─────│             │ │
│  └───────────┘                  └─────────────┘ │
│                                                  │
└─────────────────────────────────────────────────┘
```

### Components

1. **Service Server**
   - Provides the service
   - Waits for requests
   - Processes and responds

2. **Service Client**
   - Calls the service
   - Sends request
   - Waits for response

3. **Service Name**
   - Unique identifier
   - Example: `/robot/set_speed`

4. **Service Type**
   - Defines request and response structure
   - Example: `std_srvs/srv/SetBool`

## Service Types

### Service Definition Structure

Every service type has **two parts**:

```
RequestType
---
ResponseType
```

### Example: AddTwoInts

```
# example_interfaces/srv/AddTwoInts.srv

int64 a
int64 b
---
int64 sum
```

- **Request**: Two integers (a, b)
- **Response**: One integer (sum)

### Common Service Types

#### std_srvs - Standard Services

**Empty** - No request, no response
```
# std_srvs/srv/Empty.srv
---
```

**SetBool** - Set a boolean value
```
# std_srvs/srv/SetBool.srv
bool data          # Request
---
bool success       # Response
string message
```

**Trigger** - Trigger an action
```
# std_srvs/srv/Trigger.srv
---
bool success
string message
```

#### example_interfaces - Example Services

**AddTwoInts** - Add numbers
```
# example_interfaces/srv/AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

### Service Type Naming

Format: `package/srv/TypeName`

```cpp
std_srvs/srv/SetBool
example_interfaces/srv/AddTwoInts
nav_msgs/srv/GetPlan
sensor_msgs/srv/SetCameraInfo
```

## Inspecting Services

### Command Line Tools

```bash
# List all active services
ros2 service list

# Show service type
ros2 service type /service_name

# Show service info
ros2 service info /service_name

# Call a service manually
ros2 service call /service_name service_type "request_data"

# Show service definition
ros2 interface show std_srvs/srv/SetBool
```

### Examples

```bash
# List services
ros2 service list
# Output:
# /add_two_ints
# /robot/set_mode
# /camera/trigger

# Check type
ros2 service type /add_two_ints
# Output: example_interfaces/srv/AddTwoInts

# View definition
ros2 interface show example_interfaces/srv/AddTwoInts
# Output:
# int64 a
# int64 b
# ---
# int64 sum

# Call service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
# Output:
# waiting for service to become available...
# response:
# sum: 8
```

## Service Workflow

### 1. Server Side (Provides Service)

```
┌──────────────────┐
│  Create Server   │
├──────────────────┤
│ Register service │
│ with name & type │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  Wait for        │
│  Requests        │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Request Received │
├──────────────────┤
│ Execute callback │
│ Process request  │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  Send Response   │
└──────────────────┘
```

### 2. Client Side (Calls Service)

```
┌──────────────────┐
│  Create Client   │
├──────────────────┤
│ Specify service  │
│ name & type      │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Wait for Server  │
│ to be available  │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  Send Request    │
├──────────────────┤
│ Fill request data│
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Wait for Response│
│ (blocking)       │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Process Response │
└──────────────────┘
```

## Service Example (Conceptual)

### Server (C++ Preview)

```cpp
class AddTwoIntsServer : public rclcpp::Node {
public:
    AddTwoIntsServer() : Node("add_two_ints_server") {
        // Create service
        service_ = create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServer::handle_service, this, _1, _2)
        );
    }

private:
    void handle_service(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        // Process request
        response->sum = request->a + request->b;

        RCLCPP_INFO(this->get_logger(), "Request: %ld + %ld = %ld",
            request->a, request->b, response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};
```

### Client (C++ Preview)

```cpp
class AddTwoIntsClient : public rclcpp::Node {
public:
    AddTwoIntsClient() : Node("add_two_ints_client") {
        // Create client
        client_ = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    void send_request(int64_t a, int64_t b) {
        // Wait for service
        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }

        // Create request
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        // Send async request
        auto future = client_->async_send_request(request);

        // Wait for response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Result: %ld", response->sum);
        }
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};
```

## Service Timeouts

### Problem: Server Not Available

```cpp
// Client blocks forever if server doesn't exist!
auto future = client_->async_send_request(request);
rclcpp::spin_until_future_complete(node, future);  // May block forever
```

### Solution: Timeout

```cpp
// Wait with timeout (1 second)
auto future = client_->async_send_request(request);
auto status = rclcpp::spin_until_future_complete(
    node,
    future,
    std::chrono::seconds(1)  // Timeout
);

if (status == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    // Process response
} else {
    RCLCPP_ERROR(node->get_logger(), "Service call timed out!");
}
```

## Service Patterns

### Pattern 1: Simple Query

**Use case:** Get current state

```
Request:  (empty)
Response: current_mode (string)
```

Example: "What mode is the robot in?"

### Pattern 2: Command with Confirmation

**Use case:** Set state and confirm

```
Request:  target_speed (double)
Response: success (bool), message (string)
```

Example: "Set speed to 1.5 m/s"

### Pattern 3: Computation

**Use case:** Calculate result

```
Request:  start (Point), goal (Point)
Response: path (Point[]), distance (double)
```

Example: "Calculate path from A to B"

### Pattern 4: Trigger Action

**Use case:** Start/stop operation

```
Request:  (empty or minimal)
Response: success (bool), message (string)
```

Example: "Reset odometry"

## Common ROS2 Services

### Robot Control

```bash
# Set parameter
/robot/set_parameters

# Get parameter
/robot/get_parameters

# Reset state
/robot/reset
```

### Sensors

```bash
# Trigger camera
/camera/trigger_capture

# Configure sensor
/lidar/set_scan_rate
```

### Navigation

```bash
# Get map
/map_server/get_map

# Compute path
/nav2/compute_path

# Clear costmap
/nav2/clear_costmap
```

## Service QoS

Services use **reliable** QoS by default:

```cpp
// Services always use reliable communication
// No message loss
// Delivery guaranteed
```

Unlike topics, you can't configure service QoS (it's always reliable).

## Best Practices

### DO ✓

```cpp
// Use descriptive service names
create_service<std_srvs::srv::SetBool>("/robot/emergency_stop", ...);

// Always wait for service availability
while (!client->wait_for_service(1s)) {
    RCLCPP_INFO(logger, "Waiting for service...");
}

// Use timeouts to prevent blocking
rclcpp::spin_until_future_complete(node, future, 5s);

// Provide meaningful response messages
response->success = true;
response->message = "Mode changed to AUTO";
```

### DON'T ✗

```cpp
// Don't use services for high-frequency data
// Use topics instead!

// Don't call services in time-critical code
void emergency_stop() {
    client->call_service(...);  // Too slow! Use topic
}

// Don't create service clients in callbacks
void callback() {
    auto client = create_client<...>(...);  // Bad!
}

// Don't block spin() with long-running service handlers
void handle_request(...) {
    expensive_computation();  // Blocks other callbacks!
}
```

## Services vs Actions

For **long-running tasks**, use **actions** instead:

| Feature | Services | Actions |
|---------|----------|---------|
| **Duration** | Short (< 1 second) | Long (seconds to minutes) |
| **Feedback** | No | Yes (progress updates) |
| **Cancellation** | No | Yes |
| **Preemption** | No | Yes |
| **Best for** | Quick queries | Navigation, manipulation |

**Examples:**
- Service: "Get current position" (instant)
- Action: "Navigate to waypoint" (takes time, provides feedback)

## Debugging Services

### Check Service Availability

```bash
# List all services
ros2 service list

# Check if specific service exists
ros2 service list | grep my_service

# Show service type
ros2 service type /my_service

# Test service call
ros2 service call /my_service std_srvs/srv/Trigger
```

### Common Issues

**Issue 1: Service not found**
```bash
# Verify service name
ros2 service list

# Check server is running
ros2 node list
```

**Issue 2: Wrong service type**
```bash
# Check type matches
ros2 service type /my_service
ros2 interface show std_srvs/srv/SetBool
```

**Issue 3: Server not responding**
```bash
# Test with command line
ros2 service call /my_service std_srvs/srv/Trigger

# If timeout, server may be stuck
```

## Visualizing Services

### Using `rqt_graph`

```bash
rqt_graph

# Services shown as separate connections
# Different from topic connections
```

### Using `ros2 node info`

```bash
ros2 node info /my_node

# Shows:
# - Services provided
# - Services called
```

## Summary

**Key Takeaways:**
- Services implement request-response communication
- Synchronous and blocking (client waits)
- Use for queries, commands, and computations
- Don't use for continuous data (use topics)
- Always handle timeouts
- Service types define request and response structure

**Service Structure:**
```
Client ──request──▶ Server
Client ◀─response── Server
```

**When to Use:**
- ✓ One-time queries
- ✓ Configuration changes
- ✓ Computations
- ✗ High-frequency data
- ✗ Continuous streams

## Practice Exercise

Using command-line tools:
1. List all services in a running ROS2 system
2. Find a service and check its type
3. View the service definition
4. Call a service manually
5. Observe the response

## What's Next?

- **Next Lesson**: [Service Servers](07-service-servers.md) - Implement service providers
- **After That**: [Service Clients](08-service-clients.md) - Call services
- **Advanced**: [Actions](../03-ros2-intermediate/04-actions.md) - Long-running tasks

---

**You now understand ROS2 services!** Next: implementing your own service servers.
