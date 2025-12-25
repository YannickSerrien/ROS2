# Service Example

This package demonstrates ROS2 service-based communication using service servers and clients.

## Overview

This package contains four executable nodes demonstrating different service patterns:
1. **add_server** - Simple service server for adding two integers
2. **add_client** - Service client with command-line arguments
3. **calculator_server** - Multi-service calculator with enable/disable
4. **robot_controller_server** - State machine-based robot controller

## Concepts Demonstrated

- Creating service servers with `create_service<>()`
- Creating service clients with `create_client<>()`
- Handling service requests and responses
- Synchronous service calls
- Waiting for service availability
- Multiple services in one node
- State management in services
- Input validation and error handling
- Using Trigger and SetBool service types

## Building

```bash
# From your workspace root
cd ~/ros2_ws

# Copy this package to src/ if needed
cp -r path/to/service_example src/

# Build
colcon build --packages-select service_example

# Source the workspace
source install/setup.bash
```

## Running the Examples

### Example 1: Basic Add Server and Client

**Terminal 1 - Start the server:**
```bash
ros2 run service_example add_server
```

Output:
```
[INFO] [add_server]: Add Two Ints service server is ready
[INFO] [add_server]: Call with: ros2 service call /add_two_ints ...
```

**Terminal 2 - Call service from command line:**
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

**Terminal 3 - Use the client node:**
```bash
ros2 run service_example add_client 10 20
```

Output:
```
[INFO] [add_client]: Waiting for service to be available...
[INFO] [add_client]: Service is available!
[INFO] [add_client]: Sending request: 10 + 20
[INFO] [add_client]: Received response: 30
[INFO] [add_client]: Result: 10 + 20 = 30
```

**Inspect services:**
```bash
# List all services
ros2 service list

# Get service type
ros2 service type /add_two_ints

# Find all services of a type
ros2 service find example_interfaces/srv/AddTwoInts
```

---

### Example 2: Multi-Service Calculator

**Terminal 1 - Start calculator server:**
```bash
ros2 run service_example calculator_server
```

**Terminal 2 - Use the calculator:**
```bash
# Add two numbers
ros2 service call /calculator/add example_interfaces/srv/AddTwoInts "{a: 15, b: 7}"

# Disable calculator
ros2 service call /calculator/enable std_srvs/srv/SetBool "{data: false}"

# Try to add (will warn that calculator is disabled)
ros2 service call /calculator/add example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# Re-enable calculator
ros2 service call /calculator/enable std_srvs/srv/SetBool "{data: true}"

# Add again (works now)
ros2 service call /calculator/add example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

---

### Example 3: Robot Controller with State Machine

**Terminal 1 - Start robot controller:**
```bash
ros2 run service_example robot_controller_server
```

**Terminal 2 - Control the robot:**

**Step 1: Home the robot** (transitions IDLE → HOMING → READY)
```bash
ros2 service call /robot/home std_srvs/srv/Trigger
```

**Step 2: Start robot motion** (transitions READY → MOVING)
```bash
ros2 service call /robot/start std_srvs/srv/Trigger
```

**Step 3: Stop robot** (transitions MOVING → READY)
```bash
ros2 service call /robot/stop std_srvs/srv/Trigger
```

**Test error handling:**
```bash
# Try to start without homing (will fail)
ros2 service call /robot/start std_srvs/srv/Trigger

# Activate emergency stop (transitions to ERROR)
ros2 service call /robot/estop std_srvs/srv/SetBool "{data: true}"

# Try to home (will fail - in ERROR state)
ros2 service call /robot/home std_srvs/srv/Trigger

# Reset from ERROR (transitions ERROR → IDLE)
ros2 service call /robot/reset std_srvs/srv/Trigger

# Now home again (works)
ros2 service call /robot/home std_srvs/srv/Trigger
```

**State transition diagram:**
```
IDLE → (home) → HOMING → READY → (start) → MOVING → (stop) → READY
  ↑                                                               ↓
  └─────────────────── (reset) ← ERROR ← (estop) ────────────────┘
```

---

## Code Structure

### add_server.cpp
```
AddServerNode
├── service_    (Service server shared pointer)
└── handle_add() (Request handler)
```

### add_client.cpp
```
AddClientNode
├── client_          (Service client shared pointer)
├── wait_for_service() (Wait for server)
└── send_request()   (Synchronous call)
```

### calculator_server.cpp
```
CalculatorServerNode
├── add_service_     (AddTwoInts service)
├── enable_service_  (SetBool service)
├── enabled_         (State flag)
├── handle_add()     (Add handler with state check)
└── handle_enable()  (Enable/disable handler)
```

### robot_controller_server.cpp
```
RobotControllerNode
├── home_service_    (Home command)
├── start_service_   (Start motion)
├── stop_service_    (Stop motion)
├── estop_service_   (Emergency stop)
├── reset_service_   (Reset from error)
├── state_           (Current robot state)
├── handle_home()    (Home with state validation)
├── handle_start()   (Start with state validation)
├── handle_stop()    (Stop with state validation)
├── handle_estop()   (E-stop activation)
└── handle_reset()   (Error recovery)
```

---

## Key Concepts

### Service Server Pattern
```cpp
// 1. Declare service server member
rclcpp::Service<ServiceType>::SharedPtr service_;

// 2. Create service in constructor
service_ = this->create_service<ServiceType>(
    "service_name",
    std::bind(&MyNode::handle_request, this, _1, _2));

// 3. Define request handler
void handle_request(
    const std::shared_ptr<ServiceType::Request> request,
    std::shared_ptr<ServiceType::Response> response)
{
    // Process request, fill response
    response->field = compute(request->input);
}
```

### Service Client Pattern (Synchronous)
```cpp
// 1. Declare client member
rclcpp::Client<ServiceType>::SharedPtr client_;

// 2. Create client
client_ = this->create_client<ServiceType>("service_name");

// 3. Wait for service
if (!client_->wait_for_service(timeout)) {
    // Handle timeout
}

// 4. Send request and wait for response
auto request = std::make_shared<ServiceType::Request>();
request->field = value;

auto future = client_->async_send_request(request);
auto result = rclcpp::spin_until_future_complete(node, future);

if (result == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    // Use response
}
```

### Common Service Types

**Trigger** (std_srvs/srv/Trigger):
```
# Request: empty
---
# Response:
bool success
string message
```

**SetBool** (std_srvs/srv/SetBool):
```
bool data
---
bool success
string message
```

**AddTwoInts** (example_interfaces/srv/AddTwoInts):
```
int64 a
int64 b
---
int64 sum
```

---

## Troubleshooting

**Issue: "Service not available"**
```bash
# Check if server is running
ros2 service list

# Check service type
ros2 service type /service_name

# Verify node is running
ros2 node list
```

**Issue: "Request timeout"**
- Server might be overloaded
- Network issues in distributed systems
- Increase timeout in client code

**Issue: "Wrong service type"**
```bash
# Verify server type
ros2 service type /service_name

# Should match client expectation
```

**Issue: State machine errors in robot_controller**
- Read the response message - it explains the error
- Check current state by looking at server logs
- Follow the correct sequence: IDLE → home → READY → start → MOVING

---

## Learning Exercises

1. **Add subtraction**: Add a `/calculator/subtract` service to calculator_server
2. **Async client**: Modify add_client to use asynchronous callback instead of `spin_until_future_complete`
3. **Status service**: Add a `/robot/status` service to robot_controller that returns current state
4. **Validation**: Add input validation to add_server (reject negative numbers)
5. **Timeout handling**: Add timeout to add_client with error message
6. **Multiple clients**: Create a client that calls both add and enable services

---

## Related Lessons

- [Lesson 6: Services Overview](../../lessons/06-services-overview.md)
- [Lesson 7: Service Servers](../../lessons/07-service-servers.md)
- [Lesson 8: Service Clients](../../lessons/08-service-clients.md)
- [Lesson 10: Logging](../../lessons/10-logging.md)

---

## Next Steps

After mastering this example:
1. Try the **parameters_example** package for runtime configuration
2. Experiment with custom service types (define your own .srv files)
3. Build a multi-step workflow using multiple services
4. Combine services with topics for hybrid communication patterns
5. Add timeout and retry logic to service clients
