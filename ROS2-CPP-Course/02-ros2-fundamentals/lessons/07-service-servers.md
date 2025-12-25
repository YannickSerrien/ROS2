# Lesson 7: Service Servers

## Learning Objectives

- Create ROS2 service servers
- Handle service requests with callbacks
- Define custom service types
- Implement different service patterns
- Handle errors and edge cases
- Test service servers
- Apply best practices

## What is a Service Server?

A **service server** provides a service - it waits for requests and sends back responses:

```
Server: "I can add two numbers"
         ↓
Client: "Please add 5 + 3"
         ↓
Server: "Result is 8"
```

## Your First Service Server

### Minimal Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

class MinimalServer : public rclcpp::Node {
public:
    MinimalServer() : Node("minimal_server") {
        // Create service server
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&MinimalServer::handle_request, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Service server ready");
    }

private:
    void handle_request(
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalServer>());
    rclcpp::shutdown();
    return 0;
}
```

**Output when called:**
```
[INFO] [minimal_server]: Service server ready
[INFO] [minimal_server]: Request: 5 + 3 = 8
[INFO] [minimal_server]: Request: 10 + 20 = 30
```

### Breaking It Down

#### 1. Include Service Header

```cpp
#include <example_interfaces/srv/add_two_ints.hpp>
```

Format: `<package>/srv/<type>.hpp`

#### 2. Create Service

```cpp
service_ = create_service<ServiceType>(
    "service_name",
    callback_function
);
```

**Parameters:**
- **Template**: Service type (e.g., `AddTwoInts`)
- **Name**: Service name (e.g., `"add_two_ints"`)
- **Callback**: Function to handle requests

#### 3. Service Callback Signature

```cpp
void handle_request(
    const std::shared_ptr<ServiceType::Request> request,
    std::shared_ptr<ServiceType::Response> response)
{
    // Read request data
    auto input = request->data;

    // Compute result
    // ...

    // Fill response
    response->result = computed_value;
}
```

**Key points:**
- Request is `const` (read-only)
- Response is mutable (fill it)
- Both are `shared_ptr`

#### 4. Bind Callback

```cpp
std::bind(&ClassName::method, this, std::placeholders::_1, std::placeholders::_2)
```

- `_1` - Request parameter
- `_2` - Response parameter

## Complete Service Server Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <memory>

class AddTwoIntsServer : public rclcpp::Node {
public:
    AddTwoIntsServer() : Node("add_two_ints_server"), request_count_(0) {
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServer::add_callback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "AddTwoInts service ready");
    }

private:
    void add_callback(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        request_count_++;

        RCLCPP_INFO(this->get_logger(),
                    "Request #%d: %ld + %ld",
                    request_count_, request->a, request->b);

        // Perform computation
        response->sum = request->a + request->b;

        RCLCPP_INFO(this->get_logger(), "Sending response: %ld", response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
    int request_count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Using Different Service Types

### SetBool Service

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

class ModeServer : public rclcpp::Node {
public:
    ModeServer() : Node("mode_server"), enabled_(false) {
        service_ = create_service<std_srvs::srv::SetBool>(
            "set_mode",
            std::bind(&ModeServer::set_mode_callback, this, _1, _2)
        );
    }

private:
    void set_mode_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // Update state
        enabled_ = request->data;

        // Fill response
        response->success = true;
        response->message = enabled_ ? "Mode ENABLED" : "Mode DISABLED";

        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    bool enabled_;
};
```

**Test it:**
```bash
# Enable
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"

# Output:
# response:
#   success: true
#   message: 'Mode ENABLED'
```

### Trigger Service

```cpp
#include <std_srvs/srv/trigger.hpp>

class ResetServer : public rclcpp::Node {
public:
    ResetServer() : Node("reset_server"), counter_(0) {
        service_ = create_service<std_srvs::srv/Trigger>(
            "reset_counter",
            std::bind(&ResetServer::reset_callback, this, _1, _2)
        );
    }

private:
    void reset_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // Trigger has empty request
        (void)request;  // Unused

        // Perform reset
        int old_value = counter_;
        counter_ = 0;

        // Response
        response->success = true;
        response->message = "Counter reset from " + std::to_string(old_value) + " to 0";

        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    int counter_;
};
```

## Lambda Callbacks

Use lambdas for simpler services:

```cpp
class SimpleServer : public rclcpp::Node {
public:
    SimpleServer() : Node("simple_server") {
        service_ = create_service<std_srvs::srv::SetBool>(
            "enable",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response)
            {
                enabled_ = request->data;
                response->success = true;
                response->message = enabled_ ? "ON" : "OFF";
                RCLCPP_INFO(this->get_logger(), "Set to %s", response->message.c_str());
            }
        );
    }

private:
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    bool enabled_ = false;
};
```

## Error Handling

### Validate Input

```cpp
void callback(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
    // Check for overflow
    if (request->a > 0 && request->b > 0 &&
        request->a > std::numeric_limits<int64_t>::max() - request->b)
    {
        RCLCPP_ERROR(this->get_logger(), "Integer overflow!");
        response->sum = 0;  // Or handle differently
        return;
    }

    response->sum = request->a + request->b;
}
```

### Success/Failure Responses

```cpp
void set_speed_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    // Validate
    if (!robot_ready_) {
        response->success = false;
        response->message = "Robot not ready";
        RCLCPP_WARN(this->get_logger(), "Service call rejected: robot not ready");
        return;
    }

    // Execute
    enabled_ = request->data;
    response->success = true;
    response->message = "Speed set successfully";
}
```

## Multiple Service Servers

One node can provide multiple services:

```cpp
class RobotServer : public rclcpp::Node {
public:
    RobotServer() : Node("robot_server") {
        // Service 1: Enable/disable
        enable_service_ = create_service<std_srvs::srv::SetBool>(
            "enable",
            std::bind(&RobotServer::enable_callback, this, _1, _2)
        );

        // Service 2: Reset
        reset_service_ = create_service<std_srvs::srv::Trigger>(
            "reset",
            std::bind(&RobotServer::reset_callback, this, _1, _2)
        );

        // Service 3: Get status
        status_service_ = create_service<std_srvs::srv::Trigger>(
            "get_status",
            std::bind(&RobotServer::status_callback, this, _1, _2)
        );

        RCLCPP_INFO(this->get_logger(), "Robot server ready with 3 services");
    }

private:
    void enable_callback(...) { /* ... */ }
    void reset_callback(...) { /* ... */ }
    void status_callback(...) { /* ... */ }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
};
```

## Service with State

Service that maintains and modifies state:

```cpp
class CounterServer : public rclcpp::Node {
public:
    CounterServer() : Node("counter_server"), counter_(0) {
        increment_srv_ = create_service<std_srvs::srv::Trigger>(
            "increment",
            [this](auto req, auto res) {
                (void)req;
                counter_++;
                res->success = true;
                res->message = "Counter: " + std::to_string(counter_);
            }
        );

        decrement_srv_ = create_service<std_srvs::srv::Trigger>(
            "decrement",
            [this](auto req, auto res) {
                (void)req;
                counter_--;
                res->success = true;
                res->message = "Counter: " + std::to_string(counter_);
            }
        );

        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "reset",
            [this](auto req, auto res) {
                (void)req;
                counter_ = 0;
                res->success = true;
                res->message = "Counter reset to 0";
            }
        );
    }

private:
    int counter_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr increment_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr decrement_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};
```

## Performance Considerations

### Fast Callbacks

Service callbacks should be **fast**:

```cpp
// GOOD - Fast processing
void callback(auto request, auto response) {
    response->result = request->a + request->b;  // Fast
}

// BAD - Slow processing
void callback(auto request, auto response) {
    expensive_computation();  // Blocks other services!
    response->result = result;
}
```

### Long Operations

For slow operations, consider:

```cpp
void callback(auto request, auto response) {
    // Option 1: Quick validation, queue work
    if (!validate(request)) {
        response->success = false;
        return;
    }

    work_queue_.push(request);
    response->success = true;
    response->message = "Request queued";
}

// Option 2: Use actions instead (for very long tasks)
```

## Service Naming

### Best Practices

```cpp
// Good names (descriptive, namespaced)
create_service<...>("/robot/set_speed", ...);
create_service<...>("/camera/trigger_capture", ...);
create_service<...>("/nav/clear_costmap", ...);

// Bad names (vague, no namespace)
create_service<...>("service1", ...);
create_service<...>("do_thing", ...);
```

### Namespace Services

```cpp
class SensorNode : public rclcpp::Node {
public:
    SensorNode() : Node("sensor_node") {
        // Service automatically namespaced under node namespace
        service_ = create_service<std_srvs::srv::SetBool>(
            "enable",  // Becomes /sensor_node/enable
            callback
        );
    }
};

// Or set namespace explicitly
auto node = std::make_shared<rclcpp::Node>("node", "robot1");
// Services will be under /robot1/
```

## Testing Service Servers

### Manual Testing

```bash
# 1. Run server
ros2 run my_package server_node

# 2. List services
ros2 service list
# /add_two_ints

# 3. Check type
ros2 service type /add_two_ints
# example_interfaces/srv/AddTwoInts

# 4. Call service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# 5. Check response
# response:
#   sum: 8
```

### Debugging

```bash
# Check if service exists
ros2 service list | grep my_service

# View service type definition
ros2 interface show std_srvs/srv/SetBool

# Call with verbose output
ros2 service call /my_service std_srvs/srv/Trigger --verbose

# Check node info
ros2 node info /server_node
# Services:
#   /my_service
```

## Best Practices

### DO ✓

```cpp
// Validate inputs
void callback(auto request, auto response) {
    if (request->speed < 0 || request->speed > max_speed_) {
        response->success = false;
        response->message = "Invalid speed";
        return;
    }
    // Process
}

// Provide meaningful messages
response->message = "Mode changed to AUTO, previous was MANUAL";

// Log service calls
RCLCPP_INFO(this->get_logger(), "Service called with data: %d", request->value);

// Keep callbacks fast
void callback(auto req, auto res) {
    // Quick validation and response
    res->success = true;
}
```

### DON'T ✗

```cpp
// Don't do long computations
void callback(auto req, auto res) {
    complex_calculation();  // Blocks!
}

// Don't throw exceptions (handle gracefully)
void callback(auto req, auto res) {
    if (error) {
        throw std::runtime_error("Error");  // Bad!
    }
    // Better:
    res->success = false;
    res->message = "Error occurred";
}

// Don't access members without thread safety
void callback(auto req, auto res) {
    shared_data_ = req->value;  // Race condition if using multi-threaded executor!
}
```

## Complete Example: Robot Mode Server

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

class RobotModeServer : public rclcpp::Node {
public:
    RobotModeServer() : Node("robot_mode_server"), mode_("MANUAL") {
        // Enable/disable autonomous mode
        set_mode_srv_ = create_service<std_srvs::srv::SetBool>(
            "set_autonomous",
            std::bind(&RobotModeServer::set_mode, this, _1, _2)
        );

        // Get current mode
        get_mode_srv_ = create_service<std_srvs::srv::Trigger>(
            "get_mode",
            std::bind(&RobotModeServer::get_mode, this, _1, _2)
        );

        RCLCPP_INFO(this->get_logger(), "Robot mode server ready. Current mode: %s", mode_.c_str());
    }

private:
    void set_mode(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::string old_mode = mode_;
        mode_ = request->data ? "AUTONOMOUS" : "MANUAL";

        response->success = true;
        response->message = "Mode changed from " + old_mode + " to " + mode_;

        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void get_mode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        response->success = true;
        response->message = "Current mode: " + mode_;

        RCLCPP_INFO(this->get_logger(), "Mode queried: %s", mode_.c_str());
    }

    std::string mode_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_mode_srv_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotModeServer>());
    rclcpp::shutdown();
    return 0;
}
```

## Summary

**Key Takeaways:**
- Service servers provide services that clients can call
- Use `create_service<Type>("name", callback)`
- Callback receives request (const) and response (mutable)
- Keep callbacks fast - no blocking operations
- Always validate inputs and provide meaningful responses
- One node can provide multiple services

**Critical Pattern:**
```cpp
service_ = create_service<ServiceType>(
    "service_name",
    std::bind(&Class::callback, this, _1, _2)
);

void callback(
    const std::shared_ptr<ServiceType::Request> request,
    std::shared_ptr<ServiceType::Response> response)
{
    // Read request->data
    // Fill response->result
}
```

## Practice Exercise

Create a calculator server that provides four services:
1. `add` - Add two numbers
2. `subtract` - Subtract two numbers
3. `multiply` - Multiply two numbers
4. `divide` - Divide (handle division by zero!)

Use `example_interfaces/srv/AddTwoInts` for all services.

## What's Next?

- **Next Lesson**: [Service Clients](08-service-clients.md) - Call services from code
- **Related**: [Parameters](09-parameters.md) - Alternative to configuration services
- **Advanced**: [Actions](../03-ros2-intermediate/04-actions.md) - Long-running tasks

---

**You can now create service servers!** Next: calling them with service clients.
