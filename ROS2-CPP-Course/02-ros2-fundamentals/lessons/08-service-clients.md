# Lesson 8: Service Clients

## Learning Objectives

- Create ROS2 service clients
- Call services synchronously and asynchronously
- Handle service responses and timeouts
- Understand callback-based service calls
- Implement error handling
- Test service clients
- Apply best practices

## What is a Service Client?

A **service client** calls services provided by service servers:

```
Client: create_client → wait_for_service → send_request → get_response
Server:                 waiting...         process       send back
```

## Your First Service Client

### Minimal Example (Synchronous)

```cpp
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MinimalClient : public rclcpp::Node {
public:
    MinimalClient() : Node("minimal_client") {
        // Create client
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    void send_request(int64_t a, int64_t b) {
        // Wait for service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }

        // Create request
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        // Send request (async)
        auto future = client_->async_send_request(request);

        // Wait for response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Result: %ld + %ld = %ld",
                        a, b, response->sum);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalClient>();
    node->send_request(5, 3);
    rclcpp::shutdown();
    return 0;
}
```

**Output:**
```
[INFO] [minimal_client]: Waiting for service...
[INFO] [minimal_client]: Result: 5 + 3 = 8
```

### Breaking It Down

#### 1. Create Client

```cpp
client_ = create_client<ServiceType>("service_name");
```

**Parameters:**
- **Template**: Service type
- **Name**: Service name (must match server)

#### 2. Wait for Service

```cpp
while (!client_->wait_for_service(1s)) {
    RCLCPP_INFO(logger, "Waiting...");
}
```

- Waits up to 1 second
- Returns `true` if service available
- Loop until available or shutdown

#### 3. Create Request

```cpp
auto request = std::make_shared<ServiceType::Request>();
request->field1 = value1;
request->field2 = value2;
```

- Use `make_shared` for efficient memory management
- Fill request fields

#### 4. Send Request (Async)

```cpp
auto future = client_->async_send_request(request);
```

- Returns `std::future` immediately
- Actual call happens asynchronously
- Non-blocking

#### 5. Wait for Response

```cpp
auto result = rclcpp::spin_until_future_complete(node, future);
if (result == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    // Use response
}
```

- Blocks until response received or timeout
- Check return code for success

## Asynchronous Clients (Callback-Based)

Better approach for nodes with other work:

```cpp
class AsyncClient : public rclcpp::Node {
public:
    AsyncClient() : Node("async_client") {
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

        // Send with callback
        auto response_callback = [this, a, b](
            rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Result: %ld + %ld = %ld",
                        a, b, response->sum);
        };

        client_->async_send_request(request, response_callback);
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AsyncClient>();
    node->send_request(10, 20);

    // Node can do other work while waiting
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
```

**Advantages of Async with Callback:**
- ✓ Node remains responsive
- ✓ Can handle multiple concurrent requests
- ✓ Non-blocking
- ✓ Better for real-time systems

## Complete Service Client Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <chrono>

using namespace std::chrono_literals;

class RobotControlClient : public rclcpp::Node {
public:
    RobotControlClient() : Node("robot_control_client") {
        client_ = create_client<std_srvs::srv::SetBool>("set_autonomous");
        RCLCPP_INFO(this->get_logger(), "Robot control client ready");
    }

    bool set_autonomous_mode(bool enable) {
        // Check if service is available
        if (!client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Service not available!");
            return false;
        }

        // Create request
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = enable;

        RCLCPP_INFO(this->get_logger(), "Calling service: set_autonomous(%s)",
                    enable ? "true" : "false");

        // Send request
        auto future = client_->async_send_request(request);

        // Wait for response (with timeout)
        auto result = rclcpp::spin_until_future_complete(
            this->get_node_base_interface(),
            future,
            5s  // 5 second timeout
        );

        if (result == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response: %s - %s",
                        response->success ? "SUCCESS" : "FAILURE",
                        response->message.c_str());
            return response->success;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out");
            return false;
        }
    }

private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControlClient>();

    // Call service
    bool success = node->set_autonomous_mode(true);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "Successfully enabled autonomous mode");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to enable autonomous mode");
    }

    rclcpp::shutdown();
    return 0;
}
```

## Timeout Handling

### With Timeout

```cpp
void call_service_with_timeout() {
    auto request = std::make_shared<ServiceType::Request>();
    auto future = client_->async_send_request(request);

    // Wait with 2 second timeout
    auto status = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        future,
        std::chrono::seconds(2)
    );

    switch (status) {
        case rclcpp::FutureReturnCode::SUCCESS:
            RCLCPP_INFO(this->get_logger(), "Service call succeeded");
            auto response = future.get();
            // Process response
            break;

        case rclcpp::FutureReturnCode::TIMEOUT:
            RCLCPP_ERROR(this->get_logger(), "Service call timed out");
            break;

        case rclcpp::FutureReturnCode::INTERRUPTED:
            RCLCPP_ERROR(this->get_logger(), "Service call interrupted");
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
            break;
    }
}
```

### Without Blocking spin()

For nodes that need to remain responsive:

```cpp
void call_service_non_blocking() {
    auto request = std::make_shared<ServiceType::Request>();

    // Send request with callback
    auto callback = [this](auto future) {
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Got response: %s", response->message.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    };

    client_->async_send_request(request, callback);

    // Node continues processing other callbacks
    // Response callback will be called when response arrives
}
```

## Different Service Types

### SetBool

```cpp
void call_set_bool(bool value) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = value;

    auto future = client_->async_send_request(request);
    // Wait and process...
}
```

### Trigger

```cpp
void call_trigger() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // Request is empty for Trigger

    auto future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        RCLCPP_INFO(logger, "%s: %s",
                    response->success ? "SUCCESS" : "FAILURE",
                    response->message.c_str());
    }
}
```

### AddTwoInts

```cpp
int64_t add_numbers(int64_t a, int64_t b) {
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    auto future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        return future.get()->sum;
    }

    return 0;  // Error value
}
```

## Multiple Service Clients

One node can call multiple services:

```cpp
class MultiServiceClient : public rclcpp::Node {
public:
    MultiServiceClient() : Node("multi_service_client") {
        // Create multiple clients
        enable_client_ = create_client<std_srvs::srv::SetBool>("enable");
        reset_client_ = create_client<std_srvs::srv::Trigger>("reset");
        mode_client_ = create_client<std_srvs::srv::SetBool>("set_mode");
    }

    void initialize_robot() {
        // 1. Enable robot
        RCLCPP_INFO(this->get_logger(), "Enabling robot...");
        call_enable(true);

        // 2. Reset state
        RCLCPP_INFO(this->get_logger(), "Resetting state...");
        call_reset();

        // 3. Set mode
        RCLCPP_INFO(this->get_logger(), "Setting autonomous mode...");
        call_set_mode(true);

        RCLCPP_INFO(this->get_logger(), "Robot initialization complete");
    }

private:
    void call_enable(bool value) { /* ... */ }
    void call_reset() { /* ... */ }
    void call_set_mode(bool value) { /* ... */ }

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mode_client_;
};
```

## Service Call in Timer

Periodic service calls:

```cpp
class PeriodicClient : public rclcpp::Node {
public:
    PeriodicClient() : Node("periodic_client") {
        client_ = create_client<std_srvs::srv::Trigger>("get_status");

        // Call service every 5 seconds
        timer_ = create_wall_timer(
            5s,
            std::bind(&PeriodicClient::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        if (!client_->wait_for_service(100ms)) {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        // Async call with callback
        client_->async_send_request(request,
            [this](auto future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Status: %s", response->message.c_str());
            }
        );
    }

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## Service Client + Subscriber Pattern

React to topic messages by calling services:

```cpp
class ReactiveClient : public rclcpp::Node {
public:
    ReactiveClient() : Node("reactive_client") {
        // Subscribe to commands
        subscription_ = create_subscription<std_msgs::msg::Bool>(
            "enable_command",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->handle_command(msg->data);
            }
        );

        // Create service client
        client_ = create_client<std_srvs::srv::SetBool>("set_mode");
    }

private:
    void handle_command(bool enable) {
        RCLCPP_INFO(this->get_logger(), "Received command: %s",
                    enable ? "ENABLE" : "DISABLE");

        // Call service based on topic message
        if (!client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = enable;

        client_->async_send_request(request,
            [this](auto future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Service response: %s",
                            response->message.c_str());
            }
        );
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};
```

## Error Handling

### Handle Exceptions

```cpp
void safe_service_call() {
    try {
        auto request = std::make_shared<ServiceType::Request>();
        auto future = client_->async_send_request(request);

        auto status = rclcpp::spin_until_future_complete(node, future, 5s);

        if (status == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            // Process response
        } else {
            RCLCPP_ERROR(logger, "Service call failed");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Exception: %s", e.what());
    }
}
```

### Validate Response

```cpp
void validate_response() {
    auto future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();

        // Check success flag
        if (!response->success) {
            RCLCPP_WARN(logger, "Service rejected request: %s", response->message.c_str());
            return;
        }

        // Validate data
        if (response->value < min_value_ || response->value > max_value_) {
            RCLCPP_ERROR(logger, "Response value out of range");
            return;
        }

        // Use response
        process_valid_response(response);
    }
}
```

## Testing Service Clients

### Manual Testing

```bash
# Terminal 1: Run service server
ros2 run my_package server_node

# Terminal 2: Run client
ros2 run my_package client_node

# Observe output in both terminals
```

### With Mock Server

```bash
# Terminal 1: Mock server (using CLI)
# When client calls, manually respond:
ros2 service call /service_name service_type "{...}"

# Terminal 2: Run client
ros2 run my_package client_node
```

## Best Practices

### DO ✓

```cpp
// Always check service availability
if (!client_->wait_for_service(timeout)) {
    RCLCPP_ERROR(logger, "Service not available");
    return;
}

// Use timeouts
auto status = rclcpp::spin_until_future_complete(node, future, 5s);

// Handle all return codes
switch (status) {
    case rclcpp::FutureReturnCode::SUCCESS: /* ... */ break;
    case rclcpp::FutureReturnCode::TIMEOUT: /* ... */ break;
    case rclcpp::FutureReturnCode::INTERRUPTED: /* ... */ break;
}

// Use async with callbacks for responsive nodes
client_->async_send_request(request, callback);

// Validate responses
if (!response->success) {
    // Handle failure
}
```

### DON'T ✗

```cpp
// Don't block forever
auto future = client_->async_send_request(request);
auto response = future.get();  // Blocks forever if service fails!

// Don't call services in constructors
MyNode() {
    client_->async_send_request(request);  // Service may not exist yet!
}

// Don't create clients repeatedly
void function() {
    auto client = create_client<...>(...);  // Bad! Create once
}

// Don't ignore timeouts
if (spin_until_future_complete(node, future) == SUCCESS) {
    // What if it times out? Handle it!
}
```

## Complete Example: Robot Initializer

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

class RobotInitializer : public rclcpp::Node {
public:
    RobotInitializer() : Node("robot_initializer") {
        enable_client_ = create_client<std_srvs::srv::SetBool>("enable_motors");
        reset_client_ = create_client<std_srvs::srv::Trigger>("reset_odometry");
        mode_client_ = create_client<std_srvs::srv::SetBool>("set_autonomous");
    }

    bool initialize() {
        RCLCPP_INFO(this->get_logger(), "Starting robot initialization...");

        // Step 1: Enable motors
        if (!call_enable_motors(true)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable motors");
            return false;
        }

        // Step 2: Reset odometry
        if (!call_reset_odometry()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to reset odometry");
            return false;
        }

        // Step 3: Set autonomous mode
        if (!call_set_autonomous(true)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set autonomous mode");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Robot initialization complete!");
        return true;
    }

private:
    bool call_enable_motors(bool enable) {
        if (!enable_client_->wait_for_service(2s)) {
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = enable;

        auto future = enable_client_->async_send_request(request);
        auto status = rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future, 5s);

        if (status == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Motors: %s", response->message.c_str());
            return response->success;
        }
        return false;
    }

    bool call_reset_odometry() {
        if (!reset_client_->wait_for_service(2s)) {
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = reset_client_->async_send_request(request);
        auto status = rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future, 5s);

        if (status == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Odometry: %s", response->message.c_str());
            return response->success;
        }
        return false;
    }

    bool call_set_autonomous(bool enable) {
        if (!mode_client_->wait_for_service(2s)) {
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = enable;

        auto future = mode_client_->async_send_request(request);
        auto status = rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future, 5s);

        if (status == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Mode: %s", response->message.c_str());
            return response->success;
        }
        return false;
    }

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mode_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotInitializer>();

    if (node->initialize()) {
        RCLCPP_INFO(node->get_logger(), "SUCCESS - Robot ready!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "FAILED - Robot not ready");
    }

    rclcpp::shutdown();
    return 0;
}
```

## Summary

**Key Takeaways:**
- Service clients call services provided by servers
- Use `create_client<Type>("name")`
- Always wait for service availability
- Use timeouts to prevent blocking forever
- Async with callbacks for responsive nodes
- Handle all return codes (SUCCESS, TIMEOUT, INTERRUPTED)
- Validate responses before using

**Critical Patterns:**

**Synchronous:**
```cpp
auto future = client_->async_send_request(request);
rclcpp::spin_until_future_complete(node, future, timeout);
auto response = future.get();
```

**Asynchronous:**
```cpp
client_->async_send_request(request,
    [this](auto future) {
        auto response = future.get();
        // Process
    }
);
```

## Practice Exercise

Create a client node that:
1. Calls a calculator service to add two numbers
2. Waits for service with timeout
3. Handles timeout gracefully
4. Validates the response
5. Calls the service 5 times with different values

## What's Next?

- **Next Lesson**: [Parameters](09-parameters.md) - Node configuration
- **Related**: [Service Servers](07-service-servers.md) - Providing services
- **Advanced**: [Executors](12-executors.md) - Multi-threaded execution

---

**You can now call services in ROS2!** Next: configuring nodes with parameters.
