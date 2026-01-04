# Exercise 2: Services - Calculator

## Objective
Create a service-based calculator that performs basic arithmetic operations using ROS2 services.

## Learning Goals
- Create and call ROS2 services
- Define custom service interfaces
- Handle request-response patterns
- Implement error handling in services

## Task Description

Create two nodes:

### 1. CalculatorServer Node
- Provides a service `/calculate` that performs arithmetic operations
- Supports: addition, subtraction, multiplication, division
- Returns result and success status
- Handles division by zero

### 2. CalculatorClient Node
- Calls the `/calculate` service with test cases
- Displays results
- Handles service failures gracefully

## Service Interface

You'll create a custom service definition:

**File**: `srv/Calculate.srv`
```
# Request
float64 a
float64 b
string operation  # "add", "subtract", "multiply", "divide"
---
# Response
float64 result
bool success
string message
```

## Requirements

### CalculatorServer
- Service name: `/calculate`
- Supported operations: "add", "subtract", "multiply", "divide"
- Return `success=false` for:
  - Division by zero
  - Unknown operations
- Log each received request

### CalculatorClient
- Test all four operations
- Test division by zero
- Test invalid operation
- Display results in readable format

## Files to Complete

**Starter code**: `starter/`
- `srv/Calculate.srv` - Service definition (provided)
- `src/calculator_server.cpp` - Complete the TODOs
- `src/calculator_client.cpp` - Complete the TODOs
- `CMakeLists.txt` - Already configured
- `package.xml` - Already configured

**Solution**: `solution/` (for reference after attempting)

## Build Instructions

```bash
cd ROS2-CPP-Course/02-ros2-fundamentals/exercises/02-service-exercise/starter
colcon build --packages-select service_exercise
source install/setup.bash
```

## Running the Exercise

Terminal 1 (Server):
```bash
ros2 run service_exercise calculator_server
```

Terminal 2 (Client):
```bash
ros2 run service_exercise calculator_client
```

## Expected Output

**calculator_server terminal:**
```
[INFO] [calculator_server]: Calculator server ready
[INFO] [calculator_server]: Received: 10.0 + 5.0
[INFO] [calculator_server]: Received: 10.0 - 5.0
[INFO] [calculator_server]: Received: 10.0 * 5.0
[INFO] [calculator_server]: Received: 10.0 / 5.0
[WARN] [calculator_server]: Division by zero attempted
[WARN] [calculator_server]: Unknown operation: invalid_op
```

**calculator_client terminal:**
```
[INFO] [calculator_client]: Sending request: 10.0 + 5.0
[INFO] [calculator_client]: Result: 15.0 (Success)
[INFO] [calculator_client]: Sending request: 10.0 - 5.0
[INFO] [calculator_client]: Result: 5.0 (Success)
[INFO] [calculator_client]: Sending request: 10.0 * 5.0
[INFO] [calculator_client]: Result: 50.0 (Success)
[INFO] [calculator_client]: Sending request: 10.0 / 5.0
[INFO] [calculator_client]: Result: 2.0 (Success)
[INFO] [calculator_client]: Sending request: 10.0 / 0.0
[WARN] [calculator_client]: Service failed: Division by zero
[INFO] [calculator_client]: Sending request: 10.0 invalid_op 5.0
[WARN] [calculator_client]: Service failed: Unknown operation
```

## Testing Your Solution

1. All four basic operations should work correctly
2. Division by zero should be handled (no crash)
3. Invalid operations should return error message
4. Server should respond to all requests
5. Client should handle failures gracefully

## Hints

1. Service callback signature:
   ```cpp
   void handle_calculate(
       const std::shared_ptr<service_exercise::srv::Calculate::Request> request,
       std::shared_ptr<service_exercise::srv::Calculate::Response> response)
   ```

2. Creating a service:
   ```cpp
   service_ = this->create_service<service_exercise::srv::Calculate>(
       "service_name",
       std::bind(&ClassName::callback, this, _1, _2));
   ```

3. Calling a service asynchronously:
   ```cpp
   auto result_future = client_->async_send_request(request);
   if (rclcpp::spin_until_future_complete(node, result_future) ==
       rclcpp::FutureReturnCode::SUCCESS) {
       auto result = result_future.get();
   }
   ```

4. Remember to check `response->success` before using the result!

## Verification Questions

After completing this exercise, you should be able to answer:
1. What is the difference between a topic and a service?
2. When would you use a service instead of a topic?
3. What happens if a service is called but no server is running?
4. How does the client know if the service succeeded or failed?
5. Why is it important to validate inputs in the service server?

## Common Mistakes

- Forgetting to build after creating/modifying `.srv` file
- Not checking `response->success` in the client
- Not handling division by zero
- Incorrect string comparison for operation type
- Not waiting for service to be available before calling
- Forgetting `std::placeholders::_1, _2` in service bind

## Related Lessons

- [Lesson 4: Services](../../lessons/04-services.md)
- [Lesson 5: Custom Messages and Services](../../lessons/05-custom-messages-services.md)

## Extension Challenges

Once you've completed the basic exercise:
1. Add more operations (power, modulo, square root)
2. Add input validation (check for NaN, infinity)
3. Create a service that accepts a list of operations
4. Add a service to get calculation history

## Next Steps

Once you've completed this exercise, move on to:
- **Exercise 3**: Parameters and configuration
