/**
 * @file calculator_client.cpp
 * @brief Exercise: Create a service client that calls the calculator
 *
 * TODO: Complete this node to call the calculator service with test cases
 */

#include <rclcpp/rclcpp.hpp>
#include "service_exercise/srv/calculate.hpp"
#include <chrono>

using namespace std::chrono_literals;
using Calculate = service_exercise::srv::Calculate;

class CalculatorClient : public rclcpp::Node
{
public:
    CalculatorClient() : Node("calculator_client")
    {
        // TODO: Create a client for the "/calculate" service
        // Hint: this->create_client<ServiceType>(service_name)

        client_ = /* YOUR CODE HERE */;

        // Wait for service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for calculator service...");
        }

        // Run test cases
        run_tests();
    }

private:
    void run_tests()
    {
        // TODO: Create test cases for all operations
        // Test addition
        call_service(10.0, 5.0, "add");

        // TODO: Test subtraction
        /* YOUR CODE HERE */

        // TODO: Test multiplication
        /* YOUR CODE HERE */

        // TODO: Test division
        /* YOUR CODE HERE */

        // TODO: Test division by zero (should fail gracefully)
        /* YOUR CODE HERE */

        // TODO: Test invalid operation (should fail gracefully)
        /* YOUR CODE HERE */

        RCLCPP_INFO(this->get_logger(), "All tests completed!");
    }

    void call_service(double a, double b, const std::string& operation)
    {
        // TODO: Create a request
        auto request = std::make_shared<Calculate::Request>();
        /* YOUR CODE HERE - Set request->a, request->b, request->operation */

        RCLCPP_INFO(this->get_logger(),
                    "Sending request: %.1f %s %.1f",
                    a, operation.c_str(), b);

        // TODO: Send the request asynchronously and get the future
        auto result_future = /* YOUR CODE HERE */;

        // TODO: Wait for the result
        // Hint: Use rclcpp::spin_until_future_complete(shared_from_this(), result_future)
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();

            // TODO: Check if the service call was successful
            if (/* YOUR CODE HERE - check result->success */) {
                RCLCPP_INFO(this->get_logger(),
                           "Result: %.2f (Success)", result->result);
            } else {
                RCLCPP_WARN(this->get_logger(),
                           "Service failed: %s", result->message.c_str());
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

    // TODO: Declare client member variable
    rclcpp::Client<Calculate>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalculatorClient>();
    rclcpp::shutdown();
    return 0;
}
