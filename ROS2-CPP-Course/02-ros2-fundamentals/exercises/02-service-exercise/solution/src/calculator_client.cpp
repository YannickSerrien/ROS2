/**
 * @file calculator_client.cpp
 * @brief Solution: Service client that calls the calculator
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
        client_ = this->create_client<Calculate>("/calculate");

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
        // Test all operations
        call_service(10.0, 5.0, "add");
        call_service(10.0, 5.0, "subtract");
        call_service(10.0, 5.0, "multiply");
        call_service(10.0, 5.0, "divide");

        // Test error cases
        call_service(10.0, 0.0, "divide");  // Division by zero
        call_service(10.0, 5.0, "invalid_op");  // Invalid operation

        RCLCPP_INFO(this->get_logger(), "All tests completed!");
    }

    void call_service(double a, double b, const std::string& operation)
    {
        auto request = std::make_shared<Calculate::Request>();
        request->a = a;
        request->b = b;
        request->operation = operation;

        RCLCPP_INFO(this->get_logger(),
                    "Sending request: %.1f %s %.1f",
                    a, operation.c_str(), b);

        auto result_future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();

            if (result->success) {
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

    rclcpp::Client<Calculate>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalculatorClient>();
    rclcpp::shutdown();
    return 0;
}
