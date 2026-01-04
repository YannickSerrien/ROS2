/**
 * @file calculator_server.cpp
 * @brief Solution: Service server that performs calculations
 */

#include <rclcpp/rclcpp.hpp>
#include "service_exercise/srv/calculate.hpp"

using Calculate = service_exercise::srv::Calculate;

class CalculatorServer : public rclcpp::Node
{
public:
    CalculatorServer() : Node("calculator_server")
    {
        service_ = this->create_service<Calculate>(
            "/calculate",
            std::bind(&CalculatorServer::handle_calculate, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Calculator server ready");
    }

private:
    void handle_calculate(
        const std::shared_ptr<Calculate::Request> request,
        std::shared_ptr<Calculate::Response> response)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Received: %.1f %s %.1f",
                    request->a, request->operation.c_str(), request->b);

        if (request->operation == "add") {
            response->result = request->a + request->b;
            response->success = true;
            response->message = "Addition successful";
        }
        else if (request->operation == "subtract") {
            response->result = request->a - request->b;
            response->success = true;
            response->message = "Subtraction successful";
        }
        else if (request->operation == "multiply") {
            response->result = request->a * request->b;
            response->success = true;
            response->message = "Multiplication successful";
        }
        else if (request->operation == "divide") {
            if (request->b == 0.0) {
                response->result = 0.0;
                response->success = false;
                response->message = "Division by zero";
                RCLCPP_WARN(this->get_logger(), "Division by zero attempted");
            } else {
                response->result = request->a / request->b;
                response->success = true;
                response->message = "Division successful";
            }
        }
        else {
            response->result = 0.0;
            response->success = false;
            response->message = "Unknown operation: " + request->operation;
            RCLCPP_WARN(this->get_logger(),
                       "Unknown operation: %s", request->operation.c_str());
        }
    }

    rclcpp::Service<Calculate>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalculatorServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
