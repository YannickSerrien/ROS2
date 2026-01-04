/**
 * @file calculator_server.cpp
 * @brief Exercise: Create a service server that performs calculations
 *
 * TODO: Complete this node to provide a calculator service
 */

#include <rclcpp/rclcpp.hpp>
#include "service_exercise/srv/calculate.hpp"

using Calculate = service_exercise::srv::Calculate;

class CalculatorServer : public rclcpp::Node
{
public:
    CalculatorServer() : Node("calculator_server")
    {
        // TODO: Create a service named "/calculate" that uses the Calculate service type
        // Hint: this->create_service<ServiceType>(service_name, callback)
        // The callback should be: std::bind(&CalculatorServer::handle_calculate, this,
        //                                    std::placeholders::_1, std::placeholders::_2)

        service_ = /* YOUR CODE HERE */;

        RCLCPP_INFO(this->get_logger(), "Calculator server ready");
    }

private:
    void handle_calculate(
        const std::shared_ptr<Calculate::Request> request,
        std::shared_ptr<Calculate::Response> response)
    {
        // Log the received request
        RCLCPP_INFO(this->get_logger(),
                    "Received: %.1f %s %.1f",
                    request->a, request->operation.c_str(), request->b);

        // TODO: Implement the calculator logic
        // Based on request->operation, perform the appropriate calculation
        // Set response->result, response->success, and response->message

        if (request->operation == "add") {
            // TODO: Implement addition
            /* YOUR CODE HERE */
        }
        else if (request->operation == "subtract") {
            // TODO: Implement subtraction
            /* YOUR CODE HERE */
        }
        else if (request->operation == "multiply") {
            // TODO: Implement multiplication
            /* YOUR CODE HERE */
        }
        else if (request->operation == "divide") {
            // TODO: Implement division with zero-check
            // Hint: Check if request->b == 0.0 first!
            /* YOUR CODE HERE */
        }
        else {
            // TODO: Handle unknown operation
            // Set success=false and appropriate error message
            /* YOUR CODE HERE */
        }
    }

    // TODO: Declare service member variable
    rclcpp::Service<Calculate>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // TODO: Create the server node and spin
    /* YOUR CODE HERE */

    rclcpp::shutdown();
    return 0;
}
