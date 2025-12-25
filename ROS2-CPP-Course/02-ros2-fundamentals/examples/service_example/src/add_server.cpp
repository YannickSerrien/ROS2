/**
 * @file add_server.cpp
 * @brief Simple service server that adds two integers
 *
 * Demonstrates:
 * - Creating a service server
 * - Handling service requests
 * - Sending service responses
 * - Basic error handling
 */

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <memory>

class AddServerNode : public rclcpp::Node
{
public:
    AddServerNode() : Node("add_server")
    {
        // Create service server
        // Signature: create_service<ServiceType>(name, callback)
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddServerNode::handle_add, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Add Two Ints service server is ready");
        RCLCPP_INFO(this->get_logger(), "Call with: ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts \"{a: 5, b: 3}\"");
    }

private:
    void handle_add(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        // Log incoming request
        RCLCPP_INFO(this->get_logger(),
                    "Incoming request: a=%ld, b=%ld",
                    request->a, request->b);

        // Perform calculation
        response->sum = request->a + request->b;

        // Log response
        RCLCPP_INFO(this->get_logger(),
                    "Sending response: %ld + %ld = %ld",
                    request->a, request->b, response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
