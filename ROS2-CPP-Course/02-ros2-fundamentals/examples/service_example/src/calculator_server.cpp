/**
 * @file calculator_server.cpp
 * @brief Multi-service calculator demonstrating multiple services in one node
 *
 * Demonstrates:
 * - Multiple service servers in one node
 * - Different service types
 * - Error handling and validation
 * - Using SetBool for calculator state
 */

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <memory>

class CalculatorServerNode : public rclcpp::Node
{
public:
    CalculatorServerNode() : Node("calculator_server"), enabled_(true)
    {
        // Create multiple service servers
        add_service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "calculator/add",
            std::bind(&CalculatorServerNode::handle_add, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Using lambda for enable/disable service
        enable_service_ = this->create_service<std_srvs::srv::SetBool>(
            "calculator/enable",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                this->handle_enable(request, response);
            });

        RCLCPP_INFO(this->get_logger(), "Calculator server started with services:");
        RCLCPP_INFO(this->get_logger(), "  - /calculator/add");
        RCLCPP_INFO(this->get_logger(), "  - /calculator/enable");
    }

private:
    void handle_add(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        if (!enabled_) {
            RCLCPP_WARN(this->get_logger(),
                       "Calculator is disabled. Enable it first!");
            response->sum = 0;
            return;
        }

        response->sum = request->a + request->b;

        RCLCPP_INFO(this->get_logger(),
                    "Addition: %ld + %ld = %ld",
                    request->a, request->b, response->sum);
    }

    void handle_enable(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        enabled_ = request->data;

        response->success = true;
        response->message = enabled_ ? "Calculator enabled" : "Calculator disabled";

        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr add_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    bool enabled_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting calculator server");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Try:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  ros2 service call /calculator/add example_interfaces/srv/AddTwoInts \"{a: 10, b: 5}\"");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  ros2 service call /calculator/enable std_srvs/srv/SetBool \"{data: false}\"");

    auto node = std::make_shared<CalculatorServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
