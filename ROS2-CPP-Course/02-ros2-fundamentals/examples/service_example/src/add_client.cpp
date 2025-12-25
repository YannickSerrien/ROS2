/**
 * @file add_client.cpp
 * @brief Service client that calls the add_two_ints service
 *
 * Demonstrates:
 * - Creating a service client
 * - Waiting for service availability
 * - Making synchronous service calls
 * - Handling service responses
 * - Command-line argument parsing
 */

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class AddClientNode : public rclcpp::Node
{
public:
    AddClientNode() : Node("add_client")
    {
        // Create service client
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    bool wait_for_service(std::chrono::seconds timeout = 5s)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");

        if (!client_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(),
                        "Service not available after waiting for %ld seconds",
                        timeout.count());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Service is available!");
        return true;
    }

    int64_t send_request(int64_t a, int64_t b)
    {
        // Create request
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        RCLCPP_INFO(this->get_logger(), "Sending request: %ld + %ld", a, b);

        // Send request synchronously
        auto future = client_->async_send_request(request);

        // Wait for response (blocking call)
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Received response: %ld", response->sum);
            return response->sum;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive response");
            return 0;
        }
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Parse command-line arguments
    if (argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("add_client"),
                    "Usage: ros2 run service_example add_client <a> <b>");
        return 1;
    }

    int64_t a = std::atoll(argv[1]);
    int64_t b = std::atoll(argv[2]);

    auto node = std::make_shared<AddClientNode>();

    // Wait for service to be available
    if (!node->wait_for_service()) {
        return 1;
    }

    // Send request
    int64_t result = node->send_request(a, b);

    RCLCPP_INFO(node->get_logger(), "Result: %ld + %ld = %ld", a, b, result);

    rclcpp::shutdown();
    return 0;
}
