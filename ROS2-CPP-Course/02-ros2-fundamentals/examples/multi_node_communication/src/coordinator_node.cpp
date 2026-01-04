/**
 * @file coordinator_node.cpp
 * @brief System coordinator (service client)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CoordinatorNode : public rclcpp::Node
{
public:
    CoordinatorNode() : Node("coordinator_node")
    {
        client_ = this->create_client<std_srvs::srv::Trigger>("get_stats");

        timer_ = this->create_wall_timer(10s, [this]() {
            if (client_->wait_for_service(1s)) {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto future = client_->async_send_request(request);

                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 1s) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Stats: %s", response->message.c_str());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Monitor service not available");
            }
        });

        RCLCPP_INFO(this->get_logger(), "Coordinator started, requesting stats every 10s");
    }

private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinatorNode>());
    rclcpp::shutdown();
    return 0;
}
