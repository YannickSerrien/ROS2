/**
 * @file monitor_node.cpp
 * @brief Monitor with subscriber and service
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

class MonitorNode : public rclcpp::Node
{
public:
    MonitorNode() : Node("monitor_node"), count_(0), sum_(0.0)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "processed_data", 10, [this](std_msgs::msg::Float64::SharedPtr msg) {
                count_++;
                sum_ += msg->data;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Monitoring: %.2f (count: %zu, avg: %.2f)", msg->data, count_, sum_/count_);
            });

        service_ = this->create_service<std_srvs::srv::Trigger>(
            "get_stats", [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                response->success = true;
                response->message = "Count: " + std::to_string(count_) +
                                   ", Avg: " + std::to_string(sum_/count_);
                RCLCPP_INFO(this->get_logger(), "Stats requested: %s", response->message.c_str());
            });

        RCLCPP_INFO(this->get_logger(), "Monitor node started, call /get_stats for statistics");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    size_t count_;
    double sum_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitorNode>());
    rclcpp::shutdown();
    return 0;
}
