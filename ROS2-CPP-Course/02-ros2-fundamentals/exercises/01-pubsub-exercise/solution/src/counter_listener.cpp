/**
 * @file counter_listener.cpp
 * @brief Solution: Subscriber that receives and sums integers
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class CounterListener : public rclcpp::Node
{
public:
    CounterListener() : Node("counter_listener"), running_sum_(0)
    {
        // Create subscription to /counter topic
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/counter",
            10,
            std::bind(&CounterListener::counter_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Counter Listener started");
    }

private:
    void counter_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // Extract received value
        int received_value = msg->data;

        // Update running sum
        running_sum_ += received_value;

        // Log received value and running sum
        RCLCPP_INFO(this->get_logger(),
                    "Received: %d, Running sum: %d",
                    received_value,
                    running_sum_);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    int running_sum_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CounterListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
