/**
 * @file counter_listener.cpp
 * @brief Exercise: Create a subscriber that receives and sums integers
 *
 * TODO: Complete this node to subscribe to /counter topic and calculate running sum
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class CounterListener : public rclcpp::Node
{
public:
    CounterListener() : Node("counter_listener"), running_sum_(0)
    {
        // TODO: Create a subscriber for std_msgs::msg::Int32 on topic "/counter"
        // Hint: this->create_subscription<MessageType>(topic, queue_size, callback)
        // The callback should be: std::bind(&CounterListener::counter_callback, this, std::placeholders::_1)

        subscription_ = /* YOUR CODE HERE */;

        RCLCPP_INFO(this->get_logger(), "Counter Listener started");
    }

private:
    void counter_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // TODO: Extract the received value from the message
        // Hint: int received_value = msg->data;

        /* YOUR CODE HERE */

        // TODO: Add the received value to the running sum
        /* YOUR CODE HERE */;

        // TODO: Log the received value and running sum
        // Hint: RCLCPP_INFO(this->get_logger(), "Received: %d, Running sum: %d", received_value, running_sum_);

        /* YOUR CODE HERE */
    }

    // TODO: Declare member variables
    // Hint: You need a subscription and a variable to track the running sum
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    int running_sum_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // TODO: Create the node and spin it
    /* YOUR CODE HERE */

    rclcpp::shutdown();
    return 0;
}
