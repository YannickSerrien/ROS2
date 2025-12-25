/**
 * @file listener_node.cpp
 * @brief Simple subscriber node that receives and processes messages
 *
 * Demonstrates:
 * - Creating a subscriber
 * - Callback function for message handling
 * - Logging received messages
 * - Lambda callback alternative
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ListenerNode : public rclcpp::Node
{
public:
    ListenerNode() : Node("listener")
    {
        // Create subscriber on "chatter" topic with queue size 10
        // Using std::bind to bind member function as callback
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter",
            10,
            std::bind(&ListenerNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Listener node started, subscribed to 'chatter' topic");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

// Alternative implementation using lambda callback
class ListenerNodeLambda : public rclcpp::Node
{
public:
    ListenerNodeLambda() : Node("listener_lambda")
    {
        // Using lambda function as callback (modern C++ style)
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Lambda heard: '%s'", msg->data.c_str());
            });

        RCLCPP_INFO(this->get_logger(), "Listener (lambda) node started");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Choose which implementation to use
    // auto node = std::make_shared<ListenerNode>();         // std::bind version
    auto node = std::make_shared<ListenerNodeLambda>();   // lambda version

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
