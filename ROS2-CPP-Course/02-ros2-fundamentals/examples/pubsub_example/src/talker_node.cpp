/**
 * @file talker_node.cpp
 * @brief Simple publisher node that sends periodic messages
 *
 * Demonstrates:
 * - Creating a publisher
 * - Timer-based periodic publishing
 * - Basic logging
 * - String message type
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node
{
public:
    TalkerNode() : Node("talker"), count_(0)
    {
        // Create publisher on "chatter" topic with queue size 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

        // Create timer that fires every 500ms
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&TalkerNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Talker node started, publishing on 'chatter' topic");
    }

private:
    void timer_callback()
    {
        // Create message
        auto message = std_msgs::msg::String();
        message.data = "Hello, ROS2! Message #" + std::to_string(count_++);

        // Log and publish
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TalkerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
