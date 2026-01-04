/**
 * @file talker_component.cpp
 * @brief Simple talker component for composition demonstration
 *
 * Demonstrates:
 * - Converting regular node to component
 * - NodeOptions parameter
 * - Component registration macro
 * - Zero-copy publishing with unique_ptr
 *
 * Used in: Module 3 - Composition
 */

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace composition_example
{

class TalkerComponent : public rclcpp::Node
{
public:
    /**
     * IMPORTANT: Constructor must take NodeOptions
     * This is the key difference from regular nodes
     */
    explicit TalkerComponent(const rclcpp::NodeOptions & options)
    : Node("talker", options),
      count_(0)
    {
        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

        // Create timer
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&TalkerComponent::publish_message, this)
        );

        RCLCPP_INFO(this->get_logger(), "Talker Component initialized");
    }

private:
    void publish_message()
    {
        // Use unique_ptr for zero-copy publishing in composed systems
        auto message = std::make_unique<std_msgs::msg::String>();
        message->data = "Hello World: " + std::to_string(count_++);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message->data.c_str());

        // std::move transfers ownership - enables zero-copy
        publisher_->publish(std::move(message));
    }

    size_t count_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace composition_example

// Register component with ROS2 composition system
RCLCPP_COMPONENTS_REGISTER_NODE(composition_example::TalkerComponent)
