/**
 * @file listener_component.cpp
 * @brief Simple listener component for composition demonstration
 *
 * Demonstrates:
 * - Receiving messages via intra-process communication
 * - Zero-copy subscription (receives pointer, not copy)
 * - Component node pattern
 *
 * Used in: Module 3 - Composition
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition_example
{

class ListenerComponent : public rclcpp::Node
{
public:
    /**
     * Constructor with NodeOptions (required for components)
     */
    explicit ListenerComponent(const rclcpp::NodeOptions & options)
    : Node("listener", options)
    {
        // Create subscription
        // When composed with talker, this receives messages via zero-copy!
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter",
            10,
            std::bind(&ListenerComponent::message_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Listener Component initialized");
    }

private:
    void message_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // In composed systems, msg is the SAME pointer published by talker
        // No serialization, no copying - just a shared pointer!
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}  // namespace composition_example

// Register component
RCLCPP_COMPONENTS_REGISTER_NODE(composition_example::ListenerComponent)
