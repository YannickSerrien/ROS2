/**
 * @file parameter_talker.cpp
 * @brief Configurable publisher using parameters
 *
 * Demonstrates:
 * - Declaring parameters with defaults
 * - Getting parameter values
 * - Configuring node behavior via parameters
 * - QoS settings
 * - Parameter-driven publishing rate
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class ParameterTalkerNode : public rclcpp::Node
{
public:
    ParameterTalkerNode() : Node("parameter_talker"), count_(0)
    {
        // Declare parameters with default values
        this->declare_parameter("topic", "chatter");
        this->declare_parameter("rate", 1.0);  // Hz
        this->declare_parameter("message_prefix", "Hello from parameter_talker");
        this->declare_parameter("queue_size", 10);

        // Get parameter values
        std::string topic = this->get_parameter("topic").as_string();
        double rate = this->get_parameter("rate").as_double();
        message_prefix_ = this->get_parameter("message_prefix").as_string();
        int queue_size = this->get_parameter("queue_size").as_int();

        // Create publisher with configured topic and queue size
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic, queue_size);

        // Create timer with configured rate
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
        timer_ = this->create_wall_timer(
            period,
            std::bind(&ParameterTalkerNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(),
                    "Parameter Talker started with:");
        RCLCPP_INFO(this->get_logger(), "  Topic: %s", topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Rate: %.2f Hz", rate);
        RCLCPP_INFO(this->get_logger(), "  Prefix: %s", message_prefix_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Queue size: %d", queue_size);
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = message_prefix_ + " #" + std::to_string(count_++);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string message_prefix_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("main"),
                "Starting parameter_talker node. Configure with:");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "  ros2 run pubsub_example parameter_talker --ros-args -p topic:=custom_topic -p rate:=2.0");

    auto node = std::make_shared<ParameterTalkerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
