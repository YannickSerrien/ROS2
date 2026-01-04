/**
 * @file periodic_publisher.cpp
 * @brief Timer-based periodic publisher with different timer intervals
 *
 * Demonstrates:
 * - Creating wall timers
 * - Different timer periods (milliseconds, seconds)
 * - Timer-based publishing
 * - Using chrono literals
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class PeriodicPublisherNode : public rclcpp::Node
{
public:
    PeriodicPublisherNode() : Node("periodic_publisher"), fast_count_(0), slow_count_(0)
    {
        // Create publishers
        fast_publisher_ = this->create_publisher<std_msgs::msg::String>("fast_updates", 10);
        slow_publisher_ = this->create_publisher<std_msgs::msg::String>("slow_updates", 10);

        // Fast timer: 100ms (10 Hz)
        fast_timer_ = this->create_wall_timer(
            100ms,
            [this]() {
                auto msg = std_msgs::msg::String();
                msg.data = "Fast update #" + std::to_string(fast_count_++);
                fast_publisher_->publish(msg);
                RCLCPP_DEBUG(this->get_logger(), "Fast: %s", msg.data.c_str());
            });

        // Slow timer: 1 second (1 Hz)
        slow_timer_ = this->create_wall_timer(
            1s,
            [this]() {
                auto msg = std_msgs::msg::String();
                msg.data = "Slow update #" + std::to_string(slow_count_++);
                slow_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Slow: %s", msg.data.c_str());
            });

        RCLCPP_INFO(this->get_logger(), "Periodic Publisher started");
        RCLCPP_INFO(this->get_logger(), "  Fast timer: 100ms (10 Hz) on /fast_updates");
        RCLCPP_INFO(this->get_logger(), "  Slow timer: 1s (1 Hz) on /slow_updates");
        RCLCPP_INFO(this->get_logger(), "Run with --log-level debug to see fast updates");
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fast_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slow_publisher_;
    rclcpp::TimerBase::SharedPtr fast_timer_;
    rclcpp::TimerBase::SharedPtr slow_timer_;
    size_t fast_count_;
    size_t slow_count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting periodic publisher");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Echo topics:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  ros2 topic echo /fast_updates");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  ros2 topic echo /slow_updates");

    auto node = std::make_shared<PeriodicPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
