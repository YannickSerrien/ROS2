/**
 * @file heartbeat_node.cpp
 * @brief Heartbeat node demonstrating all logging levels
 *
 * Demonstrates:
 * - All ROS2 logging levels (DEBUG, INFO, WARN, ERROR, FATAL)
 * - Conditional logging (ONCE, SKIPFIRST, THROTTLE)
 * - Expression-based logging
 * - Formatted logging
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class HeartbeatNode : public rclcpp::Node
{
public:
    HeartbeatNode() : Node("heartbeat"), count_(0)
    {
        // Create timer for heartbeat (500ms)
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&HeartbeatNode::heartbeat_callback, this));

        RCLCPP_INFO(this->get_logger(), "Heartbeat node started");
        RCLCPP_INFO(this->get_logger(), "Run with different log levels:");
        RCLCPP_INFO(this->get_logger(), "  ros2 run timer_logging_example heartbeat --ros-args --log-level debug");
        RCLCPP_INFO(this->get_logger(), "  ros2 run timer_logging_example heartbeat --ros-args --log-level warn");
    }

private:
    void heartbeat_callback()
    {
        count_++;

        // Regular INFO log (always shows at INFO level and above)
        RCLCPP_INFO(this->get_logger(), "Heartbeat #%zu", count_);

        // DEBUG log (only shows with --log-level debug)
        RCLCPP_DEBUG(this->get_logger(), "Debug: Detailed heartbeat info for #%zu", count_);

        // Log ONCE - only the first time
        RCLCPP_INFO_ONCE(this->get_logger(), "This message appears only once!");

        // Log SKIPFIRST - skip the first N calls
        if (count_ >= 3) {
            RCLCPP_INFO_SKIPFIRST(this->get_logger(), "This message skipped first 2 calls");
        }

        // THROTTLE - log at most once per time period (5 seconds)
        RCLCPP_INFO_THROTTLE(this->get_logger(),
                            *this->get_clock(),
                            5000,  // milliseconds
                            "Throttled message (max once per 5s). Count: %zu", count_);

        // Conditional logging based on expression
        if (count_ % 5 == 0) {
            RCLCPP_INFO_EXPRESSION(this->get_logger(),
                                  count_ % 5 == 0,
                                  "Count is divisible by 5: %zu", count_);
        }

        // Different severity levels based on count
        if (count_ == 10) {
            RCLCPP_WARN(this->get_logger(), "Warning: Count reached 10!");
        }

        if (count_ == 20) {
            RCLCPP_ERROR(this->get_logger(), "Error condition: Count reached 20!");
        }

        if (count_ == 25) {
            RCLCPP_FATAL(this->get_logger(), "Fatal error: Count reached 25! (node continues anyway)");
        }

        // Stream-style logging (alternative syntax)
        if (count_ % 10 == 0) {
            RCLCPP_INFO_STREAM(this->get_logger(),
                              "Stream-style log at count " << count_ << " (every 10 beats)");
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting heartbeat node");
    RCLCPP_INFO(rclcpp::get_logger("main"), "This node demonstrates all logging levels and patterns");

    auto node = std::make_shared<HeartbeatNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
