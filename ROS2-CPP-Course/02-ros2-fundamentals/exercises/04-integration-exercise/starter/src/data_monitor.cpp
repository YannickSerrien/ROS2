/**
 * @file data_monitor.cpp
 * @brief Exercise: Monitor processed data and request statistics
 *
 * TODO: Complete this node to monitor data and call stats service
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>

using namespace std::chrono_literals;

class DataMonitor : public rclcpp::Node
{
public:
    DataMonitor() : Node("data_monitor")
    {
        // TODO: Declare parameters
        // - alert_threshold (double, default: 50.0)
        // - stats_interval (double, default: 5.0)
        /* YOUR CODE HERE */

        // TODO: Get parameter values
        /* YOUR CODE HERE */

        // TODO: Create subscriber for /processed_data
        /* YOUR CODE HERE */

        // TODO: Create client for /process_stats service
        /* YOUR CODE HERE */

        // TODO: Create timer to request stats periodically
        // Use stats_interval parameter to determine period
        /* YOUR CODE HERE */

        RCLCPP_INFO(this->get_logger(),
                    "Data Monitor started (threshold: %.2f, stats interval: %.2fs)",
                    alert_threshold_, stats_interval_);
    }

private:
    void data_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double value = msg->data;

        // TODO: Check if value exceeds threshold
        // If yes, log as WARNING with [ALERT: Above threshold!]
        // If no, log as INFO
        /* YOUR CODE HERE */
    }

    void request_stats()
    {
        // TODO: Check if service is available
        /* YOUR CODE HERE */

        // TODO: Create service request
        /* YOUR CODE HERE */

        // TODO: Call service asynchronously
        /* YOUR CODE HERE */

        // TODO: Handle response
        // Hint: Use a callback with result_future.add_done_callback(...)
        //       or use spin_until_future_complete
        /* YOUR CODE HERE */
    }

    // TODO: Declare member variables
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    double alert_threshold_;
    double stats_interval_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
