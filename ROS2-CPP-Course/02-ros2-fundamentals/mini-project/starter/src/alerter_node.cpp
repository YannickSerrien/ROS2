/**
 * @file alerter_node.cpp
 * @brief Display and log temperature alerts
 *
 * TODO: Complete the alerter node implementation
 */

#include <rclcpp/rclcpp.hpp>
#include "temperature_monitor/msg/alert.hpp"
#include <fstream>

class AlerterNode : public rclcpp::Node
{
public:
    AlerterNode() : Node("alerter")
    {
        // TODO: Declare parameters
        // - log_file (string, default: "")
        // - enable_sound (bool, default: false)

        // TODO: Get parameter values

        // TODO: If log_file is specified, open file for appending

        // TODO: Create subscriber for /alerts topic

        RCLCPP_INFO(this->get_logger(), "Alerter started");
    }

    ~AlerterNode()
    {
        // TODO: Close log file if open
    }

private:
    void alert_callback(const temperature_monitor::msg::Alert::SharedPtr msg)
    {
        // TODO: Format and display alert with color based on alert_type
        // HIGH -> Yellow/Warning
        // CRITICAL -> Red/Error
        // LOW -> Blue/Info

        // TODO: If logging enabled, write to log file
        // Format: [timestamp] [alert_type] room_name: temperature message

        // TODO: If sound enabled, simulate sound alert (just log)
    }

    std::string format_alert_message(const temperature_monitor::msg::Alert::SharedPtr msg)
    {
        // TODO: Create formatted string for alert
        // Include emoji/icon based on alert_type
        return "";
    }

    // TODO: Declare member variables
    rclcpp::Subscription<temperature_monitor::msg::Alert>::SharedPtr subscription_;
    std::string log_file_;
    bool enable_sound_;
    std::ofstream log_stream_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AlerterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
