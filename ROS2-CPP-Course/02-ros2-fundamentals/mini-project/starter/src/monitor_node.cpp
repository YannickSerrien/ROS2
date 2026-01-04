/**
 * @file monitor_node.cpp
 * @brief Monitor temperature readings and generate alerts
 *
 * TODO: Complete the monitor node implementation
 */

#include <rclcpp/rclcpp.hpp>
#include "temperature_monitor/msg/temperature_reading.hpp"
#include "temperature_monitor/msg/alert.hpp"
#include "temperature_monitor/srv/get_stats.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <map>
#include <limits>

// Structure to track statistics for each room
struct RoomStats {
    int count = 0;
    double sum = 0.0;
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    double current = 0.0;
};

class MonitorNode : public rclcpp::Node
{
public:
    MonitorNode() : Node("temp_monitor")
    {
        // TODO: Declare parameters for thresholds

        // TODO: Get parameter values

        // TODO: Create subscriber for /temperatures

        // TODO: Create publisher for /alerts

        // TODO: Create service for /get_stats

        // TODO: Create service for /reset_stats

        RCLCPP_INFO(this->get_logger(), "Monitor started (thresholds: %.1f / %.1f / %.1f)",
                    low_threshold_, high_threshold_, critical_threshold_);
    }

private:
    void temperature_callback(const temperature_monitor::msg::TemperatureReading::SharedPtr msg)
    {
        // TODO: Update statistics for this room
        // Hint: Use room_stats_ map with room_name as key

        // TODO: Check temperature against thresholds and generate alerts if needed
        // Generate Alert message if temperature exceeds thresholds

        // TODO: Publish alert if generated
    }

    void get_stats_callback(
        const std::shared_ptr<temperature_monitor::srv::GetStats::Request> request,
        std::shared_ptr<temperature_monitor::srv::GetStats::Response> response)
    {
        // TODO: If request->room_name is empty, return stats for all rooms
        // TODO: Otherwise, return stats for specific room
        // Fill response vectors: rooms, avg_temperatures, min_temperatures, max_temperatures, sample_counts

        RCLCPP_INFO(this->get_logger(), "Stats requested for: %s",
                    request->room_name.empty() ? "all rooms" : request->room_name.c_str());
    }

    void reset_stats_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        // TODO: Clear all statistics
        room_stats_.clear();
        response->success = true;
        response->message = "Statistics reset";
        RCLCPP_INFO(this->get_logger(), "Statistics reset");
    }

    // TODO: Declare member variables
    rclcpp::Subscription<temperature_monitor::msg::TemperatureReading>::SharedPtr subscription_;
    rclcpp::Publisher<temperature_monitor::msg::Alert>::SharedPtr alert_publisher_;
    rclcpp::Service<temperature_monitor::srv::GetStats>::SharedPtr stats_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    double high_threshold_;
    double low_threshold_;
    double critical_threshold_;

    std::map<std::string, RoomStats> room_stats_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
