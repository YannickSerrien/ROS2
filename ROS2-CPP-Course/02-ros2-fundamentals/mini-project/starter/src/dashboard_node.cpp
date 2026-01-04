/**
 * @file dashboard_node.cpp
 * @brief Display temperature statistics dashboard
 *
 * TODO: Complete the dashboard node implementation
 */

#include <rclcpp/rclcpp.hpp>
#include "temperature_monitor/srv/get_stats.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

class DashboardNode : public rclcpp::Node
{
public:
    DashboardNode() : Node("dashboard")
    {
        // TODO: Declare parameter for update_interval

        // TODO: Get parameter value

        // TODO: Create client for /get_stats service

        // TODO: Create timer to request stats periodically

        RCLCPP_INFO(this->get_logger(), "Dashboard started (update interval: %.1fs)", update_interval_);
    }

private:
    void request_and_display_stats()
    {
        // TODO: Check if service is available

        // TODO: Create request (empty room_name for all rooms)

        // TODO: Call service and wait for response

        // TODO: If successful, format and display statistics table
        // Use display_stats_table() helper function
    }

    void display_stats_table(const temperature_monitor::srv::GetStats::Response::SharedPtr response)
    {
        // TODO: Create formatted table output
        // Use box-drawing characters or simple ASCII art
        // Include: Room name, Sample count, Avg, Min, Max temperatures

        /*
        Example format:
        ╔════════════════════════════════════════════════════╗
        ║         TEMPERATURE MONITORING DASHBOARD           ║
        ╠════════════════════════════════════════════════════╣
        ║ Room        │ Samples │ Avg    │ Min  │ Max       ║
        ╠═════════════╪═════════╪════════╪══════╪═══════════╣
        ║ Living Room │   45    │ 22.1°C │ 19°C │ 25°C     ║
        ╚════════════════════════════════════════════════════╝
        */
    }

    // TODO: Declare member variables
    rclcpp::Client<temperature_monitor::srv::GetStats>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    double update_interval_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DashboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
