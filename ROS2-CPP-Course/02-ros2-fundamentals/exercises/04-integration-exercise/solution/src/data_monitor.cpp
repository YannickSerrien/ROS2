/**
 * @file data_monitor.cpp
 * @brief Solution: Monitor processed data and request statistics
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
        // Declare parameters
        this->declare_parameter("alert_threshold", 50.0);
        this->declare_parameter("stats_interval", 5.0);

        // Get parameter values
        alert_threshold_ = this->get_parameter("alert_threshold").as_double();
        stats_interval_ = this->get_parameter("stats_interval").as_double();

        // Create subscriber
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/processed_data",
            10,
            std::bind(&DataMonitor::data_callback, this, std::placeholders::_1));

        // Create service client
        client_ = this->create_client<std_srvs::srv::Trigger>("/process_stats");

        // Create timer for stats requests
        auto period = std::chrono::duration<double>(stats_interval_);
        timer_ = this->create_wall_timer(
            period,
            std::bind(&DataMonitor::request_stats, this));

        RCLCPP_INFO(this->get_logger(),
                    "Data Monitor started (threshold: %.2f, stats interval: %.2fs)",
                    alert_threshold_, stats_interval_);
    }

private:
    void data_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double value = msg->data;

        if (value > alert_threshold_) {
            RCLCPP_WARN(this->get_logger(),
                       "Received: %.2f [ALERT: Above threshold!]", value);
        } else {
            RCLCPP_INFO(this->get_logger(),
                       "Received: %.2f", value);
        }
    }

    void request_stats()
    {
        // Check if service is available
        if (!client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Service /process_stats not available");
            return;
        }

        // Create request
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        // Call service asynchronously
        auto result_future = client_->async_send_request(request);

        // Wait for result
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(),
                           "Statistics: %s", result->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(),
                           "Stats service failed: %s", result->message.c_str());
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call stats service");
        }
    }

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
