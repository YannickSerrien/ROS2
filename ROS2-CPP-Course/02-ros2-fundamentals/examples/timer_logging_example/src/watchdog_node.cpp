/**
 * @file watchdog_node.cpp
 * @brief Watchdog timer pattern for timeout monitoring
 *
 * Demonstrates:
 * - Watchdog timer pattern
 * - Timeout detection
 * - Timer reset on activity
 * - Simulated data source with intermittent failures
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class WatchdogNode : public rclcpp::Node
{
public:
    WatchdogNode() : Node("watchdog"),
                     data_timeout_(false),
                     last_data_time_(this->now()),
                     random_gen_(std::random_device{}())
    {
        // Subscribe to data source
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "data_source",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->data_callback(msg);
            });

        // Watchdog timer: check every 1 second
        watchdog_timer_ = this->create_wall_timer(
            1s,
            std::bind(&WatchdogNode::watchdog_callback, this));

        // Simulated data source: publishes every 500ms (sometimes fails)
        publisher_ = this->create_publisher<std_msgs::msg::String>("data_source", 10);
        data_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&WatchdogNode::publish_data, this));

        RCLCPP_INFO(this->get_logger(), "Watchdog node started");
        RCLCPP_INFO(this->get_logger(), "Monitoring /data_source topic");
        RCLCPP_INFO(this->get_logger(), "Timeout threshold: 3 seconds");
    }

private:
    void data_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Reset watchdog on data reception
        last_data_time_ = this->now();

        if (data_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Data source recovered!");
            data_timeout_ = false;
        }

        RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
    }

    void watchdog_callback()
    {
        // Check time since last data
        auto elapsed = (this->now() - last_data_time_).seconds();

        if (elapsed > 3.0) {  // 3 second timeout
            if (!data_timeout_) {
                RCLCPP_ERROR(this->get_logger(),
                            "TIMEOUT! No data received for %.1f seconds", elapsed);
                data_timeout_ = true;
            } else {
                // Already in timeout state, log periodically
                RCLCPP_ERROR_THROTTLE(this->get_logger(),
                                     *this->get_clock(),
                                     5000,  // Every 5 seconds
                                     "Still in timeout (%.1f seconds)", elapsed);
            }
        } else {
            // Normal operation
            RCLCPP_DEBUG(this->get_logger(),
                        "Watchdog OK (last data: %.1f seconds ago)", elapsed);
        }
    }

    void publish_data()
    {
        // Simulate occasional data source failures (20% chance)
        std::uniform_int_distribution<> dist(1, 100);
        int roll = dist(random_gen_);

        if (roll > 20) {  // 80% success rate
            auto msg = std_msgs::msg::String();
            msg.data = "Data at " + std::to_string(this->now().seconds());
            publisher_->publish(msg);
            RCLCPP_DEBUG(this->get_logger(), "Published data");
        } else {
            RCLCPP_WARN(this->get_logger(), "Simulated data source failure!");
        }
    }

    // Watchdog state
    bool data_timeout_;
    rclcpp::Time last_data_time_;

    // ROS objects
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr data_timer_;

    // Random number generator for failures
    std::mt19937 random_gen_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting watchdog node");
    RCLCPP_INFO(rclcpp::get_logger("main"), "This node:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - Publishes data every 500ms (with simulated failures)");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - Monitors data reception with watchdog timer");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - Detects timeouts after 3 seconds");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Run with --log-level debug to see all details");

    auto node = std::make_shared<WatchdogNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
