/**
 * @file controller_node.cpp
 * @brief Temperature-based controller (simulated HVAC)
 *
 * Demonstrates:
 * - Subscribing to processed data
 * - Publishing control commands
 * - Namespace usage
 * - Parameter-based control logic
 *
 * Parameters:
 * - target_temp: Desired temperature setpoint
 * - tolerance: Temperature tolerance (±°C)
 * - max_power: Maximum control power (%)
 *
 * Used in: Module 3 - Launch Files Example
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
    : Node("controller_node")
    {
        // Declare parameters
        this->declare_parameter("target_temp", 22.0);
        this->declare_parameter("tolerance", 2.0);
        this->declare_parameter("max_power", 100.0);

        // Get parameters
        target_temp_ = this->get_parameter("target_temp").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        max_power_ = this->get_parameter("max_power").as_double();

        // Create subscriber
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "temperature_avg",
            10,
            std::bind(&ControllerNode::temp_callback, this, std::placeholders::_1)
        );

        // Create publisher for control commands
        control_publisher_ = this->create_publisher<std_msgs::msg::Float64>("hvac_power", 10);

        RCLCPP_INFO(this->get_logger(), "Controller node started");
        RCLCPP_INFO(this->get_logger(), "Target: %.1f°C ± %.1f°C", target_temp_, tolerance_);
        RCLCPP_INFO(this->get_logger(), "Max power: %.0f%%", max_power_);
    }

private:
    void temp_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double current_temp = msg->data;
        double error = target_temp_ - current_temp;

        // Simple proportional control
        double power = 0.0;

        if (std::abs(error) > tolerance_) {
            // Proportional gain
            double kp = 5.0;
            power = kp * error;

            // Clamp to max power
            if (power > max_power_) power = max_power_;
            if (power < -max_power_) power = -max_power_;
        }

        // Publish control command
        auto control_msg = std_msgs::msg::Float64();
        control_msg.data = power;
        control_publisher_->publish(control_msg);

        // Log control action
        std::string action;
        if (power > 0.0) {
            action = "HEATING";
        } else if (power < 0.0) {
            action = "COOLING";
        } else {
            action = "OFF";
        }

        RCLCPP_INFO(this->get_logger(),
                   "Temp: %.2f°C | Target: %.1f°C | Error: %+.2f°C | %s (%.0f%%)",
                   current_temp, target_temp_, error, action.c_str(), std::abs(power));
    }

    double target_temp_;
    double tolerance_;
    double max_power_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
