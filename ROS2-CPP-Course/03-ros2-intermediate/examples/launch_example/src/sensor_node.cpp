/**
 * @file sensor_node.cpp
 * @brief Simulated sensor that publishes temperature data
 *
 * Demonstrates:
 * - Node with configurable parameters
 * - Publishing sensor data
 * - Parameter-driven behavior
 *
 * Parameters:
 * - sensor_name: Name of the sensor
 * - publish_rate: Publishing frequency (Hz)
 * - min_temp: Minimum temperature value
 * - max_temp: Maximum temperature value
 *
 * Used in: Module 3 - Launch Files Example
 */

#include <memory>
#include <random>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class SensorNode : public rclcpp::Node
{
public:
    SensorNode()
    : Node("sensor_node"),
      gen_(rd_())
    {
        // Declare parameters with defaults
        this->declare_parameter("sensor_name", "temperature_sensor");
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("min_temp", 15.0);
        this->declare_parameter("max_temp", 30.0);

        // Get parameters
        sensor_name_ = this->get_parameter("sensor_name").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        min_temp_ = this->get_parameter("min_temp").as_double();
        max_temp_ = this->get_parameter("max_temp").as_double();

        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("temperature", 10);

        // Create timer based on publish rate
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            period,
            std::bind(&SensorNode::publish_temperature, this)
        );

        // Initialize random distribution
        dist_ = std::uniform_real_distribution<double>(min_temp_, max_temp_);

        RCLCPP_INFO(this->get_logger(), "Sensor node '%s' started", sensor_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing at %.1f Hz", publish_rate);
        RCLCPP_INFO(this->get_logger(), "Temperature range: [%.1f, %.1f]°C",
                   min_temp_, max_temp_);
    }

private:
    void publish_temperature()
    {
        // Generate random temperature within range
        double temperature = dist_(gen_);

        // Publish message
        auto msg = std_msgs::msg::Float64();
        msg.data = temperature;
        publisher_->publish(msg);

        RCLCPP_DEBUG(this->get_logger(), "%s: %.2f°C", sensor_name_.c_str(), temperature);
    }

    std::string sensor_name_;
    double min_temp_;
    double max_temp_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
