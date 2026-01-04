/**
 * @file temp_sensor_node.cpp
 * @brief Temperature sensor simulator
 *
 * TODO: Complete the temperature sensor node implementation
 */

#include <rclcpp/rclcpp.hpp>
#include "temperature_monitor/msg/temperature_reading.hpp"
#include <random>
#include <chrono>

class TempSensorNode : public rclcpp::Node
{
public:
    TempSensorNode() : Node("temp_sensor")
    {
        // TODO: Declare parameters
        // - room_name (string)
        // - sensor_id (string)
        // - base_temperature (double, default: 20.0)
        // - temperature_variance (double, default: 5.0)
        // - humidity_base (double, default: 50.0)
        // - publish_rate (double, default: 1.0)

        // TODO: Get parameter values

        // TODO: Initialize random number generator for temperature variations
        // Hint: Use std::random_device and std::mt19937

        // TODO: Initialize current temperature to base temperature
        current_temp_ = base_temperature_;

        // TODO: Create publisher for /temperatures topic

        // TODO: Create timer based on publish_rate

        RCLCPP_INFO(this->get_logger(),
                    "Temperature sensor started: %s (%s)",
                    room_name_.c_str(), sensor_id_.c_str());
    }

private:
    void publish_reading()
    {
        // TODO: Generate realistic temperature change (gradual, not random jumps)
        // Hint: Change current_temp_ by small random amount each time
        //       Use std::uniform_real_distribution for small delta

        // TODO: Generate humidity (can be random or based on temperature)

        // TODO: Create TemperatureReading message
        auto msg = temperature_monitor::msg::TemperatureReading();
        // TODO: Fill in all message fields (sensor_id, room_name, temperature, humidity, timestamp)
        // Hint: For timestamp use: this->now().nanoseconds()

        // TODO: Publish the message

        // TODO: Log the reading (every 10th reading to avoid spam)
        if (/* condition for every 10th */) {
            RCLCPP_INFO(this->get_logger(),
                       "[%s] Temp: %.1f°C, Humidity: %.1f%%",
                       room_name_.c_str(), msg.temperature, msg.humidity);
        }
    }

    // TODO: Declare member variables
    rclcpp::Publisher<temperature_monitor::msg::TemperatureReading>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 gen_;

    std::string room_name_;
    std::string sensor_id_;
    double base_temperature_;
    double temperature_variance_;
    double humidity_base_;
    double publish_rate_;

    double current_temp_;
    int reading_count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TempSensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
