/**
 * @file data_generator.cpp
 * @brief Exercise: Generate random sensor data
 *
 * TODO: Complete this node to generate and publish random data
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <random>
#include <chrono>

class DataGenerator : public rclcpp::Node
{
public:
    DataGenerator() : Node("data_generator")
    {
        // TODO: Declare parameters
        // - publish_rate (double, default: 1.0)
        // - data_min (double, default: 0.0)
        // - data_max (double, default: 100.0)
        /* YOUR CODE HERE */

        // TODO: Get parameter values
        /* YOUR CODE HERE */

        // TODO: Initialize random number generator
        // Hint: std::random_device rd; gen_ = std::mt19937(rd());
        /* YOUR CODE HERE */

        // TODO: Create publisher for /raw_data topic (Float64)
        /* YOUR CODE HERE */

        // TODO: Create timer based on publish_rate parameter
        // Hint: Convert rate to period (1.0 / rate_hz)
        /* YOUR CODE HERE */

        RCLCPP_INFO(this->get_logger(),
                    "Data Generator started (rate: %.2f Hz, range: [%.2f, %.2f])",
                    publish_rate_, data_min_, data_max_);
    }

private:
    void publish_data()
    {
        // TODO: Generate random value in range [data_min, data_max]
        // Hint: std::uniform_real_distribution<> dist(min_, max_);
        //       double value = dist(gen_);
        /* YOUR CODE HERE */

        // TODO: Create and publish Float64 message
        /* YOUR CODE HERE */

        // TODO: Log published value
        /* YOUR CODE HERE */
    }

    // TODO: Declare member variables
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 gen_;
    double publish_rate_;
    double data_min_;
    double data_max_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
