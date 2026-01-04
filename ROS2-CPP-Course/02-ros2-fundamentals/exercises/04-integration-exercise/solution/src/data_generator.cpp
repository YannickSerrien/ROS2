/**
 * @file data_generator.cpp
 * @brief Solution: Generate random sensor data
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
        // Declare parameters
        this->declare_parameter("publish_rate", 1.0);
        this->declare_parameter("data_min", 0.0);
        this->declare_parameter("data_max", 100.0);

        // Get parameter values
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        data_min_ = this->get_parameter("data_min").as_double();
        data_max_ = this->get_parameter("data_max").as_double();

        // Initialize random number generator
        std::random_device rd;
        gen_ = std::mt19937(rd());

        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/raw_data", 10);

        // Create timer based on publish rate
        auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            period,
            std::bind(&DataGenerator::publish_data, this));

        RCLCPP_INFO(this->get_logger(),
                    "Data Generator started (rate: %.2f Hz, range: [%.2f, %.2f])",
                    publish_rate_, data_min_, data_max_);
    }

private:
    void publish_data()
    {
        // Generate random value
        std::uniform_real_distribution<> dist(data_min_, data_max_);
        double value = dist(gen_);

        // Create and publish message
        auto msg = std_msgs::msg::Float64();
        msg.data = value;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Publishing: %.2f", value);
    }

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
