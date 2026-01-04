/**
 * @file sensor_node.cpp
 * @brief Simulated sensor publisher
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class SensorNode : public rclcpp::Node
{
public:
    SensorNode() : Node("sensor_node"), random_gen_(std::random_device{}())
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("raw_data", 10);
        timer_ = this->create_wall_timer(100ms, [this]() {
            std::uniform_real_distribution<> dist(20.0, 30.0);
            auto msg = std_msgs::msg::Float64();
            msg.data = dist(random_gen_);
            publisher_->publish(msg);
            RCLCPP_DEBUG(this->get_logger(), "Published: %.2f", msg.data);
        });
        RCLCPP_INFO(this->get_logger(), "Sensor node started, publishing to /raw_data");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 random_gen_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNode>());
    rclcpp::shutdown();
    return 0;
}
