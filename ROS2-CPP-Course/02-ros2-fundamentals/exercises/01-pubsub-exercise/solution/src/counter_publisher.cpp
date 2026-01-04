/**
 * @file counter_publisher.cpp
 * @brief Solution: Publisher that sends incrementing integers
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CounterPublisher : public rclcpp::Node
{
public:
    CounterPublisher() : Node("counter_publisher"), counter_(0)
    {
        // Create publisher for Int32 messages on /counter topic
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/counter", 10);

        // Create timer that fires every 1 second
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&CounterPublisher::publish_counter, this));

        RCLCPP_INFO(this->get_logger(), "Starting Counter Publisher");
    }

private:
    void publish_counter()
    {
        // Create and populate message
        auto msg = std_msgs::msg::Int32();
        msg.data = counter_;

        // Publish the message
        publisher_->publish(msg);

        // Log the published value
        RCLCPP_INFO(this->get_logger(), "Publishing: %d", counter_);

        // Increment for next iteration
        counter_++;
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CounterPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
