/**
 * @file counter_publisher.cpp
 * @brief Exercise: Create a publisher that sends incrementing integers
 *
 * TODO: Complete this node to publish integers to /counter topic
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
        // TODO: Create a publisher for std_msgs::msg::Int32 on topic "/counter"
        // Hint: this->create_publisher<MessageType>(topic_name, queue_size)
        publisher_ = /* YOUR CODE HERE */;

        // TODO: Create a timer that calls publish_counter() every 1 second
        // Hint: this->create_wall_timer(period, callback)
        timer_ = /* YOUR CODE HERE */;

        RCLCPP_INFO(this->get_logger(), "Starting Counter Publisher");
    }

private:
    void publish_counter()
    {
        // TODO: Create a message, set its data field, and publish it
        // Hint: auto msg = std_msgs::msg::Int32();
        //       msg.data = counter_;
        //       publisher_->publish(msg);

        /* YOUR CODE HERE */

        // TODO: Log the published value
        // Hint: RCLCPP_INFO(this->get_logger(), "Publishing: %d", counter_);

        /* YOUR CODE HERE */

        // TODO: Increment the counter for next time
        /* YOUR CODE HERE */;
    }

    // TODO: Declare member variables
    // Hint: You need a publisher, a timer, and a counter variable
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // TODO: Create the node and spin it
    // Hint: auto node = std::make_shared<CounterPublisher>();
    //       rclcpp::spin(node);

    /* YOUR CODE HERE */

    rclcpp::shutdown();
    return 0;
}
