/**
 * @file processor_node.cpp
 * @brief Data processor (subscriber + publisher)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <deque>

class ProcessorNode : public rclcpp::Node
{
public:
    ProcessorNode() : Node("processor_node"), window_size_(10)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "raw_data", 10, [this](std_msgs::msg::Float64::SharedPtr msg) {
                window_.push_back(msg->data);
                if (window_.size() > window_size_) window_.pop_front();

                double avg = 0.0;
                for (auto v : window_) avg += v;
                avg /= window_.size();

                auto out_msg = std_msgs::msg::Float64();
                out_msg.data = avg;
                publisher_->publish(out_msg);
                RCLCPP_INFO(this->get_logger(), "Processed: %.2f (avg of %zu samples)", avg, window_.size());
            });

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("processed_data", 10);
        RCLCPP_INFO(this->get_logger(), "Processor node started");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    std::deque<double> window_;
    size_t window_size_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
