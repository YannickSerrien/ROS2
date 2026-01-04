/**
 * @file processor_node.cpp
 * @brief Processes sensor data and computes statistics
 *
 * Demonstrates:
 * - Subscribing with remapped topics
 * - Processing and republishing data
 * - Running average calculation
 * - Parameter configuration
 *
 * Parameters:
 * - window_size: Number of samples for moving average
 * - warning_threshold: Temperature threshold for warnings
 *
 * Used in: Module 3 - Launch Files Example
 */

#include <memory>
#include <vector>
#include <numeric>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class ProcessorNode : public rclcpp::Node
{
public:
    ProcessorNode()
    : Node("processor_node")
    {
        // Declare parameters
        this->declare_parameter("window_size", 10);
        this->declare_parameter("warning_threshold", 28.0);

        // Get parameters
        window_size_ = this->get_parameter("window_size").as_int();
        warning_threshold_ = this->get_parameter("warning_threshold").as_double();

        // Create subscriber
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "temperature",
            10,
            std::bind(&ProcessorNode::temperature_callback, this, std::placeholders::_1)
        );

        // Create publisher for processed data
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("temperature_avg", 10);

        RCLCPP_INFO(this->get_logger(), "Processor node started");
        RCLCPP_INFO(this->get_logger(), "Window size: %d samples", window_size_);
        RCLCPP_INFO(this->get_logger(), "Warning threshold: %.1f°C", warning_threshold_);
    }

private:
    void temperature_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Add to window
        temperature_window_.push_back(msg->data);

        // Maintain window size
        if (static_cast<int>(temperature_window_.size()) > window_size_) {
            temperature_window_.erase(temperature_window_.begin());
        }

        // Calculate moving average
        double sum = std::accumulate(temperature_window_.begin(),
                                     temperature_window_.end(), 0.0);
        double average = sum / temperature_window_.size();

        // Publish average
        auto avg_msg = std_msgs::msg::Float64();
        avg_msg.data = average;
        publisher_->publish(avg_msg);

        // Check threshold
        if (average > warning_threshold_) {
            RCLCPP_WARN(this->get_logger(),
                       "Temperature above threshold! Current: %.2f°C, Avg: %.2f°C",
                       msg->data, average);
        } else {
            RCLCPP_INFO(this->get_logger(),
                       "Current: %.2f°C, Avg: %.2f°C (last %zu samples)",
                       msg->data, average, temperature_window_.size());
        }
    }

    int window_size_;
    double warning_threshold_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

    std::vector<double> temperature_window_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
