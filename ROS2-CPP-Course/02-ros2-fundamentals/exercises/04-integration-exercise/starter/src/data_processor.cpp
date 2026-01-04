/**
 * @file data_processor.cpp
 * @brief Exercise: Process incoming data and provide statistics
 *
 * TODO: Complete this node to process data and track statistics
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <limits>

class DataProcessor : public rclcpp::Node
{
public:
    DataProcessor() : Node("data_processor"),
                      count_(0),
                      sum_(0.0),
                      min_(std::numeric_limits<double>::max()),
                      max_(std::numeric_limits<double>::lowest())
    {
        // TODO: Declare parameters
        // - scale_factor (double, default: 1.0)
        // - enable_filtering (bool, default: false)
        /* YOUR CODE HERE */

        // TODO: Get parameter values
        /* YOUR CODE HERE */

        // TODO: Create subscriber for /raw_data
        /* YOUR CODE HERE */

        // TODO: Create publisher for /processed_data
        /* YOUR CODE HERE */

        // TODO: Create service /process_stats
        // Hint: this->create_service<std_srvs::srv::Trigger>(...)
        /* YOUR CODE HERE */

        RCLCPP_INFO(this->get_logger(),
                    "Data Processor started (scale: %.2f, filtering: %s)",
                    scale_factor_, enable_filtering_ ? "enabled" : "disabled");
    }

private:
    void data_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // TODO: Process the data (apply scale_factor)
        /* YOUR CODE HERE */

        // TODO: Update statistics (count, sum, min, max)
        /* YOUR CODE HERE */

        // TODO: Publish processed data
        /* YOUR CODE HERE */

        // TODO: Log received and processed values
        /* YOUR CODE HERE */
    }

    void stats_service_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;  // Unused

        // TODO: Calculate average
        /* YOUR CODE HERE */

        // TODO: Build response message with statistics
        // Format: "Count: X, Avg: Y, Min: Z, Max: W"
        /* YOUR CODE HERE */

        // TODO: Log stats request
        /* YOUR CODE HERE */
    }

    // TODO: Declare member variables
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    double scale_factor_;
    bool enable_filtering_;

    // Statistics
    size_t count_;
    double sum_;
    double min_;
    double max_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
