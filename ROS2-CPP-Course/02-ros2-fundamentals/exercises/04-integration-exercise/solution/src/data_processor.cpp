/**
 * @file data_processor.cpp
 * @brief Solution: Process incoming data and provide statistics
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <limits>
#include <sstream>

class DataProcessor : public rclcpp::Node
{
public:
    DataProcessor() : Node("data_processor"),
                      count_(0),
                      sum_(0.0),
                      min_(std::numeric_limits<double>::max()),
                      max_(std::numeric_limits<double>::lowest())
    {
        // Declare parameters
        this->declare_parameter("scale_factor", 1.0);
        this->declare_parameter("enable_filtering", false);

        // Get parameter values
        scale_factor_ = this->get_parameter("scale_factor").as_double();
        enable_filtering_ = this->get_parameter("enable_filtering").as_bool();

        // Create subscriber
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/raw_data",
            10,
            std::bind(&DataProcessor::data_callback, this, std::placeholders::_1));

        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/processed_data", 10);

        // Create service
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "/process_stats",
            std::bind(&DataProcessor::stats_service_callback, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(),
                    "Data Processor started (scale: %.2f, filtering: %s)",
                    scale_factor_, enable_filtering_ ? "enabled" : "disabled");
    }

private:
    void data_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double raw_value = msg->data;

        // Process data (apply scaling)
        double processed_value = raw_value * scale_factor_;

        // Update statistics
        count_++;
        sum_ += processed_value;
        min_ = std::min(min_, processed_value);
        max_ = std::max(max_, processed_value);

        // Publish processed data
        auto out_msg = std_msgs::msg::Float64();
        out_msg.data = processed_value;
        publisher_->publish(out_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Received: %.2f -> Processed: %.2f",
                    raw_value, processed_value);
    }

    void stats_service_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        if (count_ == 0) {
            response->success = false;
            response->message = "No data received yet";
        } else {
            double avg = sum_ / count_;

            std::ostringstream oss;
            oss << "Count: " << count_
                << ", Avg: " << avg
                << ", Min: " << min_
                << ", Max: " << max_;

            response->success = true;
            response->message = oss.str();

            RCLCPP_INFO(this->get_logger(),
                       "Stats requested - %s", response->message.c_str());
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    double scale_factor_;
    bool enable_filtering_;

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
