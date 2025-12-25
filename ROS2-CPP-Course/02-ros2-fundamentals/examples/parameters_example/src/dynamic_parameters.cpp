/**
 * @file dynamic_parameters.cpp
 * @brief Dynamic parameter updates with callbacks
 *
 * Demonstrates:
 * - Parameter change callbacks
 * - Dynamic reconfiguration at runtime
 * - Parameter validation in callbacks
 * - Rejecting invalid parameter changes
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class DynamicParametersNode : public rclcpp::Node
{
public:
    DynamicParametersNode() : Node("dynamic_parameters"), count_(0)
    {
        // Declare parameters
        this->declare_parameter("message_prefix", "Status");
        this->declare_parameter("publish_rate", 1.0);  // Hz
        this->declare_parameter("max_count", 10);

        // Get initial values
        message_prefix_ = this->get_parameter("message_prefix").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        max_count_ = this->get_parameter("max_count").as_int();

        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("dynamic_status", 10);

        // Create timer with initial rate
        update_timer();

        // Register parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DynamicParametersNode::parameters_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Dynamic Parameters Node started");
        RCLCPP_INFO(this->get_logger(), "Try changing parameters at runtime:");
        RCLCPP_INFO(this->get_logger(), "  ros2 param set /dynamic_parameters message_prefix 'New Message'");
        RCLCPP_INFO(this->get_logger(), "  ros2 param set /dynamic_parameters publish_rate 2.0");
        RCLCPP_INFO(this->get_logger(), "  ros2 param set /dynamic_parameters max_count 20");
    }

private:
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto & param : parameters) {
            RCLCPP_INFO(this->get_logger(),
                       "Parameter change requested: %s", param.get_name().c_str());

            if (param.get_name() == "message_prefix") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    std::string new_prefix = param.as_string();
                    if (new_prefix.empty()) {
                        result.successful = false;
                        result.reason = "message_prefix cannot be empty";
                        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
                    } else {
                        message_prefix_ = new_prefix;
                        RCLCPP_INFO(this->get_logger(),
                                   "Updated message_prefix to: %s", message_prefix_.c_str());
                    }
                }
            }
            else if (param.get_name() == "publish_rate") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    double new_rate = param.as_double();
                    if (new_rate <= 0.0 || new_rate > 100.0) {
                        result.successful = false;
                        result.reason = "publish_rate must be in range (0.0, 100.0]";
                        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
                    } else {
                        publish_rate_ = new_rate;
                        RCLCPP_INFO(this->get_logger(),
                                   "Updated publish_rate to: %.2f Hz", publish_rate_);
                        // Update timer with new rate
                        update_timer();
                    }
                }
            }
            else if (param.get_name() == "max_count") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                    int new_max = param.as_int();
                    if (new_max <= 0) {
                        result.successful = false;
                        result.reason = "max_count must be positive";
                        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
                    } else {
                        max_count_ = new_max;
                        RCLCPP_INFO(this->get_logger(),
                                   "Updated max_count to: %d", max_count_);
                    }
                }
            }
        }

        return result;
    }

    void update_timer()
    {
        // Cancel existing timer
        if (timer_) {
            timer_->cancel();
        }

        // Create new timer with updated rate
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
        timer_ = this->create_wall_timer(
            period,
            std::bind(&DynamicParametersNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Timer updated to %.2f Hz", publish_rate_);
    }

    void timer_callback()
    {
        if (count_ < max_count_) {
            auto msg = std_msgs::msg::String();
            msg.data = message_prefix_ + " #" + std::to_string(count_++);

            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(),
                                *this->get_clock(),
                                5000,  // Log every 5 seconds
                                "Reached max_count (%d). Increase max_count to continue.",
                                max_count_);
        }
    }

    // Parameters
    std::string message_prefix_;
    double publish_rate_;
    int max_count_;
    int count_;

    // ROS objects
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicParametersNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
