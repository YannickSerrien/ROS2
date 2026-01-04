/**
 * @file robot_sim_node.cpp
 * @brief Solution: Configurable robot simulation node with parameters
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class RobotSimNode : public rclcpp::Node
{
public:
    RobotSimNode() : Node("robot_sim")
    {
        // Declare parameters with defaults
        this->declare_parameter("robot_name", "DefaultRobot");
        this->declare_parameter("max_speed", 1.0);
        this->declare_parameter("operating_mode", "manual");
        this->declare_parameter("enable_sensors", true);

        // Read parameter values
        robot_name_ = this->get_parameter("robot_name").as_string();
        max_speed_ = this->get_parameter("max_speed").as_double();
        operating_mode_ = this->get_parameter("operating_mode").as_string();
        enable_sensors_ = this->get_parameter("enable_sensors").as_bool();

        // Set up parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&RobotSimNode::parameters_callback, this, std::placeholders::_1));

        // Create status timer
        timer_ = this->create_wall_timer(
            2s,
            std::bind(&RobotSimNode::publish_status, this));

        log_configuration();
    }

private:
    void log_configuration()
    {
        RCLCPP_INFO(this->get_logger(), "Robot Simulation Node started");
        RCLCPP_INFO(this->get_logger(), "Configuration:");
        RCLCPP_INFO(this->get_logger(), "  Robot Name: %s", robot_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Max Speed: %.2f m/s", max_speed_);
        RCLCPP_INFO(this->get_logger(), "  Operating Mode: %s", operating_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Sensors Enabled: %s", enable_sensors_ ? "true" : "false");
    }

    void publish_status()
    {
        RCLCPP_INFO(this->get_logger(),
                    "Status - Name: %s, Speed: %.2f m/s, Mode: %s, Sensors: %s",
                    robot_name_.c_str(),
                    max_speed_,
                    operating_mode_.c_str(),
                    enable_sensors_ ? "ON" : "OFF");
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Success";

        for (const auto & param : parameters) {
            if (param.get_name() == "robot_name") {
                std::string new_name = param.as_string();
                if (new_name.empty()) {
                    result.successful = false;
                    result.reason = "Robot name cannot be empty";
                    RCLCPP_WARN(this->get_logger(), "Rejected empty robot name");
                } else {
                    robot_name_ = new_name;
                    RCLCPP_INFO(this->get_logger(),
                               "Parameter 'robot_name' changed to: %s", robot_name_.c_str());
                }
            }
            else if (param.get_name() == "max_speed") {
                double new_speed = param.as_double();
                if (new_speed < 0.1 || new_speed > 5.0) {
                    result.successful = false;
                    result.reason = "max_speed must be between 0.1 and 5.0";
                    RCLCPP_WARN(this->get_logger(),
                               "Invalid max_speed: %.2f (must be between 0.1 and 5.0)", new_speed);
                } else {
                    max_speed_ = new_speed;
                    RCLCPP_INFO(this->get_logger(),
                               "Parameter 'max_speed' changed to: %.2f", max_speed_);
                }
            }
            else if (param.get_name() == "operating_mode") {
                std::string new_mode = param.as_string();
                if (new_mode != "manual" && new_mode != "auto" && new_mode != "standby") {
                    result.successful = false;
                    result.reason = "operating_mode must be 'manual', 'auto', or 'standby'";
                    RCLCPP_WARN(this->get_logger(),
                               "Invalid operating_mode: %s", new_mode.c_str());
                } else {
                    operating_mode_ = new_mode;
                    RCLCPP_INFO(this->get_logger(),
                               "Parameter 'operating_mode' changed to: %s", operating_mode_.c_str());
                }
            }
            else if (param.get_name() == "enable_sensors") {
                enable_sensors_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(),
                           "Parameter 'enable_sensors' changed to: %s",
                           enable_sensors_ ? "true" : "false");
            }
        }

        return result;
    }

    std::string robot_name_;
    double max_speed_;
    std::string operating_mode_;
    bool enable_sensors_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
