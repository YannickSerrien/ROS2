/**
 * @file configurable_robot.cpp
 * @brief Robot node with parameter-based configuration
 *
 * Demonstrates:
 * - Using parameters for robot configuration
 * - Loading parameters from YAML files
 * - Parameter-driven behavior
 * - Validation of parameter values
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class ConfigurableRobotNode : public rclcpp::Node
{
public:
    ConfigurableRobotNode() : Node("configurable_robot")
    {
        // Declare robot configuration parameters
        this->declare_parameter("robot_name", "RobotAlpha");
        this->declare_parameter("max_speed", 1.0);          // m/s
        this->declare_parameter("max_acceleration", 0.5);   // m/s²
        this->declare_parameter("wheel_diameter", 0.1);     // meters
        this->declare_parameter("publish_rate", 1.0);       // Hz
        this->declare_parameter("enable_safety", true);

        // Get parameters
        robot_name_ = this->get_parameter("robot_name").as_string();
        max_speed_ = this->get_parameter("max_speed").as_double();
        max_acceleration_ = this->get_parameter("max_acceleration").as_double();
        wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        safety_enabled_ = this->get_parameter("enable_safety").as_bool();

        // Validate parameters
        if (!validate_parameters()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid parameters! Using defaults.");
            // Could throw or exit here in production
        }

        // Log configuration
        log_configuration();

        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("robot_status", 10);

        // Create timer with configured rate
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            period,
            std::bind(&ConfigurableRobotNode::publish_status, this));
    }

private:
    bool validate_parameters()
    {
        bool valid = true;

        if (max_speed_ <= 0.0 || max_speed_ > 10.0) {
            RCLCPP_WARN(this->get_logger(),
                       "max_speed (%.2f) out of range [0.01, 10.0]", max_speed_);
            valid = false;
        }

        if (max_acceleration_ <= 0.0 || max_acceleration_ > 5.0) {
            RCLCPP_WARN(this->get_logger(),
                       "max_acceleration (%.2f) out of range [0.01, 5.0]", max_acceleration_);
            valid = false;
        }

        if (wheel_diameter_ <= 0.0 || wheel_diameter_ > 1.0) {
            RCLCPP_WARN(this->get_logger(),
                       "wheel_diameter (%.3f) out of range [0.001, 1.0]", wheel_diameter_);
            valid = false;
        }

        return valid;
    }

    void log_configuration()
    {
        RCLCPP_INFO(this->get_logger(), "=== Robot Configuration ===");
        RCLCPP_INFO(this->get_logger(), "Robot Name: %s", robot_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Max Speed: %.2f m/s", max_speed_);
        RCLCPP_INFO(this->get_logger(), "Max Acceleration: %.2f m/s²", max_acceleration_);
        RCLCPP_INFO(this->get_logger(), "Wheel Diameter: %.3f m", wheel_diameter_);
        RCLCPP_INFO(this->get_logger(), "Safety Enabled: %s", safety_enabled_ ? "Yes" : "No");
        RCLCPP_INFO(this->get_logger(), "==========================");
    }

    void publish_status()
    {
        auto msg = std_msgs::msg::String();
        msg.data = robot_name_ + " is operational. Speed limit: " +
                   std::to_string(max_speed_) + " m/s";

        publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published status");
    }

    // Parameters
    std::string robot_name_;
    double max_speed_;
    double max_acceleration_;
    double wheel_diameter_;
    bool safety_enabled_;

    // ROS objects
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting configurable robot node");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Override parameters with:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  ros2 run parameters_example configurable_robot --ros-args -p robot_name:=Beta -p max_speed:=2.0");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Or load from YAML:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  ros2 run parameters_example configurable_robot --ros-args --params-file config/robot_params.yaml");

    auto node = std::make_shared<ConfigurableRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
