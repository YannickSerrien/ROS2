/**
 * @file robot_sim_node.cpp
 * @brief Exercise: Create a configurable robot simulation node
 *
 * TODO: Complete this node to use parameters for configuration
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class RobotSimNode : public rclcpp::Node
{
public:
    RobotSimNode() : Node("robot_sim")
    {
        // TODO: Declare parameters with default values
        // Hint: this->declare_parameter("name", default_value);

        /* YOUR CODE HERE - Declare robot_name (string, default: "DefaultRobot") */
        /* YOUR CODE HERE - Declare max_speed (double, default: 1.0) */
        /* YOUR CODE HERE - Declare operating_mode (string, default: "manual") */
        /* YOUR CODE HERE - Declare enable_sensors (bool, default: true) */

        // TODO: Read parameter values
        /* YOUR CODE HERE - Get all parameter values and store in member variables */

        // TODO: Set up parameter change callback
        // Hint: param_callback_handle_ = this->add_on_set_parameters_callback(...)
        /* YOUR CODE HERE */

        // TODO: Create a timer to publish status every 2 seconds
        /* YOUR CODE HERE */

        // Log startup configuration
        log_configuration();
    }

private:
    void log_configuration()
    {
        RCLCPP_INFO(this->get_logger(), "Robot Simulation Node started");
        RCLCPP_INFO(this->get_logger(), "Configuration:");

        // TODO: Log all parameter values
        /* YOUR CODE HERE */
    }

    void publish_status()
    {
        // TODO: Log current robot status
        // Format: "Status - Name: X, Speed: Y m/s, Mode: Z, Sensors: ON/OFF"
        /* YOUR CODE HERE */
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Success";

        for (const auto & param : parameters) {
            // TODO: Validate and handle parameter changes

            if (param.get_name() == "robot_name") {
                // TODO: Validate robot name (non-empty)
                /* YOUR CODE HERE */
            }
            else if (param.get_name() == "max_speed") {
                // TODO: Validate max_speed (range 0.1 to 5.0)
                /* YOUR CODE HERE */
            }
            else if (param.get_name() == "operating_mode") {
                // TODO: Validate operating_mode ("manual", "auto", or "standby")
                /* YOUR CODE HERE */
            }
            else if (param.get_name() == "enable_sensors") {
                // TODO: Handle enable_sensors change (always valid)
                /* YOUR CODE HERE */
            }
        }

        return result;
    }

    // TODO: Declare member variables to store parameter values
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

    // TODO: Create and spin the node
    /* YOUR CODE HERE */

    rclcpp::shutdown();
    return 0;
}
