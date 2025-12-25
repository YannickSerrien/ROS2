/**
 * @file parameter_node.cpp
 * @brief Basic parameter usage demonstration
 *
 * Demonstrates:
 * - Declaring parameters with defaults
 * - Getting parameter values
 * - Different parameter types (int, double, string, bool, array)
 * - Setting parameters from command line
 */

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

class ParameterNode : public rclcpp::Node
{
public:
    ParameterNode() : Node("parameter_node")
    {
        // Declare parameters with default values
        this->declare_parameter("my_integer", 42);
        this->declare_parameter("my_double", 3.14);
        this->declare_parameter("my_string", "Hello, ROS2!");
        this->declare_parameter("my_bool", true);
        this->declare_parameter("my_array", std::vector<int64_t>{1, 2, 3, 4, 5});

        // Get and log parameter values
        log_parameters();
    }

private:
    void log_parameters()
    {
        // Get integer parameter
        int my_int = this->get_parameter("my_integer").as_int();
        RCLCPP_INFO(this->get_logger(), "my_integer: %d", my_int);

        // Get double parameter
        double my_double = this->get_parameter("my_double").as_double();
        RCLCPP_INFO(this->get_logger(), "my_double: %.2f", my_double);

        // Get string parameter
        std::string my_string = this->get_parameter("my_string").as_string();
        RCLCPP_INFO(this->get_logger(), "my_string: %s", my_string.c_str());

        // Get bool parameter
        bool my_bool = this->get_parameter("my_bool").as_bool();
        RCLCPP_INFO(this->get_logger(), "my_bool: %s", my_bool ? "true" : "false");

        // Get array parameter
        std::vector<int64_t> my_array = this->get_parameter("my_array").as_integer_array();
        RCLCPP_INFO(this->get_logger(), "my_array has %zu elements:", my_array.size());
        for (size_t i = 0; i < my_array.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  [%zu] = %ld", i, my_array[i]);
        }

        RCLCPP_INFO(this->get_logger(), "---");
        RCLCPP_INFO(this->get_logger(), "To override parameters:");
        RCLCPP_INFO(this->get_logger(), "  ros2 run parameters_example parameter_node --ros-args -p my_integer:=100 -p my_string:='Custom'");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParameterNode>();

    RCLCPP_INFO(node->get_logger(), "Parameter node running. Press Ctrl+C to exit.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
