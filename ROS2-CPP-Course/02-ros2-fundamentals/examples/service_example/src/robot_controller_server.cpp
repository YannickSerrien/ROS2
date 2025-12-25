/**
 * @file robot_controller_server.cpp
 * @brief Real-world robot controller example with state management
 *
 * Demonstrates:
 * - Service-based robot control
 * - State machine management
 * - Input validation
 * - Multiple related services
 * - Error reporting
 */

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <example_interfaces/srv/set_bool.hpp>
#include <memory>
#include <string>

enum class RobotState {
    IDLE,
    HOMING,
    READY,
    MOVING,
    ERROR
};

class RobotControllerNode : public rclcpp::Node
{
public:
    RobotControllerNode() : Node("robot_controller"), state_(RobotState::IDLE)
    {
        // Home service - initialize robot
        home_service_ = this->create_service<std_srvs::srv::Trigger>(
            "robot/home",
            std::bind(&RobotControllerNode::handle_home, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Start service - start robot motion
        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "robot/start",
            std::bind(&RobotControllerNode::handle_start, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Stop service - stop robot motion
        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "robot/stop",
            std::bind(&RobotControllerNode::handle_stop, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Emergency stop service
        estop_service_ = this->create_service<std_srvs::srv::SetBool>(
            "robot/estop",
            std::bind(&RobotControllerNode::handle_estop, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Reset service - clear errors
        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            "robot/reset",
            std::bind(&RobotControllerNode::handle_reset, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Robot Controller started");
        RCLCPP_INFO(this->get_logger(), "Available services:");
        RCLCPP_INFO(this->get_logger(), "  - /robot/home   (Trigger)");
        RCLCPP_INFO(this->get_logger(), "  - /robot/start  (Trigger)");
        RCLCPP_INFO(this->get_logger(), "  - /robot/stop   (Trigger)");
        RCLCPP_INFO(this->get_logger(), "  - /robot/estop  (SetBool)");
        RCLCPP_INFO(this->get_logger(), "  - /robot/reset  (Trigger)");
        RCLCPP_INFO(this->get_logger(), "Initial state: IDLE");
    }

private:
    void handle_home(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (state_ == RobotState::ERROR) {
            response->success = false;
            response->message = "Cannot home: Robot in ERROR state. Call /robot/reset first.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        if (state_ == RobotState::MOVING) {
            response->success = false;
            response->message = "Cannot home: Robot is moving. Stop first.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Homing robot...");
        state_ = RobotState::HOMING;

        // Simulate homing process
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        state_ = RobotState::READY;
        response->success = true;
        response->message = "Robot homed successfully and ready";

        RCLCPP_INFO(this->get_logger(), "State: READY");
    }

    void handle_start(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (state_ != RobotState::READY) {
            response->success = false;
            response->message = "Cannot start: Robot not ready. Home first.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        state_ = RobotState::MOVING;
        response->success = true;
        response->message = "Robot motion started";

        RCLCPP_INFO(this->get_logger(), "State: MOVING");
    }

    void handle_stop(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (state_ != RobotState::MOVING) {
            response->success = false;
            response->message = "Robot is not moving";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        state_ = RobotState::READY;
        response->success = true;
        response->message = "Robot stopped successfully";

        RCLCPP_INFO(this->get_logger(), "State: READY");
    }

    void handle_estop(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data) {
            RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
            state_ = RobotState::ERROR;
            response->success = true;
            response->message = "E-Stop activated. Robot in ERROR state.";
        } else {
            response->success = false;
            response->message = "E-Stop can only be activated (data: true). Use /robot/reset to recover.";
        }
    }

    void handle_reset(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (state_ != RobotState::ERROR) {
            response->success = false;
            response->message = "Robot not in ERROR state";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Resetting robot from ERROR state...");
        state_ = RobotState::IDLE;

        response->success = true;
        response->message = "Robot reset to IDLE. Call /robot/home to continue.";

        RCLCPP_INFO(this->get_logger(), "State: IDLE");
    }

    std::string state_to_string() const {
        switch (state_) {
            case RobotState::IDLE: return "IDLE";
            case RobotState::HOMING: return "HOMING";
            case RobotState::READY: return "READY";
            case RobotState::MOVING: return "MOVING";
            case RobotState::ERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    RobotState state_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Robot Controller Example");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Typical workflow:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  1. ros2 service call /robot/home std_srvs/srv/Trigger");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  2. ros2 service call /robot/start std_srvs/srv/Trigger");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  3. ros2 service call /robot/stop std_srvs/srv/Trigger");

    auto node = std::make_shared<RobotControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
