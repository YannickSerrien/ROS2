/**
 * @file countdown_server.cpp
 * @brief Countdown action server implementation - SOLUTION
 *
 * Complete implementation of countdown action server with:
 * - Goal validation
 * - Periodic feedback
 * - Cancellation handling
 * - Proper result returning
 */

#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_countdown/action/countdown.hpp"

using namespace std::chrono_literals;
using Countdown = action_countdown::action::Countdown;
using GoalHandleCountdown = rclcpp_action::ServerGoalHandle<Countdown>;

class CountdownActionServer : public rclcpp::Node
{
public:
    CountdownActionServer()
    : Node("countdown_action_server")
    {
        using namespace std::placeholders;

        // Create action server
        this->action_server_ = rclcpp_action::create_server<Countdown>(
            this,
            "countdown",
            std::bind(&CountdownActionServer::handle_goal, this, _1, _2),
            std::bind(&CountdownActionServer::handle_cancel, this, _1),
            std::bind(&CountdownActionServer::handle_accepted, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Countdown Action Server started");
    }

private:
    rclcpp_action::Server<Countdown>::SharedPtr action_server_;

    /**
     * Handle incoming goal requests
     * Validate duration is between 1 and 300 seconds
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Countdown::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(this->get_logger(), "Received goal request: %d seconds", goal->duration);

        // Validate duration
        if (goal->duration <= 0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: duration must be positive");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->duration > 300) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: duration too long (max 300 seconds)");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Accept the goal
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * Handle cancellation requests
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCountdown> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * Execute goal in separate thread
     */
    void handle_accepted(const std::shared_ptr<GoalHandleCountdown> goal_handle)
    {
        // Execute in separate thread to avoid blocking
        std::thread{std::bind(&CountdownActionServer::execute, this, std::placeholders::_1), goal_handle}
            .detach();
    }

    /**
     * Main execution logic - countdown with feedback
     */
    void execute(const std::shared_ptr<GoalHandleCountdown> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing countdown");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Countdown::Feedback>();
        auto result = std::make_shared<Countdown::Result>();

        // Countdown loop
        for (int32_t i = goal->duration; i > 0; --i) {
            // Check for cancellation
            if (goal_handle->is_canceling()) {
                result->completed = false;
                result->message = "Countdown canceled";
                result->elapsed_time = goal->duration - i;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Update and publish feedback
            feedback->time_remaining = i;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Countdown: %d seconds remaining", i);

            // Wait 1 second
            std::this_thread::sleep_for(1s);
        }

        // Success!
        result->completed = true;
        result->message = "Countdown completed successfully";
        result->elapsed_time = goal->duration;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Countdown complete!");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountdownActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
