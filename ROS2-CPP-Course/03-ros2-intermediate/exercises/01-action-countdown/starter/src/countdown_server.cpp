/**
 * @file countdown_server.cpp
 * @brief Countdown action server implementation
 *
 * TODO: Complete the action server implementation
 *
 * Your tasks:
 * 1. Implement handle_goal to validate duration (1-300 seconds)
 * 2. Implement handle_cancel to accept cancellation requests
 * 3. Implement execute to run countdown with feedback
 * 4. Handle cancellation during execution
 * 5. Return appropriate results
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

        // TODO: Create action server
        // Hint: Use rclcpp_action::create_server with:
        //   - this (node pointer)
        //   - "countdown" (action name)
        //   - bind handle_goal, handle_cancel, handle_accepted

        // YOUR CODE HERE


        RCLCPP_INFO(this->get_logger(), "Countdown Action Server started");
    }

private:
    // TODO: Declare action server member variable
    // Hint: rclcpp_action::Server<Countdown>::SharedPtr action_server_;


    /**
     * TODO: Handle incoming goal requests
     * Validate: duration must be between 1 and 300 seconds
     * Return: ACCEPT_AND_EXECUTE or REJECT
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Countdown::Goal> goal)
    {
        (void)uuid;  // Unused

        RCLCPP_INFO(this->get_logger(), "Received goal request: %d seconds", goal->duration);

        // TODO: Validate goal duration
        // Reject if duration <= 0
        // Reject if duration > 300
        // Otherwise accept

        // YOUR CODE HERE

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // FIXME
    }

    /**
     * TODO: Handle cancellation requests
     * Return: ACCEPT
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCountdown> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;

        // TODO: Return ACCEPT to allow cancellation
        // YOUR CODE HERE

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
     * TODO: Main execution logic
     * Implement countdown with:
     * - Loop from duration down to 0
     * - Publish feedback every second
     * - Check for cancellation each iteration
     * - Return success or canceled result
     */
    void execute(const std::shared_ptr<GoalHandleCountdown> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Countdown::Feedback>();
        auto result = std::make_shared<Countdown::Result>();

        // TODO: Implement countdown loop
        // Hints:
        // 1. Loop from goal->duration down to 0
        // 2. Each iteration:
        //    a. Check if goal_handle->is_canceling() - if yes, cancel
        //    b. Update feedback->time_remaining
        //    c. Publish feedback with goal_handle->publish_feedback(feedback)
        //    d. Log current countdown value
        //    e. Sleep for 1 second (std::this_thread::sleep_for(1s))
        // 3. After loop, set result and call goal_handle->succeed(result)

        // YOUR CODE HERE


        // FIXME: Remove this placeholder
        result->completed = true;
        result->message = "Countdown completed";
        result->elapsed_time = goal->duration;
        goal_handle->succeed(result);
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
