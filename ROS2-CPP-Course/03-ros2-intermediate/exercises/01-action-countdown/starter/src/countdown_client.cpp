/**
 * @file countdown_client.cpp
 * @brief Countdown action client implementation
 *
 * TODO: Complete the action client implementation
 *
 * Your tasks:
 * 1. Create action client
 * 2. Send goal with callbacks
 * 3. Handle goal response (accepted/rejected)
 * 4. Display feedback updates
 * 5. Process final result
 */

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_countdown/action/countdown.hpp"

using namespace std::chrono_literals;
using Countdown = action_countdown::action::Countdown;
using GoalHandleCountdown = rclcpp_action::ClientGoalHandle<Countdown>;

class CountdownActionClient : public rclcpp::Node
{
public:
    CountdownActionClient()
    : Node("countdown_action_client")
    {
        // TODO: Create action client
        // Hint: Use rclcpp_action::create_client<Countdown>(this, "countdown")

        // YOUR CODE HERE


        RCLCPP_INFO(this->get_logger(), "Countdown Action Client started");
    }

    /**
     * TODO: Send goal to action server
     * Configure callbacks for:
     * - Goal response (accepted/rejected)
     * - Feedback (progress updates)
     * - Result (final outcome)
     */
    void send_goal(int32_t duration)
    {
        // TODO: Wait for action server
        // Hint: Use client_->wait_for_action_server(timeout)

        // YOUR CODE HERE


        // TODO: Create goal message
        // Hint: auto goal_msg = Countdown::Goal();
        //       goal_msg.duration = duration;

        // YOUR CODE HERE


        RCLCPP_INFO(this->get_logger(), "Sending goal: %d seconds", duration);

        // TODO: Configure send goal options with callbacks
        // Hint: auto send_goal_options = Client::SendGoalOptions();
        //       send_goal_options.goal_response_callback = ...
        //       send_goal_options.feedback_callback = ...
        //       send_goal_options.result_callback = ...

        // YOUR CODE HERE


        // TODO: Send goal asynchronously
        // Hint: client_->async_send_goal(goal_msg, send_goal_options);

        // YOUR CODE HERE

    }

private:
    // TODO: Declare action client member variable
    // Hint: rclcpp_action::Client<Countdown>::SharedPtr client_;


    /**
     * TODO: Handle goal response callback
     * Log whether goal was accepted or rejected
     */
    void goal_response_callback(std::shared_future<GoalHandleCountdown::SharedPtr> future)
    {
        auto goal_handle = future.get();

        // TODO: Check if goal was accepted or rejected
        // Hint: if (!goal_handle) { /* rejected */ } else { /* accepted */ }

        // YOUR CODE HERE

    }

    /**
     * TODO: Handle feedback callback
     * Display time remaining
     */
    void feedback_callback(
        GoalHandleCountdown::SharedPtr,
        const std::shared_ptr<const Countdown::Feedback> feedback)
    {
        // TODO: Log feedback
        // Hint: RCLCPP_INFO(this->get_logger(), "Time remaining: %d", feedback->time_remaining);

        // YOUR CODE HERE

    }

    /**
     * TODO: Handle result callback
     * Display final outcome
     */
    void result_callback(const GoalHandleCountdown::WrappedResult & result)
    {
        // TODO: Check result code
        // Hint: Use switch on result.code (SUCCEEDED, ABORTED, CANCELED)

        // YOUR CODE HERE


        // TODO: Display result details
        // Hint: result.result->completed, result.result->message, result.result->elapsed_time

        // YOUR CODE HERE

    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountdownActionClient>();

    // Send a 10-second countdown goal
    node->send_goal(10);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
