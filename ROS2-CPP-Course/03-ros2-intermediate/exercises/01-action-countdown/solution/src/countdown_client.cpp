/**
 * @file countdown_client.cpp
 * @brief Countdown action client implementation - SOLUTION
 *
 * Complete implementation of countdown action client with:
 * - Goal sending with callbacks
 * - Feedback handling
 * - Result processing
 */

#include <memory>
<parameter name="chrono">
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
        // Create action client
        this->client_ = rclcpp_action::create_client<Countdown>(this, "countdown");

        RCLCPP_INFO(this->get_logger(), "Countdown Action Client started");
    }

    /**
     * Send goal to action server with callbacks
     */
    void send_goal(int32_t duration)
    {
        // Wait for action server
        if (!this->client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // Create goal message
        auto goal_msg = Countdown::Goal();
        goal_msg.duration = duration;

        RCLCPP_INFO(this->get_logger(), "Sending goal: %d seconds", duration);

        // Configure send goal options with callbacks
        auto send_goal_options = rclcpp_action::Client<Countdown>::SendGoalOptions();

        // Goal response callback
        send_goal_options.goal_response_callback =
            std::bind(&CountdownActionClient::goal_response_callback, this, std::placeholders::_1);

        // Feedback callback
        send_goal_options.feedback_callback =
            std::bind(&CountdownActionClient::feedback_callback, this,
                      std::placeholders::_1, std::placeholders::_2);

        // Result callback
        send_goal_options.result_callback =
            std::bind(&CountdownActionClient::result_callback, this, std::placeholders::_1);

        // Send goal asynchronously
        this->client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Countdown>::SharedPtr client_;

    /**
     * Handle goal response (accepted or rejected)
     */
    void goal_response_callback(std::shared_future<GoalHandleCountdown::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    /**
     * Handle feedback updates
     */
    void feedback_callback(
        GoalHandleCountdown::SharedPtr,
        const std::shared_ptr<const Countdown::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Time remaining: %d seconds", feedback->time_remaining);
    }

    /**
     * Handle final result
     */
    void result_callback(const GoalHandleCountdown::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        // Display result details
        RCLCPP_INFO(this->get_logger(), "Result:");
        RCLCPP_INFO(this->get_logger(), "  Completed: %s",
                   result.result->completed ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  Message: %s", result.result->message.c_str());
        RCLCPP_INFO(this->get_logger(), "  Elapsed time: %d seconds", result.result->elapsed_time);
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
