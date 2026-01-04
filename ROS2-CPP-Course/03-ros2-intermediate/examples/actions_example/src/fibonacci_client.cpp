/**
 * @file fibonacci_client.cpp
 * @brief Action client that requests Fibonacci sequences
 *
 * Demonstrates:
 * - Creating an action client
 * - Sending goals with callbacks
 * - Processing feedback updates
 * - Handling results
 * - Canceling goals
 *
 * Used in: Module 3 - Actions
 */

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "actions_example/action/fibonacci.hpp"

using namespace std::chrono_literals;
using Fibonacci = actions_example::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

class FibonacciActionClient : public rclcpp::Node
{
public:
    FibonacciActionClient()
    : Node("fibonacci_action_client")
    {
        // Create action client
        this->client_ = rclcpp_action::create_client<Fibonacci>(
            this,
            "fibonacci"
        );

        // Timer to send goals periodically (for demo purposes)
        this->timer_ = this->create_wall_timer(
            10s,
            std::bind(&FibonacciActionClient::send_goal, this)
        );

        RCLCPP_INFO(this->get_logger(), "Fibonacci Action Client started");
    }

    /**
     * Send a goal to the action server
     */
    void send_goal()
    {
        // Wait for action server to be available
        if (!this->client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // Create goal message
        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;  // Request 10th Fibonacci number

        RCLCPP_INFO(this->get_logger(), "Sending goal: order = %d", goal_msg.order);

        // Configure send goal options with callbacks
        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

        // Goal response callback (accepted or rejected)
        send_goal_options.goal_response_callback =
            std::bind(&FibonacciActionClient::goal_response_callback, this, std::placeholders::_1);

        // Feedback callback (progress updates)
        send_goal_options.feedback_callback =
            std::bind(&FibonacciActionClient::feedback_callback, this,
                      std::placeholders::_1, std::placeholders::_2);

        // Result callback (final outcome)
        send_goal_options.result_callback =
            std::bind(&FibonacciActionClient::result_callback, this, std::placeholders::_1);

        // Send goal asynchronously
        this->client_->async_send_goal(goal_msg, send_goal_options);
    }

    /**
     * Send goal and cancel it after a delay (demonstration)
     */
    void send_goal_and_cancel()
    {
        if (!this->client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 20;

        RCLCPP_INFO(this->get_logger(), "Sending goal to be canceled: order = %d", goal_msg.order);

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

        // Store goal handle for later cancellation
        send_goal_options.goal_response_callback =
            [this](std::shared_future<GoalHandleFibonacci::SharedPtr> future) {
                auto goal_handle = future.get();
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "Goal accepted, will cancel in 2 seconds");

                // Cancel after 2 seconds
                this->cancel_timer_ = this->create_wall_timer(
                    2s,
                    [this, goal_handle]() {
                        RCLCPP_INFO(this->get_logger(), "Canceling goal");
                        this->client_->async_cancel_goal(goal_handle);
                        this->cancel_timer_->cancel();
                    }
                );
            };

        send_goal_options.result_callback =
            std::bind(&FibonacciActionClient::result_callback, this, std::placeholders::_1);

        this->client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr cancel_timer_;

    /**
     * Callback when server responds to goal request
     */
    void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    /**
     * Callback for feedback updates
     */
    void feedback_callback(
        GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %zu numbers computed",
                    feedback->partial_sequence.size());

        // Print partial sequence
        std::stringstream ss;
        ss << "Partial sequence: [";
        for (size_t i = 0; i < feedback->partial_sequence.size(); ++i) {
            ss << feedback->partial_sequence[i];
            if (i < feedback->partial_sequence.size() - 1) {
                ss << ", ";
            }
        }
        ss << "]";
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    /**
     * Callback for final result
     */
    void result_callback(const GoalHandleFibonacci::WrappedResult & result)
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
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        // Print final sequence
        std::stringstream ss;
        ss << "Final sequence: [";
        for (size_t i = 0; i < result.result->sequence.size(); ++i) {
            ss << result.result->sequence[i];
            if (i < result.result->sequence.size() - 1) {
                ss << ", ";
            }
        }
        ss << "]";
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciActionClient>();

    // Demonstration: Send goal immediately
    node->send_goal();

    // Optional: Uncomment to test cancellation
    // std::this_thread::sleep_for(3s);
    // node->send_goal_and_cancel();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
