/**
 * @file fibonacci_server.cpp
 * @brief Action server that computes Fibonacci sequences
 *
 * Demonstrates:
 * - Creating an action server
 * - Handling goal requests (accept/reject)
 * - Publishing periodic feedback
 * - Handling cancellation requests
 * - Returning final results
 *
 * Used in: Module 3 - Actions
 */

#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "actions_example/action/fibonacci.hpp"

using namespace std::chrono_literals;
using Fibonacci = actions_example::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FibonacciActionServer : public rclcpp::Node
{
public:
    FibonacciActionServer()
    : Node("fibonacci_action_server")
    {
        using namespace std::placeholders;

        // Create action server
        this->action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
            std::bind(&FibonacciActionServer::handle_cancel, this, _1),
            std::bind(&FibonacciActionServer::handle_accepted, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Fibonacci Action Server started");
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    /**
     * Handle incoming goal requests
     * Return ACCEPT_AND_EXECUTE or REJECT
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        (void)uuid;  // Unused parameter

        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);

        // Validation: reject if order is invalid
        if (goal->order < 0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: order must be non-negative");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->order > 50) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: order too large (max 50)");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Accept the goal
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * Handle cancellation requests
     * Return ACCEPT or REJECT
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * Execute the goal in a new thread
     * Called when goal is accepted
     */
    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        // Execute in separate thread to avoid blocking
        std::thread{std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1), goal_handle}
            .detach();
    }

    /**
     * Main execution logic
     * Computes Fibonacci sequence with periodic feedback
     */
    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto result = std::make_shared<Fibonacci::Result>();

        // Initialize Fibonacci sequence
        std::vector<int32_t> sequence = {0, 1};

        // Compute sequence
        for (int i = 1; i < goal->order; ++i) {
            // Check for cancellation
            if (goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Compute next Fibonacci number
            int32_t next_value = sequence[i] + sequence[i - 1];
            sequence.push_back(next_value);

            // Publish feedback
            feedback->partial_sequence = sequence;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Published feedback: %d numbers computed",
                        static_cast<int>(sequence.size()));

            // Simulate computation time
            std::this_thread::sleep_for(500ms);
        }

        // Check one final time for cancellation
        if (goal_handle->is_canceling()) {
            result->sequence = sequence;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        // Success! Return the result
        result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
