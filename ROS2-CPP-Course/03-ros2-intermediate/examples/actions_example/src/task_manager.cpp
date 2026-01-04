/**
 * @file task_manager.cpp
 * @brief Advanced action server managing robot movement tasks
 *
 * Demonstrates:
 * - Complex action server with state management
 * - Realistic robotics scenario (navigation)
 * - Custom action definition usage
 * - Progress calculation and reporting
 * - Error handling and validation
 *
 * Used in: Module 3 - Actions (Advanced Example)
 */

#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "actions_example/action/move_robot.hpp"

using namespace std::chrono_literals;
using MoveRobot = actions_example::action::MoveRobot;
using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

class TaskManager : public rclcpp::Node
{
public:
    TaskManager()
    : Node("task_manager"),
      current_x_(0.0),
      current_y_(0.0),
      current_theta_(0.0)
    {
        using namespace std::placeholders;

        // Create action server for robot movement
        this->action_server_ = rclcpp_action::create_server<MoveRobot>(
            this,
            "move_robot",
            std::bind(&TaskManager::handle_goal, this, _1, _2),
            std::bind(&TaskManager::handle_cancel, this, _1),
            std::bind(&TaskManager::handle_accepted, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Task Manager started");
        RCLCPP_INFO(this->get_logger(), "Initial position: (%.2f, %.2f, %.2f rad)",
                    current_x_, current_y_, current_theta_);
    }

private:
    rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;

    // Robot state (simulated)
    double current_x_;
    double current_y_;
    double current_theta_;

    /**
     * Handle incoming movement goal requests
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveRobot::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(this->get_logger(),
                    "Received move request to (%.2f, %.2f, %.2f rad) at %.2f m/s",
                    goal->target_x, goal->target_y, goal->target_theta, goal->speed);

        // Validation checks
        if (goal->speed <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting: speed must be positive");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->speed > 2.0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting: speed too high (max 2.0 m/s)");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Check if target is too far (safety constraint)
        double distance = calculate_distance(current_x_, current_y_,
                                             goal->target_x, goal->target_y);
        if (distance > 100.0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting: target too far (max 100m)");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * Handle cancellation requests
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancellation request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * Start executing accepted goal
     */
    void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
    {
        // Execute in separate thread
        std::thread{std::bind(&TaskManager::execute, this, std::placeholders::_1), goal_handle}
            .detach();
    }

    /**
     * Main execution: Simulate robot movement
     */
    void execute(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing movement task");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveRobot::Feedback>();
        auto result = std::make_shared<MoveRobot::Result>();

        // Calculate total distance to travel
        double total_distance = calculate_distance(current_x_, current_y_,
                                                   goal->target_x, goal->target_y);

        double start_x = current_x_;
        double start_y = current_y_;

        // Movement simulation parameters
        const double update_rate = 10.0;  // Hz
        const auto update_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate));
        const double distance_per_update = goal->speed / update_rate;

        // Simulate movement
        while (true) {
            // Check for cancellation
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Movement canceled by user";
                result->final_x = current_x_;
                result->final_y = current_y_;
                result->final_theta = current_theta_;
                result->total_distance = calculate_distance(start_x, start_y,
                                                           current_x_, current_y_);
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Calculate remaining distance
            double remaining = calculate_distance(current_x_, current_y_,
                                                  goal->target_x, goal->target_y);

            // Check if reached target
            if (remaining < 0.01) {  // 1cm threshold
                current_x_ = goal->target_x;
                current_y_ = goal->target_y;
                current_theta_ = goal->target_theta;

                result->success = true;
                result->message = "Target reached successfully";
                result->final_x = current_x_;
                result->final_y = current_y_;
                result->final_theta = current_theta_;
                result->total_distance = total_distance;

                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded - reached target");
                return;
            }

            // Move towards target
            double dx = goal->target_x - current_x_;
            double dy = goal->target_y - current_y_;
            double angle = std::atan2(dy, dx);

            // Update position
            current_x_ += distance_per_update * std::cos(angle);
            current_y_ += distance_per_update * std::sin(angle);

            // Gradually rotate towards target orientation
            double theta_diff = goal->target_theta - current_theta_;
            current_theta_ += theta_diff * 0.1;  // 10% per update

            // Publish feedback
            feedback->current_x = current_x_;
            feedback->current_y = current_y_;
            feedback->current_theta = current_theta_;
            feedback->distance_remaining = remaining;
            feedback->percent_complete = ((total_distance - remaining) / total_distance) * 100.0;

            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(),
                       "Position: (%.2f, %.2f) | Remaining: %.2f m | Progress: %.1f%%",
                       current_x_, current_y_, remaining, feedback->percent_complete);

            std::this_thread::sleep_for(update_period);
        }
    }

    /**
     * Calculate Euclidean distance between two points
     */
    double calculate_distance(double x1, double y1, double x2, double y2) const
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
