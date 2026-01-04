/**
 * @file multi_timer_node.cpp
 * @brief Node with multiple independent timers
 *
 * Demonstrates:
 * - Multiple timers in one node
 * - Different timer frequencies
 * - Canceling and resetting timers
 * - One-shot timers
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MultiTimerNode : public rclcpp::Node
{
public:
    MultiTimerNode() : Node("multi_timer"), continuous_count_(0), oneshot_created_(false)
    {
        // Timer 1: Fast continuous (200ms)
        fast_timer_ = this->create_wall_timer(
            200ms,
            [this]() {
                RCLCPP_INFO(this->get_logger(), "[Fast] Tick at 5 Hz");
            });

        // Timer 2: Slow continuous (1s)
        slow_timer_ = this->create_wall_timer(
            1s,
            [this]() {
                continuous_count_++;
                RCLCPP_INFO(this->get_logger(),
                           "[Slow] Continuous timer #%zu", continuous_count_);

                // After 5 slow ticks, create a one-shot timer
                if (continuous_count_ == 5 && !oneshot_created_) {
                    create_oneshot_timer();
                }

                // After 10 slow ticks, cancel fast timer
                if (continuous_count_ == 10) {
                    RCLCPP_WARN(this->get_logger(), "Canceling fast timer!");
                    fast_timer_->cancel();
                }

                // After 15 slow ticks, reset fast timer
                if (continuous_count_ == 15) {
                    RCLCPP_WARN(this->get_logger(), "Resetting fast timer!");
                    fast_timer_->reset();
                }
            });

        // Timer 3: Stats reporting (5s)
        stats_timer_ = this->create_wall_timer(
            5s,
            [this]() {
                RCLCPP_INFO(this->get_logger(),
                           "[Stats] Runtime: %zu seconds, Slow ticks: %zu",
                           continuous_count_, continuous_count_);
            });

        RCLCPP_INFO(this->get_logger(), "Multi-timer node started");
        RCLCPP_INFO(this->get_logger(), "  Fast timer: 200ms (5 Hz)");
        RCLCPP_INFO(this->get_logger(), "  Slow timer: 1s (1 Hz)");
        RCLCPP_INFO(this->get_logger(), "  Stats timer: 5s (0.2 Hz)");
        RCLCPP_INFO(this->get_logger(), "Watch for one-shot timer at count 5");
        RCLCPP_INFO(this->get_logger(), "Watch for fast timer cancel at count 10");
        RCLCPP_INFO(this->get_logger(), "Watch for fast timer reset at count 15");
    }

private:
    void create_oneshot_timer()
    {
        RCLCPP_WARN(this->get_logger(), "Creating one-shot timer (triggers in 2 seconds)");
        oneshot_created_ = true;

        // One-shot timer: fires once after 2 seconds
        oneshot_timer_ = this->create_wall_timer(
            2s,
            [this]() {
                RCLCPP_ERROR(this->get_logger(), "[ONE-SHOT] This timer fires only once!");
                // Cancel itself after firing
                oneshot_timer_->cancel();
            });
    }

    rclcpp::TimerBase::SharedPtr fast_timer_;
    rclcpp::TimerBase::SharedPtr slow_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr oneshot_timer_;

    size_t continuous_count_;
    bool oneshot_created_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting multi-timer node");
    RCLCPP_INFO(rclcpp::get_logger("main"), "This node demonstrates:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - Multiple independent timers");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - Timer cancel and reset");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - One-shot timers");

    auto node = std::make_shared<MultiTimerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
