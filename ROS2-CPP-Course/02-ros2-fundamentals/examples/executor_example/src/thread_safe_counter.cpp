/**
 * @file thread_safe_counter.cpp
 * @brief Thread safety demonstration with mutex and atomic
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <mutex>
#include <atomic>
#include <chrono>

using namespace std::chrono_literals;

class ThreadSafeCounterNode : public rclcpp::Node
{
public:
    ThreadSafeCounterNode() : Node("thread_safe_counter"),
                              unsafe_counter_(0),
                              mutex_counter_(0),
                              atomic_counter_(0)
    {
        auto reentrant_group = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // Multiple timers incrementing counters
        for (int i = 0; i < 4; ++i) {
            auto timer = this->create_wall_timer(10ms, [this]() {
                // Unsafe increment (race condition!)
                unsafe_counter_++;

                // Mutex-protected increment (thread-safe)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    mutex_counter_++;
                }

                // Atomic increment (thread-safe, lockless)
                atomic_counter_++;
            }, reentrant_group);

            timers_.push_back(timer);
        }

        // Stats reporting timer
        stats_timer_ = this->create_wall_timer(2s, [this]() {
            RCLCPP_INFO(this->get_logger(), "Counter values:");
            RCLCPP_INFO(this->get_logger(), "  Unsafe:  %zu (may have race conditions!)", unsafe_counter_);
            RCLCPP_INFO(this->get_logger(), "  Mutex:   %zu (thread-safe)", mutex_counter_);
            RCLCPP_INFO(this->get_logger(), "  Atomic:  %zu (thread-safe, lockless)", atomic_counter_.load());

            if (unsafe_counter_ != mutex_counter_) {
                RCLCPP_WARN(this->get_logger(), "Race condition detected! Unsafe != Mutex");
            }
        });

        RCLCPP_INFO(this->get_logger(), "Thread-safe counter started");
        RCLCPP_INFO(this->get_logger(), "Comparing unsafe, mutex-protected, and atomic counters");
    }

private:
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    // Unsafe counter (race condition)
    size_t unsafe_counter_;

    // Mutex-protected counter
    std::mutex mutex_;
    size_t mutex_counter_;

    // Atomic counter (lockless)
    std::atomic<size_t> atomic_counter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThreadSafeCounterNode>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
