/**
 * @file callback_groups_node.cpp
 * @brief Callback groups for controlling concurrent execution
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CallbackGroupsNode : public rclcpp::Node
{
public:
    CallbackGroupsNode() : Node("callback_groups")
    {
        // Mutually exclusive group (callbacks execute one at a time)
        auto exclusive_group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // Reentrant group (callbacks can execute concurrently)
        auto reentrant_group = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // Timers in exclusive group
        timer1_ = this->create_wall_timer(500ms, [this]() {
            RCLCPP_INFO(this->get_logger(), "[Exclusive1] Start");
            std::this_thread::sleep_for(300ms);
            RCLCPP_INFO(this->get_logger(), "[Exclusive1] Done");
        }, exclusive_group);

        timer2_ = this->create_wall_timer(500ms, [this]() {
            RCLCPP_INFO(this->get_logger(), "[Exclusive2] Start");
            std::this_thread::sleep_for(300ms);
            RCLCPP_INFO(this->get_logger(), "[Exclusive2] Done");
        }, exclusive_group);

        // Timers in reentrant group
        timer3_ = this->create_wall_timer(500ms, [this]() {
            RCLCPP_INFO(this->get_logger(), "[Reentrant1] Start");
            std::this_thread::sleep_for(300ms);
            RCLCPP_INFO(this->get_logger(), "[Reentrant1] Done");
        }, reentrant_group);

        timer4_ = this->create_wall_timer(500ms, [this]() {
            RCLCPP_INFO(this->get_logger(), "[Reentrant2] Start");
            std::this_thread::sleep_for(300ms);
            RCLCPP_INFO(this->get_logger(), "[Reentrant2] Done");
        }, reentrant_group);

        RCLCPP_INFO(this->get_logger(), "Callback groups node started");
        RCLCPP_INFO(this->get_logger(), "Exclusive callbacks execute sequentially");
        RCLCPP_INFO(this->get_logger(), "Reentrant callbacks can execute concurrently");
    }

private:
    rclcpp::TimerBase::SharedPtr timer1_, timer2_, timer3_, timer4_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CallbackGroupsNode>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
