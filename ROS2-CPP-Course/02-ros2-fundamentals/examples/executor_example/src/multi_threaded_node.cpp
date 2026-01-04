/**
 * @file multi_threaded_node.cpp
 * @brief Demonstration of multi-threaded executor
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MultiThreadedNode : public rclcpp::Node
{
public:
    MultiThreadedNode() : Node("multi_threaded")
    {
        timer1_ = this->create_wall_timer(500ms, [this]() {
            RCLCPP_INFO(this->get_logger(), "[Timer1] Start (thread: %s)",
                       std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())).c_str());
            std::this_thread::sleep_for(200ms);
            RCLCPP_INFO(this->get_logger(), "[Timer1] Done");
        });

        timer2_ = this->create_wall_timer(500ms, [this]() {
            RCLCPP_INFO(this->get_logger(), "[Timer2] Start (thread: %s)",
                       std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())).c_str());
            std::this_thread::sleep_for(200ms);
            RCLCPP_INFO(this->get_logger(), "[Timer2] Done");
        });

        RCLCPP_INFO(this->get_logger(), "Multi-threaded node started");
        RCLCPP_INFO(this->get_logger(), "Callbacks can execute concurrently");
    }

private:
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiThreadedNode>();

    // Multi-threaded executor with 4 threads
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
