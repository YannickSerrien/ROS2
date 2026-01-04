/**
 * @file single_threaded_node.cpp
 * @brief Demonstration of single-threaded executor (default)
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SingleThreadedNode : public rclcpp::Node
{
public:
    SingleThreadedNode() : Node("single_threaded")
    {
        timer1_ = this->create_wall_timer(500ms, [this]() {
            RCLCPP_INFO(this->get_logger(), "[Timer1] Processing (500ms)");
            std::this_thread::sleep_for(200ms);  // Simulate work
        });

        timer2_ = this->create_wall_timer(1s, [this]() {
            RCLCPP_INFO(this->get_logger(), "[Timer2] Processing (1s)");
            std::this_thread::sleep_for(300ms);  // Simulate work
        });

        RCLCPP_INFO(this->get_logger(), "Single-threaded node started");
        RCLCPP_INFO(this->get_logger(), "Callbacks execute sequentially in one thread");
    }

private:
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SingleThreadedNode>();

    // Default single-threaded executor
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
