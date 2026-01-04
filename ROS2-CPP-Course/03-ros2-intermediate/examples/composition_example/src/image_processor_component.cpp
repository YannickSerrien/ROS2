/**
 * @file image_processor_component.cpp
 * @brief Image processing component with zero-copy input
 *
 * Demonstrates:
 * - Receiving large messages via zero-copy
 * - Processing without extra copies
 * - Publishing processed result
 * - Performance benefits of composition
 *
 * Used in: Module 3 - Composition (Performance Example)
 */

#include <memory>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace composition_example
{

class ImageProcessorComponent : public rclcpp::Node
{
public:
    explicit ImageProcessorComponent(const rclcpp::NodeOptions & options)
    : Node("image_processor", options),
      frame_count_(0)
    {
        // Create subscription
        // In composed system: receives SAME pointer as producer (zero-copy!)
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw",
            10,
            std::bind(&ImageProcessorComponent::image_callback, this, std::placeholders::_1)
        );

        // Create publisher for processed image
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_processed", 10);

        RCLCPP_INFO(this->get_logger(), "Image Processor Component initialized");
        RCLCPP_INFO(this->get_logger(), "Zero-copy enabled in composed system");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        frame_count_++;

        // In a composed system, msg is the EXACT same pointer from producer!
        // No serialization overhead, no memory copying

        // Create output image (still need to allocate for processed result)
        auto processed = std::make_unique<sensor_msgs::msg::Image>(*msg);

        // Simple processing: invert colors
        for (size_t i = 0; i < processed->data.size(); ++i) {
            processed->data[i] = 255 - processed->data[i];
        }

        // Update header
        processed->header.stamp = this->now();

        // Publish processed image
        publisher_->publish(std::move(processed));

        // Log periodically
        if (frame_count_ % 30 == 0) {
            RCLCPP_INFO(this->get_logger(),
                       "Processed frame %zu (size: %zu bytes)",
                       frame_count_,
                       msg->data.size());
        }
    }

    size_t frame_count_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

}  // namespace composition_example

RCLCPP_COMPONENTS_REGISTER_NODE(composition_example::ImageProcessorComponent)
