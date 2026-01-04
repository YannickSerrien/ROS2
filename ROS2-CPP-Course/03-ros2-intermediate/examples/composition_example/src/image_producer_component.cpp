/**
 * @file image_producer_component.cpp
 * @brief Simulated camera component producing image data
 *
 * Demonstrates:
 * - Publishing large messages efficiently
 * - Zero-copy with unique_ptr for performance
 * - Realistic robotics scenario (camera node)
 * - Configurable frame rate
 *
 * Used in: Module 3 - Composition (Performance Example)
 */

#include <memory>
#include <chrono>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

namespace composition_example
{

class ImageProducerComponent : public rclcpp::Node
{
public:
    explicit ImageProducerComponent(const rclcpp::NodeOptions & options)
    : Node("image_producer", options),
      frame_count_(0)
    {
        // Declare parameters
        this->declare_parameter("width", 640);
        this->declare_parameter("height", 480);
        this->declare_parameter("frame_rate", 30.0);

        // Get parameters
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        double frame_rate = this->get_parameter("frame_rate").as_double();

        // Calculate image size
        image_size_ = width_ * height_ * 3;  // RGB image

        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

        // Create timer based on frame rate
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / frame_rate));
        timer_ = this->create_wall_timer(
            period,
            std::bind(&ImageProducerComponent::publish_image, this)
        );

        RCLCPP_INFO(this->get_logger(), "Image Producer Component initialized");
        RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d @ %.1f Hz",
                   width_, height_, frame_rate);
        RCLCPP_INFO(this->get_logger(), "Image size: %zu bytes", image_size_);
    }

private:
    void publish_image()
    {
        // Use unique_ptr for zero-copy publishing
        auto image = std::make_unique<sensor_msgs::msg::Image>();

        // Fill header
        image->header.stamp = this->now();
        image->header.frame_id = "camera_link";

        // Image metadata
        image->width = width_;
        image->height = height_;
        image->encoding = "rgb8";
        image->is_bigendian = false;
        image->step = width_ * 3;

        // Allocate and fill image data (simulated)
        image->data.resize(image_size_);

        // Generate simple pattern (gradients based on frame count)
        for (size_t y = 0; y < height_; ++y) {
            for (size_t x = 0; x < width_; ++x) {
                size_t idx = (y * width_ + x) * 3;
                image->data[idx + 0] = (x + frame_count_) % 256;      // R
                image->data[idx + 1] = (y + frame_count_) % 256;      // G
                image->data[idx + 2] = ((x + y) + frame_count_) % 256;  // B
            }
        }

        frame_count_++;

        // Publish with zero-copy (in composed system)
        publisher_->publish(std::move(image));

        // Log periodically (every 30 frames)
        if (frame_count_ % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "Published frame %zu", frame_count_);
        }
    }

    int width_;
    int height_;
    size_t image_size_;
    size_t frame_count_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace composition_example

RCLCPP_COMPONENTS_REGISTER_NODE(composition_example::ImageProducerComponent)
