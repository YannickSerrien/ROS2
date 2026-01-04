/**
 * @file tf_listener.cpp
 * @brief Listens to transforms and performs coordinate transformations
 *
 * Demonstrates:
 * - Creating TF buffer and listener
 * - Looking up transforms between frames
 * - Transforming points between coordinate frames
 * - Exception handling for TF queries
 * - Waiting for transforms to become available
 *
 * Used in: Module 3 - TF2
 */

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node
{
public:
    TFListener()
    : Node("tf_listener")
    {
        // Create TF buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer to periodically query transforms
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&TFListener::query_transforms, this)
        );

        RCLCPP_INFO(this->get_logger(), "TF Listener started");
        RCLCPP_INFO(this->get_logger(), "Querying transforms every 1 second");
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * Query and display transforms
     */
    void query_transforms()
    {
        // Example 1: Get transform between two frames
        lookup_transform_example();

        // Example 2: Transform a point from one frame to another
        transform_point_example();

        // Example 3: Check if transform exists
        check_transform_availability();
    }

    /**
     * Example 1: Look up transform between frames
     */
    void lookup_transform_example()
    {
        try {
            // Get latest transform from odom to camera_link
            auto transform = tf_buffer_->lookupTransform(
                "odom",              // Target frame
                "camera_link",       // Source frame
                tf2::TimePointZero   // Latest available
            );

            RCLCPP_INFO(this->get_logger(), "Transform odom -> camera_link:");
            RCLCPP_INFO(this->get_logger(), "  Translation: (%.2f, %.2f, %.2f)",
                       transform.transform.translation.x,
                       transform.transform.translation.y,
                       transform.transform.translation.z);
            RCLCPP_INFO(this->get_logger(), "  Rotation: (%.2f, %.2f, %.2f, %.2f)",
                       transform.transform.rotation.x,
                       transform.transform.rotation.y,
                       transform.transform.rotation.z,
                       transform.transform.rotation.w);

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                       "Could not transform odom to camera_link: %s", ex.what());
        }
    }

    /**
     * Example 2: Transform a point from camera frame to odom frame
     */
    void transform_point_example()
    {
        try {
            // Create a point in camera frame
            geometry_msgs::msg::PointStamped point_in_camera;
            point_in_camera.header.frame_id = "camera_link";
            point_in_camera.header.stamp = this->now();
            point_in_camera.point.x = 5.0;  // 5m in front of camera
            point_in_camera.point.y = 0.0;
            point_in_camera.point.z = 0.0;

            // Transform point to odom frame
            geometry_msgs::msg::PointStamped point_in_odom;
            point_in_odom = tf_buffer_->transform(point_in_camera, "odom");

            RCLCPP_INFO(this->get_logger(), "Point transformation:");
            RCLCPP_INFO(this->get_logger(), "  In camera_link: (%.2f, %.2f, %.2f)",
                       point_in_camera.point.x,
                       point_in_camera.point.y,
                       point_in_camera.point.z);
            RCLCPP_INFO(this->get_logger(), "  In odom: (%.2f, %.2f, %.2f)",
                       point_in_odom.point.x,
                       point_in_odom.point.y,
                       point_in_odom.point.z);

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                       "Could not transform point: %s", ex.what());
        }
    }

    /**
     * Example 3: Check if transform is available
     */
    void check_transform_availability()
    {
        // Check if specific frame exists
        bool can_transform = tf_buffer_->canTransform(
            "odom",
            "base_link",
            tf2::TimePointZero,
            tf2::durationFromSec(0.5)  // Timeout: 500ms
        );

        if (can_transform) {
            RCLCPP_INFO(this->get_logger(), "Transform odom -> base_link is available");
        } else {
            RCLCPP_WARN(this->get_logger(), "Transform odom -> base_link NOT available");
        }

        // List all available frames
        std::string all_frames = tf_buffer_->allFramesAsString();
        RCLCPP_INFO(this->get_logger(), "Available frames:\n%s", all_frames.c_str());
    }

    /**
     * Wait for a specific transform to become available
     */
    bool wait_for_transform(const std::string & target_frame,
                           const std::string & source_frame,
                           double timeout_sec = 5.0)
    {
        try {
            auto transform = tf_buffer_->lookupTransform(
                target_frame,
                source_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(timeout_sec)
            );
            return true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(),
                        "Timeout waiting for transform %s -> %s: %s",
                        target_frame.c_str(), source_frame.c_str(), ex.what());
            return false;
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
