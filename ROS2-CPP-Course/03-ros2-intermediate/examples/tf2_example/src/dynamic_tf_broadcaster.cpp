/**
 * @file dynamic_tf_broadcaster.cpp
 * @brief Publishes dynamic transforms that change over time
 *
 * Demonstrates:
 * - Creating dynamic transform broadcaster
 * - Publishing transforms at regular intervals
 * - Simulating moving robot (odom -> base_link)
 * - Time stamping transforms correctly
 *
 * Used in: Module 3 - TF2
 */

#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class DynamicTFBroadcaster : public rclcpp::Node
{
public:
    DynamicTFBroadcaster()
    : Node("dynamic_tf_broadcaster"),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      vx_(0.5),      // Linear velocity: 0.5 m/s
      vtheta_(0.3)   // Angular velocity: 0.3 rad/s
    {
        // Create dynamic broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Timer to publish transforms at 50 Hz
        timer_ = this->create_wall_timer(
            20ms,
            std::bind(&DynamicTFBroadcaster::publish_transform, this)
        );

        RCLCPP_INFO(this->get_logger(), "Dynamic TF Broadcaster started");
        RCLCPP_INFO(this->get_logger(), "Publishing odom -> base_link at 50 Hz");
    }

private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot state (simulated)
    double x_;
    double y_;
    double theta_;

    // Velocities
    double vx_;
    double vtheta_;

    /**
     * Publish transform from odom to base_link
     * Updates robot position based on velocity
     */
    void publish_transform()
    {
        // Update robot pose (simple motion model)
        double dt = 0.02;  // 20ms
        x_ += vx_ * std::cos(theta_) * dt;
        y_ += vx_ * std::sin(theta_) * dt;
        theta_ += vtheta_ * dt;

        // Normalize theta to [-pi, pi]
        while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

        // Create transform message
        geometry_msgs::msg::TransformStamped transform;

        // CRITICAL: Always use current time for dynamic transforms
        transform.header.stamp = this->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        // Set translation
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;

        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        // Broadcast transform
        tf_broadcaster_->sendTransform(transform);

        // Periodic logging (every 2 seconds)
        static int counter = 0;
        if (++counter % 100 == 0) {
            RCLCPP_INFO(this->get_logger(),
                       "Robot position: (%.2f, %.2f), orientation: %.2f rad",
                       x_, y_, theta_);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
