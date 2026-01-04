/**
 * @file static_tf_publisher.cpp
 * @brief Publishes static transforms for fixed robot components
 *
 * Demonstrates:
 * - Creating static transform broadcaster
 * - Publishing static transforms (pub once, available forever)
 * - Quaternion construction from Euler angles
 * - Frame naming conventions
 *
 * Used in: Module 3 - TF2
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class StaticTFPublisher : public rclcpp::Node
{
public:
    StaticTFPublisher()
    : Node("static_tf_publisher")
    {
        // Create static broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms on startup
        publish_static_transforms();

        RCLCPP_INFO(this->get_logger(), "Static TF Publisher started");
        RCLCPP_INFO(this->get_logger(), "Published transforms:");
        RCLCPP_INFO(this->get_logger(), "  base_link -> camera_link");
        RCLCPP_INFO(this->get_logger(), "  base_link -> laser_link");
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    /**
     * Publish all static transforms
     * Static transforms don't change, so publish once
     */
    void publish_static_transforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        // Camera mounted on top of robot
        geometry_msgs::msg::TransformStamped camera_transform;
        camera_transform.header.stamp = this->now();
        camera_transform.header.frame_id = "base_link";
        camera_transform.child_frame_id = "camera_link";

        // Position: 0.3m forward, 0m sideways, 0.5m up
        camera_transform.transform.translation.x = 0.3;
        camera_transform.transform.translation.y = 0.0;
        camera_transform.transform.translation.z = 0.5;

        // Orientation: pitched down 30 degrees
        tf2::Quaternion camera_quat;
        camera_quat.setRPY(0.0, -0.524, 0.0);  // 0° roll, -30° pitch, 0° yaw
        camera_transform.transform.rotation.x = camera_quat.x();
        camera_transform.transform.rotation.y = camera_quat.y();
        camera_transform.transform.rotation.z = camera_quat.z();
        camera_transform.transform.rotation.w = camera_quat.w();

        transforms.push_back(camera_transform);

        // Laser scanner mounted at front
        geometry_msgs::msg::TransformStamped laser_transform;
        laser_transform.header.stamp = this->now();
        laser_transform.header.frame_id = "base_link";
        laser_transform.child_frame_id = "laser_link";

        // Position: 0.2m forward, 0m sideways, 0.1m up
        laser_transform.transform.translation.x = 0.2;
        laser_transform.transform.translation.y = 0.0;
        laser_transform.transform.translation.z = 0.1;

        // Orientation: no rotation (identity quaternion)
        laser_transform.transform.rotation.x = 0.0;
        laser_transform.transform.rotation.y = 0.0;
        laser_transform.transform.rotation.z = 0.0;
        laser_transform.transform.rotation.w = 1.0;

        transforms.push_back(laser_transform);

        // Publish all static transforms at once
        tf_static_broadcaster_->sendTransform(transforms);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTFPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
