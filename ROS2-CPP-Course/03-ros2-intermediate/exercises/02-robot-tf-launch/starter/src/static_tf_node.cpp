/**
 * @file static_tf_node.cpp
 * @brief Static transform publisher for robot sensors - STARTER
 *
 * TODO: Implement static TF publisher
 *
 * Tasks:
 * 1. Declare parameters for sensor positions
 * 2. Create static TF broadcaster
 * 3. Build transform messages for each sensor
 * 4. Convert Euler angles to quaternions
 * 5. Publish all static transforms
 */

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class StaticTFNode : public rclcpp::Node
{
public:
    StaticTFNode()
    : Node("static_tf_node")
    {
        // TODO: Declare parameters for camera position
        // Hint: this->declare_parameter("camera_x", 0.3);

        // TODO: Declare parameters for lidar position

        // TODO: Declare parameters for IMU position

        // TODO: Create static broadcaster
        // Hint: tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // TODO: Publish static transforms
        publish_static_transforms();

        RCLCPP_INFO(this->get_logger(), "Static TF Node started");
    }

private:
    // TODO: Declare static broadcaster member variable
    // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    void publish_static_transforms()
    {
        // TODO: Get all parameters

        // TODO: Create vector of transforms

        // TODO: Build camera transform
        // Hint: Use geometry_msgs::msg::TransformStamped
        //       Set frame_id = "base_link", child_frame_id = "camera_link"
        //       Use tf2::Quaternion for rotation

        // TODO: Build lidar transform

        // TODO: Build IMU transform

        // TODO: Publish all transforms
        // Hint: tf_static_broadcaster_->sendTransform(transforms);

        RCLCPP_INFO(this->get_logger(), "Published static transforms");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
