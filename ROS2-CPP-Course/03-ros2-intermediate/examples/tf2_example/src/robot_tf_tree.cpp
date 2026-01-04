/**
 * @file robot_tf_tree.cpp
 * @brief Complete robot transform tree demonstration
 *
 * Demonstrates:
 * - Publishing complete TF tree for mobile robot
 * - Mixing static and dynamic transforms
 * - Proper frame hierarchy (world -> odom -> base_link -> sensors)
 * - Realistic robot configuration
 *
 * Transform Tree:
 *   map
 *    └─ odom (dynamic)
 *        └─ base_link (dynamic)
 *            ├─ camera_link (static)
 *            ├─ laser_link (static)
 *            └─ imu_link (static)
 *
 * Used in: Module 3 - TF2 (Complete Example)
 */

#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class RobotTFTree : public rclcpp::Node
{
public:
    RobotTFTree()
    : Node("robot_tf_tree"),
      x_(0.0),
      y_(0.0),
      theta_(0.0)
    {
        // Create broadcasters
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms once
        publish_static_transforms();

        // Timer for dynamic transforms (50 Hz)
        timer_ = this->create_wall_timer(
            20ms,
            std::bind(&RobotTFTree::publish_dynamic_transforms, this)
        );

        RCLCPP_INFO(this->get_logger(), "Complete Robot TF Tree started");
        RCLCPP_INFO(this->get_logger(), "Transform tree:");
        RCLCPP_INFO(this->get_logger(), "  map -> odom (static)");
        RCLCPP_INFO(this->get_logger(), "  odom -> base_link (dynamic, 50 Hz)");
        RCLCPP_INFO(this->get_logger(), "  base_link -> sensors (static)");
    }

private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot state
    double x_, y_, theta_;

    /**
     * Publish all static transforms
     * These represent fixed robot components
     */
    void publish_static_transforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
        rclcpp::Time now = this->now();

        // 1. map -> odom (static for this demo; in real SLAM this would be dynamic)
        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = now;
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";
        map_to_odom.transform.translation.x = 0.0;
        map_to_odom.transform.translation.y = 0.0;
        map_to_odom.transform.translation.z = 0.0;
        map_to_odom.transform.rotation.w = 1.0;
        static_transforms.push_back(map_to_odom);

        // 2. base_link -> camera_link
        geometry_msgs::msg::TransformStamped base_to_camera;
        base_to_camera.header.stamp = now;
        base_to_camera.header.frame_id = "base_link";
        base_to_camera.child_frame_id = "camera_link";
        base_to_camera.transform.translation.x = 0.25;   // 25cm forward
        base_to_camera.transform.translation.y = 0.0;
        base_to_camera.transform.translation.z = 0.40;   // 40cm up
        tf2::Quaternion camera_quat;
        camera_quat.setRPY(0.0, -0.349, 0.0);  // Pitched down 20°
        base_to_camera.transform.rotation.x = camera_quat.x();
        base_to_camera.transform.rotation.y = camera_quat.y();
        base_to_camera.transform.rotation.z = camera_quat.z();
        base_to_camera.transform.rotation.w = camera_quat.w();
        static_transforms.push_back(base_to_camera);

        // 3. base_link -> laser_link
        geometry_msgs::msg::TransformStamped base_to_laser;
        base_to_laser.header.stamp = now;
        base_to_laser.header.frame_id = "base_link";
        base_to_laser.child_frame_id = "laser_link";
        base_to_laser.transform.translation.x = 0.20;    // 20cm forward
        base_to_laser.transform.translation.y = 0.0;
        base_to_laser.transform.translation.z = 0.15;    // 15cm up
        base_to_laser.transform.rotation.w = 1.0;        // No rotation
        static_transforms.push_back(base_to_laser);

        // 4. base_link -> imu_link
        geometry_msgs::msg::TransformStamped base_to_imu;
        base_to_imu.header.stamp = now;
        base_to_imu.header.frame_id = "base_link";
        base_to_imu.child_frame_id = "imu_link";
        base_to_imu.transform.translation.x = 0.0;       // Center of robot
        base_to_imu.transform.translation.y = 0.0;
        base_to_imu.transform.translation.z = 0.05;      // 5cm up
        base_to_imu.transform.rotation.w = 1.0;
        static_transforms.push_back(base_to_imu);

        // Publish all static transforms
        tf_static_broadcaster_->sendTransform(static_transforms);

        RCLCPP_INFO(this->get_logger(), "Published %zu static transforms",
                   static_transforms.size());
    }

    /**
     * Publish dynamic transforms
     * These change as robot moves
     */
    void publish_dynamic_transforms()
    {
        // Simulate robot movement (circular motion)
        static double time = 0.0;
        double dt = 0.02;  // 20ms
        time += dt;

        // Circular motion
        double radius = 2.0;
        double angular_velocity = 0.5;  // rad/s

        x_ = radius * std::cos(angular_velocity * time);
        y_ = radius * std::sin(angular_velocity * time);
        theta_ = angular_velocity * time + M_PI / 2;

        // Normalize theta
        while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

        // Create odom -> base_link transform
        geometry_msgs::msg::TransformStamped odom_to_base;
        odom_to_base.header.stamp = this->now();
        odom_to_base.header.frame_id = "odom";
        odom_to_base.child_frame_id = "base_link";

        odom_to_base.transform.translation.x = x_;
        odom_to_base.transform.translation.y = y_;
        odom_to_base.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom_to_base.transform.rotation.x = q.x();
        odom_to_base.transform.rotation.y = q.y();
        odom_to_base.transform.rotation.z = q.z();
        odom_to_base.transform.rotation.w = q.w();

        // Broadcast transform
        tf_broadcaster_->sendTransform(odom_to_base);

        // Log position periodically (every 2 seconds)
        static int counter = 0;
        if (++counter % 100 == 0) {
            RCLCPP_INFO(this->get_logger(),
                       "Robot: (%.2f, %.2f) θ=%.2f°",
                       x_, y_, theta_ * 180.0 / M_PI);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotTFTree>();

    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "To visualize the TF tree:");
    RCLCPP_INFO(node->get_logger(), "  ros2 run tf2_tools view_frames");
    RCLCPP_INFO(node->get_logger(), "  evince frames.pdf");
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "To monitor transforms:");
    RCLCPP_INFO(node->get_logger(), "  ros2 run tf2_ros tf2_echo map camera_link");
    RCLCPP_INFO(node->get_logger(), "");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
