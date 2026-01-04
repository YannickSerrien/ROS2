/**
 * @file arm_controller.cpp
 * @brief Robot arm controller with TF2 broadcasting - STARTER
 *
 * TODO: Complete the arm controller implementation
 *
 * Your tasks:
 * 1. Declare parameters for arm configuration
 * 2. Initialize joint states
 * 3. Create TF broadcaster
 * 4. Implement forward kinematics
 * 5. Publish TF tree at regular intervals
 * 6. Publish joint states
 */

#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "robot_arm_project/kinematics.hpp"

using namespace std::chrono_literals;
using robot_arm_kinematics::ArmConfig;
using robot_arm_kinematics::forward_kinematics;

class ArmController : public rclcpp::Node
{
public:
    ArmController()
    : Node("arm_controller")
    {
        // TODO: Declare parameters
        // Hints:
        // - link1_length, link2_length (arm dimensions)
        // - joint1_min, joint1_max, joint2_min, joint2_max (limits)
        // - initial_joint1, initial_joint2 (starting position)
        // - publish_rate (TF update frequency)

        // YOUR CODE HERE


        // TODO: Get parameters and store in arm_config_

        // YOUR CODE HERE


        // TODO: Initialize joint states from parameters

        // YOUR CODE HERE


        // TODO: Create TF broadcaster
        // Hint: tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // YOUR CODE HERE


        // TODO: Create joint state publisher
        // Hint: Publisher for sensor_msgs::msg::JointState on "/joint_states"

        // YOUR CODE HERE


        // TODO: Create timer for TF updates
        // Hint: Use publish_rate parameter to calculate period
        //       timer_ = this->create_wall_timer(period, callback);

        // YOUR CODE HERE


        RCLCPP_INFO(this->get_logger(), "Arm Controller started");
    }

private:
    // TODO: Declare member variables
    // Hints:
    // - ArmConfig arm_config_;
    // - double joint1_angle_, joint2_angle_;
    // - std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // - rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    // - rclcpp::TimerBase::SharedPtr timer_;


    /**
     * TODO: Publish TF tree and joint states
     *
     * Tasks:
     * 1. Create transform for base_link → link1
     * 2. Create transform for link1 → link2
     * 3. Create transform for link2 → end_effector
     * 4. Broadcast all transforms
     * 5. Publish joint states
     *
     * Hints:
     * - Use forward_kinematics() to get positions
     * - Convert angles to quaternions with tf2::Quaternion
     * - Set proper frame_ids and child_frame_ids
     * - Timestamp all messages with this->now()
     */
    void publish_transforms_and_states()
    {
        // TODO: Create transforms vector
        // std::vector<geometry_msgs::msg::TransformStamped> transforms;

        // TODO: Transform 1: world → base_link (static)
        // Hint: Identity transform at origin

        // YOUR CODE HERE


        // TODO: Transform 2: base_link → link1
        // Hint: Rotates by joint1_angle_
        //       Use tf2::Quaternion q; q.setRPY(0, 0, joint1_angle_);

        // YOUR CODE HERE


        // TODO: Transform 3: link1 → link2
        // Hint: Translation = link1_length in x direction
        //       Rotation = joint2_angle_ around z-axis
        //       Position relative to link1 end

        // YOUR CODE HERE


        // TODO: Transform 4: link2 → end_effector
        // Hint: Translation = link2_length in x direction
        //       No rotation

        // YOUR CODE HERE


        // TODO: Broadcast all transforms
        // Hint: tf_broadcaster_->sendTransform(transforms);

        // YOUR CODE HERE


        // TODO: Publish joint states
        // Hint: Create sensor_msgs::msg::JointState message
        //       Fill names, positions arrays
        //       Publish on joint_state_pub_

        // YOUR CODE HERE


        // Optional: Log periodically
        static int count = 0;
        if (++count % 50 == 0) {  // Every ~1 second at 50 Hz
            auto [x, y] = forward_kinematics(joint1_angle_, joint2_angle_, arm_config_);
            RCLCPP_INFO(this->get_logger(),
                       "Joints: [%.2f, %.2f] rad | End-effector: (%.3f, %.3f)",
                       joint1_angle_, joint2_angle_, x, y);
        }
    }

    /**
     * TODO: Set joint positions (for future joint command subscriber)
     *
     * This will be called when receiving joint commands
     * For now, you can test by manually changing joint angles
     */
    void set_joint_positions(double joint1, double joint2)
    {
        // TODO: Validate against joint limits
        // TODO: Update joint1_angle_, joint2_angle_
        // YOUR CODE HERE
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
