/**
 * @file interactive_marker_server.cpp
 * @brief Interactive marker example - drag to set goals
 *
 * Creates interactive markers that can be dragged in RViz to set
 * robot goal positions.
 *
 * Topics:
 *   - Interactive marker server on /goal_markers
 *   - Publishes goals to /goal_pose
 *
 * Used in: Module 4, Lesson 4 (Visualization Markers)
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class InteractiveMarkerServer : public rclcpp::Node
{
public:
    InteractiveMarkerServer() : Node("interactive_marker_server")
    {
        // Create interactive marker server
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "goal_markers", shared_from_this());

        // Publisher for goal pose
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10);

        // Create interactive markers
        create_goal_marker();

        RCLCPP_INFO(this->get_logger(), "Interactive Marker Server started");
        RCLCPP_INFO(this->get_logger(), "Drag the green arrow in RViz to set goals");
    }

private:
    void create_goal_marker()
    {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "world";
        int_marker.name = "goal_marker";
        int_marker.description = "Drag to set goal";
        int_marker.pose.position.x = 2.0;
        int_marker.pose.position.y = 0.0;
        int_marker.pose.position.z = 0.5;
        int_marker.pose.orientation.w = 1.0;

        // Visual marker (arrow)
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.scale.x = 1.5;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Create control for visual
        visualization_msgs::msg::InteractiveMarkerControl marker_control;
        marker_control.always_visible = true;
        marker_control.markers.push_back(marker);
        int_marker.controls.push_back(marker_control);

        // Add 6-DOF control (move in XY plane)
        visualization_msgs::msg::InteractiveMarkerControl control;

        // Move in XY plane
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "move_xy";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
        int_marker.controls.push_back(control);

        // Rotate around Z
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);

        // Insert marker into server
        server_->insert(int_marker);
        server_->setCallback(
            int_marker.name,
            std::bind(&InteractiveMarkerServer::process_feedback, this,
                     std::placeholders::_1));

        server_->applyChanges();

        RCLCPP_INFO(this->get_logger(), "Created interactive goal marker");
    }

    void process_feedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
    {
        // Only process on mouse button release (after drag)
        if (feedback->event_type ==
            visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
        {
            // Publish new goal pose
            geometry_msgs::msg::PoseStamped goal;
            goal.header.frame_id = "world";
            goal.header.stamp = this->now();
            goal.pose = feedback->pose;

            goal_pub_->publish(goal);

            RCLCPP_INFO(this->get_logger(),
                       "New goal: (%.2f, %.2f, %.2f)",
                       feedback->pose.position.x,
                       feedback->pose.position.y,
                       feedback->pose.position.z);
        }
    }

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InteractiveMarkerServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
