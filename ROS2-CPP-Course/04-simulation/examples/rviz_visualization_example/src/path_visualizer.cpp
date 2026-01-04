/**
 * @file path_visualizer.cpp
 * @brief Visualizes robot paths and trajectories in RViz
 *
 * Publishes a circular path as nav_msgs/Path and uses line markers
 * to show trajectory visualization techniques.
 *
 * Topics published:
 *   - /planned_path (nav_msgs/Path)
 *   - /trajectory_marker (visualization_msgs/Marker)
 *
 * Used in: Module 4, Lesson 4 (Visualization Markers)
 */

#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class PathVisualizer : public rclcpp::Node
{
public:
    PathVisualizer() : Node("path_visualizer"), time_(0.0)
    {
        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "trajectory_marker", 10);

        // Publish at 2 Hz
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&PathVisualizer::publish_visualizations, this));

        RCLCPP_INFO(this->get_logger(), "Path Visualizer started");
    }

private:
    void publish_visualizations()
    {
        // Update time for animated trajectory
        time_ += 0.5;

        // Publish path
        auto path = create_circular_path();
        path_pub_->publish(path);

        // Publish trajectory marker
        auto marker = create_trajectory_marker();
        marker_pub_->publish(marker);

        RCLCPP_INFO(this->get_logger(), "Published path and trajectory");
    }

    /**
     * Create circular path (nav_msgs/Path)
     */
    nav_msgs::msg::Path create_circular_path()
    {
        nav_msgs::msg::Path path;

        path.header.frame_id = "world";
        path.header.stamp = this->now();

        // Generate circular path
        double radius = 3.0;
        int num_points = 100;

        for (int i = 0; i < num_points; ++i) {
            double angle = 2.0 * M_PI * i / num_points;

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = this->now();

            pose.pose.position.x = radius * cos(angle);
            pose.pose.position.y = radius * sin(angle);
            pose.pose.position.z = 0.5;

            // Orientation pointing tangent to circle
            pose.pose.orientation.z = sin(angle / 2.0);
            pose.pose.orientation.w = cos(angle / 2.0);

            path.poses.push_back(pose);
        }

        return path;
    }

    /**
     * Create animated trajectory marker (line strip)
     */
    visualization_msgs::msg::Marker create_trajectory_marker()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = this->now();

        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.15;  // Line width

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 0.8;

        // Create spiral trajectory (animated)
        int num_points = 50;
        for (int i = 0; i < num_points; ++i) {
            double t = time_ + i * 0.1;
            double radius = 1.0 + i * 0.05;
            double angle = t + i * 0.2;

            geometry_msgs::msg::Point p;
            p.x = radius * cos(angle);
            p.y = radius * sin(angle);
            p.z = 1.5 + sin(t + i * 0.1) * 0.5;

            marker.points.push_back(p);

            // Color gradient (red to blue along path)
            std_msgs::msg::ColorRGBA color;
            double ratio = static_cast<double>(i) / num_points;
            color.r = 1.0 - ratio;
            color.g = 0.5;
            color.b = ratio;
            color.a = 0.8;

            marker.colors.push_back(color);
        }

        return marker;
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
