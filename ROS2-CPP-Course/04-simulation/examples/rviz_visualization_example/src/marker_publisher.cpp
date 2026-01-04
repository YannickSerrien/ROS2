/**
 * @file marker_publisher.cpp
 * @brief Demonstrates various RViz marker types
 *
 * Publishes different marker types (arrow, cube, sphere, cylinder, line strip, text)
 * to visualize how markers work in RViz.
 *
 * Topics published:
 *   - /visualization_marker_array (visualization_msgs/MarkerArray)
 *
 * Used in: Module 4, Lesson 4 (Visualization Markers)
 *
 * To view:
 *   1. ros2 run rviz_visualization_example marker_publisher
 *   2. rviz2
 *   3. Add -> MarkerArray -> Topic: /visualization_marker_array
 *   4. Set Fixed Frame to "world"
 */

#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class MarkerPublisher : public rclcpp::Node
{
public:
    MarkerPublisher() : Node("marker_publisher"), counter_(0)
    {
        // Publisher for marker array
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);

        // Publish markers at 1 Hz
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&MarkerPublisher::publish_markers, this));

        RCLCPP_INFO(this->get_logger(), "Marker Publisher started");
        RCLCPP_INFO(this->get_logger(), "Publishing markers on /visualization_marker_array");
        RCLCPP_INFO(this->get_logger(), "View in RViz with Fixed Frame: world");
    }

private:
    /**
     * Publish all marker examples
     */
    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Create various marker types
        marker_array.markers.push_back(create_arrow_marker());
        marker_array.markers.push_back(create_cube_marker());
        marker_array.markers.push_back(create_sphere_marker());
        marker_array.markers.push_back(create_cylinder_marker());
        marker_array.markers.push_back(create_line_strip_marker());
        marker_array.markers.push_back(create_text_marker());

        // Publish all markers
        marker_array_pub_->publish(marker_array);

        counter_++;
        RCLCPP_INFO(this->get_logger(), "Published marker array (count: %d)", counter_);
    }

    /**
     * Create arrow marker (pointing in X direction)
     */
    visualization_msgs::msg::Marker create_arrow_marker()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = this->now();

        marker.ns = "arrows";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 1.0;
        marker.pose.orientation.w = 1.0;

        // Size (length, shaft diameter, head diameter)
        marker.scale.x = 1.5;  // Length
        marker.scale.y = 0.2;  // Shaft width
        marker.scale.z = 0.2;  // Head width

        // Color (red)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persist

        return marker;
    }

    /**
     * Create cube marker
     */
    visualization_msgs::msg::Marker create_cube_marker()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = this->now();

        marker.ns = "cubes";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position
        marker.pose.position.x = 2.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.5;

        // Rotation (45 degrees around Z axis)
        double yaw = M_PI / 4.0;
        marker.pose.orientation.z = sin(yaw / 2.0);
        marker.pose.orientation.w = cos(yaw / 2.0);

        // Size
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Color (green)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);

        return marker;
    }

    /**
     * Create sphere marker
     */
    visualization_msgs::msg::Marker create_sphere_marker()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = this->now();

        marker.ns = "spheres";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position
        marker.pose.position.x = 4.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.5;
        marker.pose.orientation.w = 1.0;

        // Size (diameter)
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Color (blue)
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);

        return marker;
    }

    /**
     * Create cylinder marker
     */
    visualization_msgs::msg::Marker create_cylinder_marker()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = this->now();

        marker.ns = "cylinders";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position
        marker.pose.position.x = 6.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.75;
        marker.pose.orientation.w = 1.0;

        // Size (diameter, diameter, height)
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 1.5;

        // Color (yellow)
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);

        return marker;
    }

    /**
     * Create line strip marker (zigzag line)
     */
    visualization_msgs::msg::Marker create_line_strip_marker()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = this->now();

        marker.ns = "lines";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;

        // Line width
        marker.scale.x = 0.1;

        // Color (magenta)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        // Create zigzag pattern
        for (int i = 0; i < 5; ++i) {
            geometry_msgs::msg::Point p;
            p.x = 0.0 + i * 0.5;
            p.y = 2.0 + (i % 2 == 0 ? 0.5 : -0.5);
            p.z = 0.5;
            marker.points.push_back(p);
        }

        marker.lifetime = rclcpp::Duration::from_seconds(0);

        return marker;
    }

    /**
     * Create text marker
     */
    visualization_msgs::msg::Marker create_text_marker()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "world";
        marker.header.stamp = this->now();

        marker.ns = "texts";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position
        marker.pose.position.x = 0.0;
        marker.pose.position.y = -2.0;
        marker.pose.position.z = 2.0;
        marker.pose.orientation.w = 1.0;

        // Text content
        marker.text = "RViz Marker Examples!";

        // Text height
        marker.scale.z = 0.5;

        // Color (white)
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);

        return marker;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
