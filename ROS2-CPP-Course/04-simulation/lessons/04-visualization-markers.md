# Lesson 4: Visualization Markers

## Learning Objectives

- Create visualization markers in RViz
- Use different marker types (arrows, spheres, cubes, lines)
- Build marker arrays for multiple visualizations
- Create interactive markers
- Visualize paths and trajectories
- Debug robot behavior with visual overlays
- Understand marker lifetimes and namespaces

## What are Visualization Markers?

**Visualization Markers**: Custom 3D shapes published by your nodes to display debug information, goals, paths, or any visual data in RViz.

**Purpose**: Visualize abstract data that isn't part of the robot's physical structure.

**Use cases**:
- Show goal positions
- Display planned paths
- Visualize sensor field of view
- Debug coordinate calculations
- Show object detections
- Annotate environment features

**Not to be confused with**:
- **RobotModel**: Shows robot URDF structure
- **TF**: Shows coordinate frames
- **Sensor displays**: Show actual sensor data

## Marker Basics

### Marker Message Structure

```cpp
#include "visualization_msgs/msg/marker.hpp"

visualization_msgs::msg::Marker marker;

// Header
marker.header.frame_id = "map";
marker.header.stamp = this->now();

// Namespace and ID (unique identifier)
marker.ns = "my_namespace";
marker.id = 0;

// Type of marker
marker.type = visualization_msgs::msg::Marker::ARROW;

// Action (ADD, DELETE, DELETEALL)
marker.action = visualization_msgs::msg::Marker::ADD;

// Pose (position and orientation)
marker.pose.position.x = 1.0;
marker.pose.position.y = 2.0;
marker.pose.position.z = 0.0;
marker.pose.orientation.w = 1.0;  // Identity quaternion

// Scale
marker.scale.x = 1.0;  // Length for arrow
marker.scale.y = 0.1;  // Width
marker.scale.z = 0.1;  // Height

// Color (RGBA, 0-1 range)
marker.color.r = 1.0;
marker.color.g = 0.0;
marker.color.b = 0.0;
marker.color.a = 1.0;  // Alpha (transparency)

// Lifetime (0 = forever)
marker.lifetime = rclcpp::Duration::from_seconds(0);
```

### Publishing Markers

```cpp
class MarkerPublisher : public rclcpp::Node
{
public:
    MarkerPublisher() : Node("marker_publisher")
    {
        // Publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "visualization_marker", 10);

        // Timer to publish periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MarkerPublisher::publish_marker, this));
    }

private:
    void publish_marker()
    {
        auto marker = create_arrow_marker();
        marker_pub_->publish(marker);
    }

    visualization_msgs::msg::Marker create_arrow_marker()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = this->now();

        marker.ns = "arrows";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = 1.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.5;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.5;  // Length
        marker.scale.y = 0.2;  // Width
        marker.scale.z = 0.2;  // Height

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        return marker;
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

### Viewing in RViz

```bash
# Terminal 1: Run your marker publisher
ros2 run my_package marker_publisher

# Terminal 2: RViz
rviz2

# In RViz:
# 1. Set Fixed Frame: map
# 2. Add → Marker
# 3. Topic: /visualization_marker
```

## Marker Types

### 1. Basic Shapes

**ARROW** - Directional indicator
```cpp
marker.type = visualization_msgs::msg::Marker::ARROW;
marker.scale.x = 1.0;  // Length
marker.scale.y = 0.1;  // Shaft diameter
marker.scale.z = 0.1;  // Head diameter
```

**CUBE** - Box shape
```cpp
marker.type = visualization_msgs::msg::Marker::CUBE;
marker.scale.x = 0.5;  // Width (X)
marker.scale.y = 0.5;  // Depth (Y)
marker.scale.z = 0.5;  // Height (Z)
```

**SPHERE** - Ball shape
```cpp
marker.type = visualization_msgs::msg::Marker::SPHERE;
marker.scale.x = 0.3;  // Diameter (all dimensions same)
marker.scale.y = 0.3;
marker.scale.z = 0.3;
```

**CYLINDER** - Pipe shape
```cpp
marker.type = visualization_msgs::msg::Marker::CYLINDER;
marker.scale.x = 0.2;  // Diameter (X, Y same)
marker.scale.y = 0.2;
marker.scale.z = 1.0;  // Height (Z-axis)
```

### 2. Line Types

**LINE_STRIP** - Connected line segments
```cpp
marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
marker.scale.x = 0.05;  // Line width (only X used)

// Define points
geometry_msgs::msg::Point p1, p2, p3;
p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
p2.x = 1.0; p2.y = 1.0; p2.z = 0.0;
p3.x = 2.0; p3.y = 0.0; p3.z = 0.0;

marker.points.push_back(p1);
marker.points.push_back(p2);
marker.points.push_back(p3);

// Each point can have different color (optional)
std_msgs::msg::ColorRGBA c1, c2, c3;
c1.r = 1.0; c1.a = 1.0;  // Red
c2.g = 1.0; c2.a = 1.0;  // Green
c3.b = 1.0; c3.a = 1.0;  // Blue

marker.colors.push_back(c1);
marker.colors.push_back(c2);
marker.colors.push_back(c3);
```

**LINE_LIST** - Separate line segments (pairs of points)
```cpp
marker.type = visualization_msgs::msg::Marker::LINE_LIST;
marker.scale.x = 0.05;

// Each pair of points creates a separate line
marker.points.push_back(p1);  // Line 1 start
marker.points.push_back(p2);  // Line 1 end
marker.points.push_back(p3);  // Line 2 start
marker.points.push_back(p4);  // Line 2 end
```

### 3. Text

**TEXT_VIEW_FACING** - Always faces camera
```cpp
marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
marker.text = "Goal Position";

marker.pose.position.x = 2.0;
marker.pose.position.y = 1.0;
marker.pose.position.z = 1.5;  // Height above ground

marker.scale.z = 0.3;  // Text height (only Z used)

marker.color.r = 1.0;
marker.color.g = 1.0;
marker.color.b = 1.0;
marker.color.a = 1.0;
```

### 4. Mesh

**MESH_RESOURCE** - Load 3D model file
```cpp
marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
marker.mesh_resource = "package://my_robot/meshes/object.stl";

marker.scale.x = 1.0;  // Scale factor
marker.scale.y = 1.0;
marker.scale.z = 1.0;
```

## Marker Arrays

### Publishing Multiple Markers Efficiently

**MarkerArray**: Publish multiple markers in single message

```cpp
#include "visualization_msgs/msg/marker_array.hpp"

class PathVisualizer : public rclcpp::Node
{
public:
    PathVisualizer() : Node("path_visualizer")
    {
        marker_array_pub_ = this->create_publisher<
            visualization_msgs::msg::MarkerArray>(
            "path_markers", 10);

        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&PathVisualizer::publish_path, this));
    }

private:
    void publish_path()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Create waypoint markers
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            auto marker = create_waypoint_marker(i, waypoints_[i]);
            marker_array.markers.push_back(marker);
        }

        // Create path line
        auto path_line = create_path_line();
        marker_array.markers.push_back(path_line);

        marker_array_pub_->publish(marker_array);
    }

    visualization_msgs::msg::Marker create_waypoint_marker(
        int id, const geometry_msgs::msg::Point & point)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = this->now();

        marker.ns = "waypoints";
        marker.id = id;  // Unique ID for each waypoint
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = point;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        return marker;
    }

    visualization_msgs::msg::Marker create_path_line()
    {
        visualization_msgs::msg::Marker line;

        line.header.frame_id = "map";
        line.header.stamp = this->now();

        line.ns = "path";
        line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;

        line.scale.x = 0.05;  // Line width

        line.color.r = 1.0;
        line.color.g = 1.0;
        line.color.b = 0.0;
        line.color.a = 0.8;

        // Add all waypoints to line
        line.points = waypoints_;

        return line;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_array_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Point> waypoints_ = {
        create_point(0, 0, 0),
        create_point(1, 1, 0),
        create_point(2, 1, 0),
        create_point(3, 0, 0)
    };

    geometry_msgs::msg::Point create_point(double x, double y, double z)
    {
        geometry_msgs::msg::Point p;
        p.x = x; p.y = y; p.z = z;
        return p;
    }
};
```

## Marker Lifetimes and Deletion

### Temporary Markers

**Lifetime**: Marker automatically disappears after duration

```cpp
// Marker disappears after 5 seconds
marker.lifetime = rclcpp::Duration::from_seconds(5.0);

// Marker stays forever (until explicitly deleted)
marker.lifetime = rclcpp::Duration::from_seconds(0);
```

### Deleting Markers

**Delete specific marker**:
```cpp
marker.action = visualization_msgs::msg::Marker::DELETE;
marker.ns = "waypoints";
marker.id = 5;  // Delete waypoint 5
```

**Delete all markers in namespace**:
```cpp
marker.action = visualization_msgs::msg::Marker::DELETEALL;
marker.ns = "waypoints";  // Delete all in "waypoints" namespace
```

**Delete all markers**:
```cpp
visualization_msgs::msg::Marker delete_all;
delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
marker_pub_->publish(delete_all);
```

## Practical Examples

### Example 1: Robot Goal Visualizer

```cpp
/**
 * @file goal_visualizer.cpp
 * @brief Visualize navigation goals with arrows
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class GoalVisualizer : public rclcpp::Node
{
public:
    GoalVisualizer() : Node("goal_visualizer")
    {
        // Subscribe to goal topic
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&GoalVisualizer::goal_callback, this,
                     std::placeholders::_1));

        // Publisher for marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "goal_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Goal visualizer started");
    }

private:
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Create arrow marker at goal position
        visualization_msgs::msg::Marker marker;

        marker.header = msg->header;

        marker.ns = "goal";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = msg->pose;

        marker.scale.x = 1.0;  // Length
        marker.scale.y = 0.2;  // Width
        marker.scale.z = 0.2;  // Height

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persist

        marker_pub_->publish(marker);

        RCLCPP_INFO(this->get_logger(),
                   "Goal marker published at (%.2f, %.2f)",
                   msg->pose.position.x, msg->pose.position.y);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalVisualizer>());
    rclcpp::shutdown();
    return 0;
}
```

### Example 2: Sensor Field of View

```cpp
/**
 * @file sensor_fov_marker.cpp
 * @brief Visualize sensor field of view as a cone
 */

class SensorFOVMarker : public rclcpp::Node
{
public:
    SensorFOVMarker() : Node("sensor_fov_marker")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "sensor_fov", 10);

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&SensorFOVMarker::publish_fov, this));
    }

private:
    void publish_fov()
    {
        visualization_msgs::msg::Marker fov;

        fov.header.frame_id = "lidar_link";
        fov.header.stamp = this->now();

        fov.ns = "fov";
        fov.id = 0;
        fov.type = visualization_msgs::msg::Marker::LINE_LIST;
        fov.action = visualization_msgs::msg::Marker::ADD;

        fov.scale.x = 0.02;  // Line width

        fov.color.r = 1.0;
        fov.color.g = 1.0;
        fov.color.b = 0.0;
        fov.color.a = 0.5;  // Semi-transparent

        // Create cone showing 180-degree FOV
        double range = 5.0;  // 5 meter range
        double fov_angle = M_PI;  // 180 degrees

        geometry_msgs::msg::Point origin;
        origin.x = 0; origin.y = 0; origin.z = 0;

        // Draw rays showing field of view
        for (double angle = -fov_angle/2; angle <= fov_angle/2; angle += 0.2) {
            geometry_msgs::msg::Point end;
            end.x = range * cos(angle);
            end.y = range * sin(angle);
            end.z = 0;

            fov.points.push_back(origin);
            fov.points.push_back(end);
        }

        marker_pub_->publish(fov);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

### Example 3: Moving Trajectory

```cpp
/**
 * @file trajectory_visualizer.cpp
 * @brief Show robot planned trajectory
 */

class TrajectoryVisualizer : public rclcpp::Node
{
public:
    TrajectoryVisualizer() : Node("trajectory_visualizer")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "trajectory", 10);

        timer_ = this->create_wall_timer(
            1s,
            std::bind(&TrajectoryVisualizer::update_trajectory, this));
    }

private:
    void update_trajectory()
    {
        visualization_msgs::msg::Marker traj;

        traj.header.frame_id = "map";
        traj.header.stamp = this->now();

        traj.ns = "trajectory";
        traj.id = 0;
        traj.type = visualization_msgs::msg::Marker::LINE_STRIP;
        traj.action = visualization_msgs::msg::Marker::ADD;

        traj.scale.x = 0.1;

        traj.color.r = 0.0;
        traj.color.g = 0.5;
        traj.color.b = 1.0;
        traj.color.a = 1.0;

        // Generate circular trajectory
        double radius = 2.0;
        for (double t = 0; t <= 2 * M_PI; t += 0.1) {
            geometry_msgs::msg::Point p;
            p.x = radius * cos(t);
            p.y = radius * sin(t);
            p.z = 0.5;
            traj.points.push_back(p);
        }

        marker_pub_->publish(traj);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## Interactive Markers

### What Are Interactive Markers?

**Interactive Markers**: Markers you can click, drag, and manipulate in RViz to send commands.

**Use cases**:
- Set robot goals by dragging
- Adjust parameters visually
- Create waypoints interactively
- Control robot arm pose

**Library**: `interactive_markers`

### Basic Interactive Marker

```cpp
#include "interactive_markers/interactive_marker_server.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"

class InteractiveGoal : public rclcpp::Node
{
public:
    InteractiveGoal() : Node("interactive_goal")
    {
        // Create interactive marker server
        server_ = std::make_shared<
            interactive_markers::InteractiveMarkerServer>(
            "goal_markers", this);

        create_interactive_marker();
    }

private:
    void create_interactive_marker()
    {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
        int_marker.name = "goal_position";
        int_marker.description = "Drag to set goal";

        // Visual marker (arrow)
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.scale.x = 1.0;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Create control for moving marker
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;
        control.markers.push_back(marker);
        int_marker.controls.push_back(control);

        // Add 6-DOF control
        visualization_msgs::msg::InteractiveMarkerControl move_control;
        move_control.name = "move_xy";
        move_control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
        move_control.orientation.w = 1;
        move_control.orientation.x = 0;
        move_control.orientation.y = 1;
        move_control.orientation.z = 0;
        int_marker.controls.push_back(move_control);

        // Insert marker
        server_->insert(int_marker);
        server_->setCallback(
            int_marker.name,
            std::bind(&InteractiveGoal::marker_feedback, this,
                     std::placeholders::_1));

        server_->applyChanges();
    }

    void marker_feedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
    {
        RCLCPP_INFO(this->get_logger(),
                   "Goal moved to (%.2f, %.2f, %.2f)",
                   feedback->pose.position.x,
                   feedback->pose.position.y,
                   feedback->pose.position.z);
    }

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
};
```

## Best Practices

**1. Use Namespaces**
```cpp
// Organize markers by category
marker.ns = "waypoints";
marker.ns = "obstacles";
marker.ns = "goals";
```

**2. Unique IDs within Namespace**
```cpp
// Each marker in namespace needs unique ID
for (int i = 0; i < waypoints.size(); ++i) {
    marker.id = i;  // Unique per waypoint
    // ...
}
```

**3. Set Proper Frame**
```cpp
// Use correct reference frame
marker.header.frame_id = "map";        // For global positions
marker.header.frame_id = "base_link";  // For robot-relative
marker.header.frame_id = "camera";     // For sensor-relative
```

**4. Update Timestamps**
```cpp
// Always use current time
marker.header.stamp = this->now();
```

**5. Use Appropriate Alpha**
```cpp
// Solid markers
marker.color.a = 1.0;

// Semi-transparent for overlays
marker.color.a = 0.5;

// Invisible (won't show!)
marker.color.a = 0.0;  // ❌ Don't do this
```

**6. Scale Appropriately**
```cpp
// Make markers visible but not overwhelming
// For 10m x 10m room:
marker.scale.x = 0.5;  // ✅ Good
marker.scale.x = 10.0; // ❌ Too large
marker.scale.x = 0.01; // ❌ Too small
```

## Debugging with Markers

### Visualizing Calculations

```cpp
// Show intermediate calculation results
void visualize_ray_cast(const Point & start, const Point & end,
                       bool hit_obstacle)
{
    visualization_msgs::msg::Marker ray;

    ray.header.frame_id = "map";
    ray.header.stamp = this->now();

    ray.ns = "raycasts";
    ray.id = raycast_id_++;
    ray.type = visualization_msgs::msg::Marker::LINE_LIST;

    ray.scale.x = 0.02;

    // Color based on result
    if (hit_obstacle) {
        ray.color.r = 1.0;  // Red if hit
    } else {
        ray.color.g = 1.0;  // Green if clear
    }
    ray.color.a = 0.5;

    ray.points.push_back(start);
    ray.points.push_back(end);

    ray.lifetime = rclcpp::Duration::from_seconds(1.0);

    debug_marker_pub_->publish(ray);
}
```

## Common Issues

**Marker Not Showing**:
```
Problem: Alpha is 0 or very low
Solution: Set marker.color.a = 1.0

Problem: Wrong frame_id
Solution: Check frame exists in TF tree

Problem: Marker behind camera
Solution: Adjust RViz camera view

Problem: Scale too small
Solution: Increase marker.scale values
```

**Markers Accumulating**:
```
Problem: Not deleting old markers
Solution: Use lifetimes or DELETE action

// Option 1: Temporary markers
marker.lifetime = rclcpp::Duration::from_seconds(5.0);

// Option 2: Delete before publishing new
delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
marker_pub_->publish(delete_marker);
```

## Summary

**Markers enable**:
- Visual debugging of robot logic
- Displaying abstract data (goals, paths, fields of view)
- Interactive robot control
- Annotating RViz scenes

**Key marker types**:
- **Shapes**: ARROW, CUBE, SPHERE, CYLINDER
- **Lines**: LINE_STRIP (connected), LINE_LIST (separate)
- **Text**: TEXT_VIEW_FACING
- **Meshes**: MESH_RESOURCE

**Important properties**:
- **Namespace + ID**: Unique identifier
- **Frame**: Coordinate reference
- **Lifetime**: Auto-deletion timer
- **Color alpha**: Transparency

**Best practices**:
- Organize with namespaces
- Update timestamps
- Use appropriate scale
- Clean up old markers

## What's Next?

**Next Lesson**: XACRO and Macros - Simplify URDF with parameterization

**You'll learn**:
- XACRO syntax and benefits
- Creating reusable robot components
- Parameterized robot descriptions
- Conditional inclusion

---

**Key Takeaway**: Markers are your visual debugging tool - use them liberally to understand what your robot is thinking!
