cmake_minimum_required(VERSION 3.8)
project(rviz_visualization_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interactive_markers REQUIRED)

# Marker publisher node
add_executable(marker_publisher src/marker_publisher.cpp)
ament_target_dependencies(marker_publisher
  rclcpp visualization_msgs geometry_msgs tf2 tf2_ros
)

# Path visualizer node
add_executable(path_visualizer src/path_visualizer.cpp)
ament_target_dependencies(path_visualizer
  rclcpp visualization_msgs nav_msgs geometry_msgs
)

# Interactive marker node
add_executable(interactive_marker_server src/interactive_marker_server.cpp)
ament_target_dependencies(interactive_marker_server
  rclcpp interactive_markers visualization_msgs geometry_msgs
)

# Install executables
install(TARGETS
  marker_publisher
  path_visualizer
  interactive_marker_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
