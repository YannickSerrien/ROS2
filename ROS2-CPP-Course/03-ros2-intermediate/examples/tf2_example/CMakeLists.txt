cmake_minimum_required(VERSION 3.8)
project(tf2_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Static Transform Publisher
add_executable(static_tf_publisher src/static_tf_publisher.cpp)
ament_target_dependencies(static_tf_publisher
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

# Dynamic Transform Broadcaster
add_executable(dynamic_tf_broadcaster src/dynamic_tf_broadcaster.cpp)
ament_target_dependencies(dynamic_tf_broadcaster
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

# Transform Listener
add_executable(tf_listener src/tf_listener.cpp)
ament_target_dependencies(tf_listener
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  std_msgs
)

# Robot Transform Tree (complete demo)
add_executable(robot_tf_tree src/robot_tf_tree.cpp)
ament_target_dependencies(robot_tf_tree
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

# Install executables
install(TARGETS
  static_tf_publisher
  dynamic_tf_broadcaster
  tf_listener
  robot_tf_tree
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
