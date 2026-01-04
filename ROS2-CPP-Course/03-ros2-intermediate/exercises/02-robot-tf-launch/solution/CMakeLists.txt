cmake_minimum_required(VERSION 3.8)
project(robot_tf_launch)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(static_tf_node src/static_tf_node.cpp)
ament_target_dependencies(static_tf_node rclcpp tf2 tf2_ros geometry_msgs)

add_executable(odom_broadcaster src/odom_broadcaster.cpp)
ament_target_dependencies(odom_broadcaster rclcpp tf2 tf2_ros geometry_msgs)

add_executable(sensor_monitor src/sensor_monitor.cpp)
ament_target_dependencies(sensor_monitor rclcpp tf2 tf2_ros geometry_msgs)

install(TARGETS static_tf_node odom_broadcaster sensor_monitor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
