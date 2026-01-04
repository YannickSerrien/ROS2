cmake_minimum_required(VERSION 3.8)
project(timer_logging_example)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Periodic publisher
add_executable(periodic_publisher src/periodic_publisher.cpp)
ament_target_dependencies(periodic_publisher rclcpp std_msgs)

# Heartbeat node
add_executable(heartbeat src/heartbeat_node.cpp)
ament_target_dependencies(heartbeat rclcpp)

# Multi-timer node
add_executable(multi_timer src/multi_timer_node.cpp)
ament_target_dependencies(multi_timer rclcpp)

# Watchdog node
add_executable(watchdog src/watchdog_node.cpp)
ament_target_dependencies(watchdog rclcpp std_msgs)

# Install executables
install(TARGETS
  periodic_publisher
  heartbeat
  multi_timer
  watchdog
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
