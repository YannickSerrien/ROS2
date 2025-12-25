cmake_minimum_required(VERSION 3.8)
project(parameters_example)

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

# Parameter node
add_executable(parameter_node src/parameter_node.cpp)
ament_target_dependencies(parameter_node rclcpp)

# Configurable robot
add_executable(configurable_robot src/configurable_robot.cpp)
ament_target_dependencies(configurable_robot rclcpp std_msgs)

# Dynamic parameters
add_executable(dynamic_parameters src/dynamic_parameters.cpp)
ament_target_dependencies(dynamic_parameters rclcpp std_msgs)

# Install executables
install(TARGETS
  parameter_node
  configurable_robot
  dynamic_parameters
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
