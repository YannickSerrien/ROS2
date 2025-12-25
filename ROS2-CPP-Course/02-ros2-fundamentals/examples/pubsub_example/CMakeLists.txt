cmake_minimum_required(VERSION 3.8)
project(pubsub_example)

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

# Talker node
add_executable(talker src/talker_node.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Listener node
add_executable(listener src/listener_node.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# Parameter talker node
add_executable(parameter_talker src/parameter_talker.cpp)
ament_target_dependencies(parameter_talker rclcpp std_msgs)

# Install executables
install(TARGETS
  talker
  listener
  parameter_talker
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if added later)
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
