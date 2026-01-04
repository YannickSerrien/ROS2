cmake_minimum_required(VERSION 3.8)
project(actions_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
  "action/MoveRobot.action"
  DEPENDENCIES std_msgs
)

# Action Server
add_executable(fibonacci_server src/fibonacci_server.cpp)
ament_target_dependencies(fibonacci_server
  rclcpp
  rclcpp_action
  rclcpp_components
)

# Action Client
add_executable(fibonacci_client src/fibonacci_client.cpp)
ament_target_dependencies(fibonacci_client
  rclcpp
  rclcpp_action
  rclcpp_components
)

# Task Manager (advanced example)
add_executable(task_manager src/task_manager.cpp)
ament_target_dependencies(task_manager
  rclcpp
  rclcpp_action
  rclcpp_components
)

# Link custom interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(fibonacci_server "${cpp_typesupport_target}")
target_link_libraries(fibonacci_client "${cpp_typesupport_target}")
target_link_libraries(task_manager "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  fibonacci_server
  fibonacci_client
  task_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
