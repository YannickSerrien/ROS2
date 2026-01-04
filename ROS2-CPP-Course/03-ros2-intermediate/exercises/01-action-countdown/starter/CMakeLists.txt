cmake_minimum_required(VERSION 3.8)
project(action_countdown)

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
find_package(rosidl_default_generators REQUIRED)

# Generate custom action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Countdown.action"
)

# Action Server
add_executable(countdown_server src/countdown_server.cpp)
ament_target_dependencies(countdown_server
  rclcpp
  rclcpp_action
  rclcpp_components
)

# Action Client
add_executable(countdown_client src/countdown_client.cpp)
ament_target_dependencies(countdown_client
  rclcpp
  rclcpp_action
  rclcpp_components
)

# Link custom interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(countdown_server "${cpp_typesupport_target}")
target_link_libraries(countdown_client "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  countdown_server
  countdown_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
