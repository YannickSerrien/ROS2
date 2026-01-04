cmake_minimum_required(VERSION 3.8)
project(robot_arm_project)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveArm.action"
  DEPENDENCIES geometry_msgs
)

# Include directory
include_directories(include)

# Arm Controller
add_executable(arm_controller src/arm_controller.cpp src/kinematics.cpp)
ament_target_dependencies(arm_controller
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
)

# Move Arm Action Server
add_executable(move_arm_server src/move_arm_action_server.cpp src/kinematics.cpp)
ament_target_dependencies(move_arm_server
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2
  tf2_ros
  geometry_msgs
)

# Move Arm Action Client
add_executable(move_arm_client src/move_arm_action_client.cpp)
ament_target_dependencies(move_arm_client
  rclcpp
  rclcpp_action
  rclcpp_components
)

# Link custom interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(move_arm_server "${cpp_typesupport_target}")
target_link_libraries(move_arm_client "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  arm_controller
  move_arm_server
  move_arm_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
