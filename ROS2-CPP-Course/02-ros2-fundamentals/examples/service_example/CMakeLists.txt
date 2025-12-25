cmake_minimum_required(VERSION 3.8)
project(service_example)

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
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# Add server
add_executable(add_server src/add_server.cpp)
ament_target_dependencies(add_server
  rclcpp
  example_interfaces
)

# Add client
add_executable(add_client src/add_client.cpp)
ament_target_dependencies(add_client
  rclcpp
  example_interfaces
)

# Calculator server
add_executable(calculator_server src/calculator_server.cpp)
ament_target_dependencies(calculator_server
  rclcpp
  example_interfaces
  std_srvs
)

# Robot controller server
add_executable(robot_controller_server src/robot_controller_server.cpp)
ament_target_dependencies(robot_controller_server
  rclcpp
  std_srvs
)

# Install executables
install(TARGETS
  add_server
  add_client
  calculator_server
  robot_controller_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
