cmake_minimum_required(VERSION 3.8)
project(multi_node_communication)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(sensor_node src/sensor_node.cpp)
ament_target_dependencies(sensor_node rclcpp std_msgs)

add_executable(processor_node src/processor_node.cpp)
ament_target_dependencies(processor_node rclcpp std_msgs)

add_executable(monitor_node src/monitor_node.cpp)
ament_target_dependencies(monitor_node rclcpp std_msgs std_srvs)

add_executable(coordinator_node src/coordinator_node.cpp)
ament_target_dependencies(coordinator_node rclcpp std_srvs)

install(TARGETS
  sensor_node processor_node monitor_node coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
