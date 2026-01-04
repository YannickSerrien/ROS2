cmake_minimum_required(VERSION 3.8)
project(launch_example)

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
find_package(geometry_msgs REQUIRED)

# Node 1: Sensor Simulator
add_executable(sensor_node src/sensor_node.cpp)
ament_target_dependencies(sensor_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Node 2: Data Processor
add_executable(processor_node src/processor_node.cpp)
ament_target_dependencies(processor_node
  rclcpp
  std_msgs
)

# Node 3: Controller
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node
  rclcpp
  geometry_msgs
)

# Install executables
install(TARGETS
  sensor_node
  processor_node
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
