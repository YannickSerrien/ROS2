cmake_minimum_required(VERSION 3.8)
project(integration_exercise)

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

# Data Generator
add_executable(data_generator src/data_generator.cpp)
ament_target_dependencies(data_generator rclcpp std_msgs)

# Data Processor
add_executable(data_processor src/data_processor.cpp)
ament_target_dependencies(data_processor rclcpp std_msgs std_srvs)

# Data Monitor
add_executable(data_monitor src/data_monitor.cpp)
ament_target_dependencies(data_monitor rclcpp std_msgs std_srvs)

install(TARGETS
  data_generator
  data_processor
  data_monitor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
