cmake_minimum_required(VERSION 3.8)
project(executor_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(single_threaded src/single_threaded_node.cpp)
ament_target_dependencies(single_threaded rclcpp)

add_executable(multi_threaded src/multi_threaded_node.cpp)
ament_target_dependencies(multi_threaded rclcpp)

add_executable(callback_groups src/callback_groups_node.cpp)
ament_target_dependencies(callback_groups rclcpp)

add_executable(thread_safe_counter src/thread_safe_counter.cpp)
ament_target_dependencies(thread_safe_counter rclcpp)

install(TARGETS
  single_threaded multi_threaded callback_groups thread_safe_counter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
