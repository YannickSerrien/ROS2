cmake_minimum_required(VERSION 3.8)
project(pubsub_exercise)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Counter Publisher
add_executable(counter_publisher src/counter_publisher.cpp)
ament_target_dependencies(counter_publisher rclcpp std_msgs)

# Counter Listener
add_executable(counter_listener src/counter_listener.cpp)
ament_target_dependencies(counter_listener rclcpp std_msgs)

install(TARGETS
  counter_publisher
  counter_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
