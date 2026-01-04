cmake_minimum_required(VERSION 3.8)
project(composition_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Image Producer Component
add_library(image_producer_component SHARED
  src/image_producer_component.cpp
)
ament_target_dependencies(image_producer_component
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(image_producer_component
  PLUGIN "composition_example::ImageProducerComponent"
  EXECUTABLE image_producer_node
)

# Image Processor Component
add_library(image_processor_component SHARED
  src/image_processor_component.cpp
)
ament_target_dependencies(image_processor_component
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(image_processor_component
  PLUGIN "composition_example::ImageProcessorComponent"
  EXECUTABLE image_processor_node
)

# Talker Component (simple example)
add_library(talker_component SHARED
  src/talker_component.cpp
)
ament_target_dependencies(talker_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_node(talker_component
  PLUGIN "composition_example::TalkerComponent"
  EXECUTABLE talker_node
)

# Listener Component (simple example)
add_library(listener_component SHARED
  src/listener_component.cpp
)
ament_target_dependencies(listener_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_node(listener_component
  PLUGIN "composition_example::ListenerComponent"
  EXECUTABLE listener_node
)

# Install component libraries
install(TARGETS
  image_producer_component
  image_processor_component
  talker_component
  listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
