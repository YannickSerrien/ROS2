cmake_minimum_required(VERSION 3.8)
project(temperature_monitor)

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
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TemperatureReading.msg"
  "msg/Alert.msg"
  "srv/GetStats.srv"
  DEPENDENCIES std_msgs
)

# Sensor Node
add_executable(temp_sensor_node src/temp_sensor_node.cpp)
ament_target_dependencies(temp_sensor_node rclcpp std_msgs)

# Monitor Node
add_executable(monitor_node src/monitor_node.cpp)
ament_target_dependencies(monitor_node rclcpp std_msgs std_srvs)

# Alerter Node
add_executable(alerter_node src/alerter_node.cpp)
ament_target_dependencies(alerter_node rclcpp)

# Dashboard Node
add_executable(dashboard_node src/dashboard_node.cpp)
ament_target_dependencies(dashboard_node rclcpp)

# Link generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(temp_sensor_node "${cpp_typesupport_target}")
target_link_libraries(monitor_node "${cpp_typesupport_target}")
target_link_libraries(alerter_node "${cpp_typesupport_target}")
target_link_libraries(dashboard_node "${cpp_typesupport_target}")

install(TARGETS
  temp_sensor_node
  monitor_node
  alerter_node
  dashboard_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
