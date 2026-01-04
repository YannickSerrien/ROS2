cmake_minimum_required(VERSION 3.8)
project(service_exercise)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Calculate.srv"
)

# Calculator Server
add_executable(calculator_server src/calculator_server.cpp)
ament_target_dependencies(calculator_server rclcpp)

# Calculator Client
add_executable(calculator_client src/calculator_client.cpp)
ament_target_dependencies(calculator_client rclcpp)

# Link generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(calculator_server "${cpp_typesupport_target}")
target_link_libraries(calculator_client "${cpp_typesupport_target}")

install(TARGETS
  calculator_server
  calculator_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
