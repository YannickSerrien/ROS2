# Lesson 10: Build Systems (CMake and package.xml)

## Learning Objectives

- Understand CMake basics for C++ projects
- Master CMakeLists.txt for ROS2 packages
- Configure package.xml for dependencies
- Build and install ROS2 C++ packages
- Use colcon build effectively

## Introduction

Building C++ projects in ROS2 requires understanding **CMake** (build system) and **package.xml** (package manifest). Every ROS2 C++ package you create will use these files. This lesson covers exactly what you need for ROS2 development.

## ROS2 Package Structure

```
my_robot_package/
├── CMakeLists.txt          # Build instructions
├── package.xml             # Package metadata and dependencies
├── include/my_robot_package/  # Header files (.hpp)
│   └── my_node.hpp
├── src/                    # Source files (.cpp)
│   └── my_node.cpp
├── launch/                 # Launch files
│   └── robot.launch.py
└── config/                 # Configuration files
    └── params.yaml
```

## package.xml - Package Manifest

The `package.xml` file describes your package:

### Basic package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <!-- Package name (must match directory name) -->
  <name>my_robot_package</name>
  <version>1.0.0</version>
  <description>My robot control package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool dependency -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Build and execution dependencies -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <!-- Build-only dependencies -->
  <build_depend>rosidl_default_generators</build_depend>

  <!-- Execution-only dependencies -->
  <exec_depend>rosidl_default_runtime</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Dependency Types

| Tag | Meaning |
|-----|---------|
| `<buildtool_depend>` | Build system tools (e.g., ament_cmake) |
| `<build_depend>` | Needed only at build time |
| `<exec_depend>` | Needed only at runtime |
| `<depend>` | Both build and runtime (most common) |
| `<test_depend>` | Needed for testing |

## CMakeLists.txt - Build Instructions

### Minimal CMakeLists.txt for ROS2

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_package)

# C++17 standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(my_node src/my_node.cpp)

# Link dependencies
ament_target_dependencies(my_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Install targets
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
```

### Detailed CMakeLists.txt Sections

#### 1. Project Setup

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_package)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
```

#### 2. Find Dependencies

```cmake
# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
```

#### 3. Include Directories

```cmake
# Include header files
include_directories(include)

# Or more explicitly:
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)
```

#### 4. Create Executable

```cmake
# Single source file
add_executable(simple_node src/simple_node.cpp)

# Multiple source files
add_executable(complex_node
  src/complex_node.cpp
  src/helper_functions.cpp
  src/data_processor.cpp
)

# Link dependencies
ament_target_dependencies(simple_node
  rclcpp
  std_msgs
)
```

#### 5. Create Library

```cmake
# Create a library
add_library(my_library SHARED
  src/sensor_interface.cpp
  src/data_processor.cpp
)

# Link dependencies to library
ament_target_dependencies(my_library
  rclcpp
  std_msgs
)

# Use library in executable
add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node my_library)
ament_target_dependencies(my_node rclcpp)
```

#### 6. Install Rules

```cmake
# Install executables
install(TARGETS
  my_node
  another_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  my_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
```

#### 7. Export Package

```cmake
# Export dependencies for downstream packages
ament_export_dependencies(
  rclcpp
  std_msgs
)

# Export include directories
ament_export_include_directories(include)

# Export libraries
ament_export_libraries(my_library)

# Finalize package
ament_package()
```

## Complete Example Package

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_controller</name>
  <version>1.0.0</version>
  <description>Robot motion controller</description>
  <maintainer email="dev@robot.com">Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_controller)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(controller_lib SHARED
  src/pid_controller.cpp
  src/motion_planner.cpp
)

ament_target_dependencies(controller_lib
  rclcpp
  geometry_msgs
)

# Create executables
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node controller_lib)
ament_target_dependencies(controller_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

add_executable(teleoperation src/teleop_node.cpp)
ament_target_dependencies(teleoperation
  rclcpp
  geometry_msgs
)

# Install
install(TARGETS
  controller_node
  teleoperation
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(controller_lib)
ament_export_dependencies(
  rclcpp
  geometry_msgs
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Building Packages

### Using colcon

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build all packages
colcon build

# Build specific package
colcon build --packages-select robot_controller

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build with symlink install (recommended for development)
colcon build --symlink-install

# Build packages up to a specific one
colcon build --packages-up-to robot_controller

# Parallel build (4 jobs)
colcon build --parallel-workers 4

# Show detailed output
colcon build --event-handlers console_direct+
```

### Build Workflow

```bash
# 1. Create package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_package \
    --dependencies rclcpp std_msgs

# 2. Write code
# Edit src/my_node.cpp

# 3. Update CMakeLists.txt and package.xml

# 4. Build
cd ~/ros2_ws
colcon build --packages-select my_package

# 5. Source workspace
source install/setup.bash

# 6. Run
ros2 run my_package my_node
```

## Common CMake Patterns

### Custom Messages

```cmake
# In package.xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

# In CMakeLists.txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMessage.msg"
  "srv/CustomService.srv"
  DEPENDENCIES std_msgs
)

# Link to executable
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(my_node "${cpp_typesupport_target}")
```

### External Libraries

```cmake
# Find external library
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Link to target
target_link_libraries(my_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

include_directories(${OpenCV_INCLUDE_DIRS})
```

## Troubleshooting

### Common Errors

**Error: Package not found**
```bash
# Solution: Install dependency
sudo apt install ros-humble-<package-name>
# Or add to package.xml and run rosdep
rosdep install --from-paths src --ignore-src -y
```

**Error: Undefined reference**
```cmake
# Solution: Link library
target_link_libraries(my_node my_library)
# Or add to ament_target_dependencies
```

**Error: Header not found**
```cmake
# Solution: Add include directory
include_directories(include)
# Or:
target_include_directories(my_node PUBLIC include)
```

## Best Practices

### DO ✓

```cmake
# Use ament_target_dependencies (easier)
ament_target_dependencies(my_node rclcpp std_msgs)

# Install all necessary files
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Use symlink install for development
colcon build --symlink-install

# Specify all dependencies in package.xml
```

### DON'T ✗

```cmake
# Don't hardcode paths
# Bad: include_directories(/usr/local/include)
# Good: find_package(MyLib REQUIRED)

# Don't forget ament_package()
# Must be last line in CMakeLists.txt

# Don't skip tests section
```

## Quick Reference

### Create Package

```bash
ros2 pkg create --build-type ament_cmake my_package \
    --dependencies rclcpp std_msgs geometry_msgs
```

### Essential CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(pkg_name)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
add_executable(node src/node.cpp)
ament_target_dependencies(node rclcpp)
install(TARGETS node DESTINATION lib/${PROJECT_NAME})
ament_package()
```

### Build and Run

```bash
colcon build --packages-select my_package
source install/setup.bash
ros2 run my_package my_node
```

## Summary

**Key Takeaways:**
- `package.xml` declares dependencies and metadata
- `CMakeLists.txt` defines build instructions
- `colcon build` builds ROS2 packages
- `ament_cmake` is the ROS2 build system
- Use `--symlink-install` for development
- Install launch, config, and include files
- Always call `ament_package()` last

**Critical for ROS2:**
- Every C++ package needs both files
- Proper dependency declaration prevents build errors
- Install rules make files available to other packages

## Practice

1. Create a simple ROS2 package from scratch
2. Add a library and link it to an executable
3. Install launch files and config files
4. Build and test the package

## What's Next?

- **Congratulations!** You've completed Module 1: C++ Refresher
- **Next Module**: [Module 2: ROS2 Fundamentals](../../02-ros2-fundamentals/)
- **Practice**: Complete exercises and mini-project

---

**Master CMake and package.xml, and you can build any ROS2 package!**
