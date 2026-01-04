# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Purpose

This is a comprehensive **educational course repository** teaching ROS2 (Robot Operating System 2) and modern C++ from fundamentals to advanced topics. It targets Python developers transitioning to robotics with C++.

**Key Point**: This is course content creation, not software development. You are building lessons, examples, exercises, and projects for students to learn from.

## Course Architecture

### Module Structure (8 Modules Total)

Each module follows this consistent pattern:

```
XX-module-name/
├── README.md           # Module overview with Mermaid diagrams, concept maps
├── lessons/            # Markdown lessons (10-12 per module)
├── examples/           # Complete ROS2 packages demonstrating concepts
├── exercises/          # Student practice with starter/ and solution/
└── mini-project/       # Integration project with starter/ and solution/
```

**Module Progression**:
- **Module 0**: Getting Started (setup, installation)
- **Module 1**: C++ Refresher (modern C++17/20 features)
- **Module 2**: ROS2 Fundamentals (nodes, topics, services, parameters)
- **Module 3**: ROS2 Intermediate (actions, TF2, launch files)
- **Module 4**: Simulation (Gazebo, RViz2, URDF)
- **Module 5**: Hardware Integration (sensors, actuators)
- **Module 6**: ROS2 Advanced (lifecycle, composition, QoS)
- **Module 7**: Projects (6 real-world applications)

### Current Completion Status

**Complete** (100%):
- Module 0: Getting Started (5 files)
- Module 1: C++ Refresher (10 lessons, 4 examples, 1 exercise with solution, 1 mini-project)
- Module 2: ROS2 Fundamentals (12 lessons, 6 example packages with 22 nodes, 4 exercises, 1 mini-project)

**Remaining**: Modules 3-7, Resources folder

## ROS2 Package Structure

ROS2 examples and exercises use standard ROS2 package structure:

```
package_name/
├── package.xml              # ROS2 package metadata
├── CMakeLists.txt          # Build configuration
├── src/                    # C++ source files
│   ├── node_name.cpp
│   └── ...
├── msg/                    # Custom message definitions (optional)
├── srv/                    # Custom service definitions (optional)
├── config/                 # YAML configuration files (optional)
├── launch/                 # Python launch files (optional)
└── README.md               # Package documentation
```

### Building ROS2 Packages

All ROS2 packages use the `colcon` build system:

```bash
# From workspace root (or package directory)
colcon build --packages-select package_name
source install/setup.bash  # Must source after build

# Run a node
ros2 run package_name executable_name

# With parameters
ros2 run package_name node_name --ros-args -p param:=value

# With parameter file
ros2 run package_name node_name --ros-args --params-file config/params.yaml
```

**Important**: Each ROS2 package needs both `package.xml` and `CMakeLists.txt` properly configured.

## Content Creation Standards

### Lesson Files (Markdown)

**Every lesson must include**:
1. **Learning Objectives** - Clear, measurable goals
2. **Python Comparison** - For Python developers transitioning to C++
3. **ROS2 Context** - Why this matters for robotics
4. **Code Examples** - Inline demonstrations
5. **Common Pitfalls** - What students struggle with
6. **Best Practices** - Production-ready patterns
7. **Summary** - Key takeaways
8. **Next Steps** - Link to related content

**Lesson Template Reference**: See `01-cpp-refresher/lessons/01-modern-cpp-overview.md` for style guide.

### Code Examples

**Standards for all code**:
- **Heavily commented** - Explain WHY, not just WHAT
- **Modern C++17/20** - Use smart pointers, auto, lambdas, std::chrono
- **ROS2 patterns** - Follow rclcpp idioms (std::make_shared, std::bind or lambdas)
- **Self-contained** - Include build instructions in comments
- **Production-ready** - Error handling, validation, proper logging levels

**C++ File Header Template**:
```cpp
/**
 * @file filename.cpp
 * @brief Brief description
 *
 * Detailed explanation of what this demonstrates.
 * Used in: Module X, Lesson Y
 */
```

**ROS2 Node Pattern**:
```cpp
class NodeName : public rclcpp::Node
{
public:
    NodeName() : Node("node_name") {
        // Declare parameters
        // Create publishers/subscribers/services
        // Initialize timers
    }

private:
    void callback() { /* ... */ }

    // Member variables
    rclcpp::Publisher<...>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeName>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Exercise Structure

Each exercise has TWO complete packages:

```
XX-exercise-name/
├── README.md          # Instructions, objectives, hints, testing
├── starter/           # Incomplete code with TODOs
│   ├── src/
│   ├── CMakeLists.txt
│   └── package.xml
└── solution/          # Complete working solution
    ├── src/
    ├── CMakeLists.txt
    └── package.xml
```

**Starter code must**:
- Have clear `// TODO:` comments
- Provide hints in comments
- Include just enough to compile (or clear compilation errors)
- Guide students without giving away the solution

### README Files with Conceptual Overviews

Module README files follow enhanced template with:
- **Mermaid diagrams** showing concept relationships
- **4-part explanations** for each major concept:
  - **What it is**: Clear definition
  - **What it does/needed for**: Functional purpose
  - **Why it matters**: Significance in ROS2/robotics
  - **How it connects**: Prerequisites, dependencies, related concepts
- **Module position diagram** showing place in course
- **Lesson dependency flow** (using Mermaid)
- **Integration examples** showing how concepts work together

**Mermaid Color Coding**:
- Critical concepts: `fill:#ff6b6b` (red)
- Important: `fill:#4dabf7` (blue)
- Foundational: `fill:#51cf66` (green)
- Current module: `fill:#4CAF50,stroke-width:3px`

## Development Workflow

### Creating a New Module

1. **Create directory structure**:
   ```bash
   mkdir -p XX-module-name/{lessons,examples,exercises,mini-project}
   ```

2. **Create README.md** with:
   - Module overview
   - Mermaid diagrams (concept map, module position, lesson flow)
   - Conceptual breakdowns (what/purpose/why/connections)
   - Learning objectives
   - Structure (lessons, examples, exercises, mini-project)
   - Verification questions

3. **Create lessons** (10-12 markdown files):
   - Follow lesson template standards
   - Progressive difficulty
   - Cross-reference other lessons

4. **Create examples** (4-6 ROS2 packages):
   - Each package demonstrates specific concepts
   - Include README with running instructions
   - Complete, compilable, tested

5. **Create exercises** (3-4 with starter + solution):
   - Reinforce lesson concepts
   - Progressive complexity
   - Clear objectives and hints

6. **Create mini-project**:
   - Integrates all module concepts
   - Realistic scenario
   - Starter template + complete solution

### Creating ROS2 Package Examples

**Template for CMakeLists.txt**:
```cmake
cmake_minimum_required(VERSION 3.8)
project(package_name)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# Add other dependencies

add_executable(node_name src/node_name.cpp)
ament_target_dependencies(node_name rclcpp std_msgs)

install(TARGETS node_name
  DESTINATION lib/${PROJECT_NAME}
)

# If you have config/launch directories:
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

**Template for package.xml**:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>package_name</name>
  <version>1.0.0</version>
  <description>Brief description</description>
  <maintainer email="student@example.com">ROS2 Course Student</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Custom Message/Service Definitions

For packages with custom interfaces:

**package.xml additions**:
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**CMakeLists.txt additions**:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MessageName.msg"
  "srv/ServiceName.srv"
  DEPENDENCIES std_msgs
)

# After add_executable:
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(node_name "${cpp_typesupport_target}")
```

## Content Philosophy

### For Python Developers

This course specifically targets Python developers, so:
- **Always compare** C++ concepts to Python equivalents
- **Explain memory management** (Python handles this automatically)
- **Clarify compilation** vs interpretation
- **Show type differences** (static vs dynamic typing)
- **Explain pointers/references** (no direct Python equivalent)

### Practical ROS2 Focus

- All C++ lessons connect to ROS2 usage
- Code examples should be ROS2 nodes when possible
- Explain why ROS2 uses specific C++ features
- Show real robotics scenarios

### Progressive Difficulty

- Module N builds on Modules 0 to N-1
- Within modules: lessons → examples → exercises → mini-project
- Don't introduce concepts before prerequisites

## Common Patterns

### ROS2 Timer with Lambda
```cpp
timer_ = this->create_wall_timer(
    1s,  // using namespace std::chrono_literals
    [this]() { this->timer_callback(); }
);
```

### ROS2 Subscriber with std::bind
```cpp
subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic_name",
    10,  // QoS depth
    std::bind(&NodeClass::callback, this, std::placeholders::_1)
);
```

### Parameter Declaration and Retrieval
```cpp
this->declare_parameter("param_name", default_value);
auto value = this->get_parameter("param_name").as_double();
```

### Service Callback Signature
```cpp
void service_callback(
    const std::shared_ptr<package_name::srv::ServiceType::Request> request,
    std::shared_ptr<package_name::srv::ServiceType::Response> response)
{
    // Process request, fill response
}
```

## Important Naming Conventions

- **Node names**: snake_case (e.g., `temp_sensor_node`)
- **Topic names**: /snake_case with leading slash (e.g., `/temperature_data`)
- **Package names**: snake_case (e.g., `pubsub_example`)
- **Class names**: PascalCase (e.g., `TempSensorNode`)
- **Variable names**: snake_case (e.g., `current_temp_`)
- **Member variables**: trailing underscore (e.g., `publisher_`)

## Cross-Referencing

When creating content:
- Link to prerequisite lessons (e.g., "See [Lesson 4: Smart Pointers](../lessons/04-smart-pointers.md)")
- Reference related examples (e.g., "Demonstrated in [pubsub_example](../examples/pubsub_example/)")
- Point to future topics (e.g., "You'll use this in Module 3 when learning about actions")
- Update module README when adding new content

## Testing Checklist for ROS2 Code

Before marking a package complete:
- [ ] Compiles without errors using `colcon build`
- [ ] All nodes run without crashes
- [ ] README includes build and run instructions
- [ ] Sample output documented
- [ ] Parameters have sensible defaults
- [ ] Logging uses appropriate levels (DEBUG/INFO/WARN/ERROR)
- [ ] Code follows ROS2 style guide
- [ ] Comments explain non-obvious logic

## Module-Specific Notes

### Module 1 (C++ Refresher)
- Examples are standalone C++ (not ROS2 packages)
- Compile with: `g++ -std=c++17 -Wall -Wextra file.cpp -o output`
- Focus on features used heavily in ROS2

### Module 2 (ROS2 Fundamentals)
- All examples are ROS2 packages
- Cover: nodes, topics, services, parameters, timers, logging, executors
- 6 example packages with 22 total nodes
- 4 exercises covering pub/sub, services, parameters, integration

### Modules 3-7
- Follow same pattern as Module 2
- Increase complexity progressively
- More integration, real-world scenarios
- Module 7 is project-based (no traditional lessons)

## Resources Folder

Will contain:
- `cheat-sheets/` - Quick reference cards (ROS2 commands, C++ syntax, CMake)
- `troubleshooting.md` - Common errors and solutions
- `best-practices.md` - Production patterns
- `tools.md` - Development tools guide
- `glossary.md` - ROS2 and robotics terminology

## Quality Standards Summary

**All content must**:
- Be production-quality (not drafts)
- Include complete examples (not pseudocode)
- Provide clear learning value
- Connect to ROS2 robotics context
- Be well-formatted Markdown or properly structured code
- Follow established patterns in existing modules

**For lessons**: Teach concepts clearly with examples and context
**For code**: Work correctly, demonstrate best practices, include documentation
**For exercises**: Challenge students appropriately, provide guidance without solutions
**For projects**: Integrate multiple concepts, simulate realistic scenarios

## Next Development Steps

Based on current completion status, continue with:
1. Module 3: ROS2 Intermediate (12 lessons, 4 packages, 3 exercises, mini-project)
2. Module 4: Simulation (10 lessons, 4 packages, 3 exercises, mini-project)
3. Module 5: Hardware Integration
4. Module 6: ROS2 Advanced
5. Module 7: Projects (6 complete projects)
6. Resources folder (cheat sheets, troubleshooting)

Maintain the same quality and detail level as Modules 0-2.
