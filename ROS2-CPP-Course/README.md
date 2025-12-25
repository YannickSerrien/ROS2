# ROS2 and C++ Course: From Basics to Advanced

A comprehensive, hands-on course for learning ROS2 (Robot Operating System 2) and modern C++ from fundamentals to advanced topics. Designed for experienced Python developers transitioning to robotics with C++.

## Course Overview

This course takes you on a complete journey from C++ refresher to building production-ready robotic systems. Through structured lessons, practical code examples, hands-on exercises, and real-world projects, you'll gain the skills needed to develop professional robotics applications.

## Who This Course Is For

- **Python developers** wanting to learn C++ and robotics
- **Engineers** transitioning to ROS2 from ROS1 or other frameworks
- **Students** learning robotics and autonomous systems
- **Hobbyists** building robot projects
- Anyone wanting a **structured, practical** approach to ROS2

## Prerequisites

- **Programming experience** (Python or any language)
- **Linux basics** (command line, file system)
- **Basic math** (algebra, trigonometry)
- **Motivation** to learn and build!

No prior C++ or ROS experience required.

## What You'll Learn

### C++ Skills
- Modern C++ features (C++17/20)
- Smart pointers and memory management
- Object-oriented programming
- Templates and lambda functions
- STL containers and algorithms
- CMake build system

### ROS2 Skills
- ROS2 architecture and concepts
- Nodes, topics, services, actions
- Custom messages and interfaces
- TF2 coordinate transformations
- Launch files and parameters
- Lifecycle nodes and composition
- Quality of Service (QoS)
- Multi-robot systems

### Robotics Skills
- Robot simulation (Gazebo, RViz2)
- URDF robot modeling
- Sensor integration (Lidar, cameras, IMU)
- Motion control
- SLAM and navigation
- Production deployment

## Course Structure

The course is organized into **8 modules** with progressive difficulty:

### [Module 0: Getting Started](00-getting-started/)
Setup your development environment and learn ROS2 basics.
- ROS2 installation (Ubuntu/Windows/Docker)
- IDE configuration
- Workspace creation
- Essential commands

### [Module 1: C++ Refresher](01-cpp-refresher/)
Master modern C++ features essential for ROS2 development.
- 10 focused lessons on C++17/20 features
- 4 code examples with detailed comments
- 3 exercises with solutions
- **Mini-Project**: Templated sensor data handler class

**Time**: 1-2 weeks

### [Module 2: ROS2 Fundamentals](02-ros2-fundamentals/)
Learn core ROS2 concepts and communication patterns.
- 12 comprehensive lessons
- 6 complete example packages
- 4 progressive exercises
- **Mini-Project**: Temperature monitoring system

**Time**: 2-3 weeks

### [Module 3: ROS2 Intermediate](03-ros2-intermediate/)
Advanced communication, custom interfaces, and transforms.
- 12 lessons on actions, launch files, TF2
- 4 example packages
- 3 challenging exercises
- **Mini-Project**: 2D robot arm with forward kinematics

**Time**: 2-3 weeks

### [Module 4: Simulation](04-simulation/)
Visualize and simulate robots in virtual environments.
- 10 lessons on RViz2, URDF, Gazebo
- 4 simulation examples
- 3 hands-on exercises
- **Mini-Project**: Mobile robot in Gazebo with sensors

**Time**: 2-3 weeks

### [Module 5: Hardware Integration](05-hardware-integration/)
Interface with real sensors and actuators.
- 10 lessons on sensor drivers, motor control, safety
- 4 hardware interface examples
- 3 practical exercises
- **Mini-Project**: Multi-sensor fusion system

**Time**: 2-3 weeks

### [Module 6: ROS2 Advanced](06-ros2-advanced/)
Production-ready features and optimization techniques.
- 12 lessons on lifecycle, composition, QoS
- 4 advanced examples
- 3 production-focused exercises
- **Mini-Project**: Production perception pipeline

**Time**: 2-3 weeks

### [Module 7: Projects](07-projects/)
Apply your skills to real-world robotic applications.
- **Line Follower**: Vision-based line following
- **Obstacle Avoider**: Lidar-based navigation
- **Pick and Place**: Robotic arm manipulation
- **SLAM Navigation**: Autonomous navigation
- **Multi-Robot Coordination**: Fleet management
- **Capstone**: Autonomous delivery robot

**Time**: 4-8 weeks

### [Resources](resources/)
Reference materials, cheat sheets, and troubleshooting guides.

## Learning Paths

### Fast Track (3-4 months)
Focus on essentials, skip optional exercises.
- Modules 0-2: Core ROS2
- Module 3: Actions and TF2 only
- Module 4: Gazebo basics
- Module 7: 2-3 projects

### Comprehensive (6-8 months)
Complete all modules, exercises, and projects.
- All modules sequentially
- All exercises and mini-projects
- All 6 projects

### Simulation-Focused
For those interested in virtual robotics.
- Modules 0-4 fully
- Module 7: Line follower, obstacle avoider, SLAM

### Hardware-Focused
For those building physical robots.
- Modules 0-3 fully
- Module 5 fully
- Module 7: Hardware-based projects

## How to Use This Course

### 1. Sequential Learning (Recommended)
Work through modules in order. Each builds on previous concepts.

```
Module 0 → Module 1 → Module 2 → ... → Module 7
```

### 2. Study Pattern
For each module:
1. **Read lessons** - Understand concepts
2. **Run examples** - See code in action
3. **Complete exercises** - Practice independently
4. **Build mini-project** - Integrate knowledge
5. **Review and reflect** - Solidify understanding

### 3. Code Philosophy
- **Read** examples to understand patterns
- **Type** code yourself (don't copy-paste)
- **Experiment** with modifications
- **Break** things and fix them
- **Build** your own variations

### 4. Getting Help
- Check `resources/troubleshooting.md` for common issues
- Review `resources/references.md` for documentation
- Consult ROS2 official docs
- Search ROS Answers and forums

## Projects Philosophy

Each project includes:
- **Starter code** - Framework to begin
- **Clear objectives** - What to build
- **Hints** - Guidance when stuck
- **Solution** - Reference implementation

**Important**: Try building projects yourself first! Only look at solutions after attempting or when truly stuck.

## Course Features

### Hands-On Learning
- 50+ complete code examples
- 30+ exercises with solutions
- 6 mini-projects
- 6 major projects

### Real-World Focus
- Industry-standard practices
- Production-ready code patterns
- Safety considerations
- Performance optimization

### Progressive Difficulty
- Concepts build logically
- Gentle learning curve
- Challenging advanced topics
- Capstone integration project

## Setup Requirements

### Hardware
- **Computer**: 8GB+ RAM, 50GB disk space
- **Optional**: Robot hardware, sensors (for Module 5)

### Software
- **OS**: Ubuntu 22.04 (recommended) or Docker
- **ROS2**: Humble Hawksbill (LTS)
- **IDE**: VSCode or CLion
- **Build tools**: colcon, CMake

See [00-getting-started/installation-guide.md](00-getting-started/installation-guide.md) for detailed setup.

## Estimated Timeline

| Level | Time Commitment | Duration |
|-------|----------------|----------|
| Part-time (5-10 hrs/week) | Comprehensive | 6-12 months |
| Part-time (5-10 hrs/week) | Fast Track | 3-6 months |
| Full-time (30-40 hrs/week) | Comprehensive | 2-3 months |
| Full-time (30-40 hrs/week) | Fast Track | 1-2 months |

**Note**: Timelines vary based on prior experience and learning pace.

## Success Criteria

By completing this course, you will:
- ✓ Write modern, idiomatic C++ for robotics
- ✓ Build ROS2 nodes, publishers, subscribers, services
- ✓ Create custom messages and action servers
- ✓ Simulate robots in Gazebo
- ✓ Integrate real sensors and actuators
- ✓ Deploy production-ready robotic systems
- ✓ Debug and optimize ROS2 applications
- ✓ Have a portfolio of robotics projects

## Course Standards

### Code Quality
- Well-commented and documented
- Follows ROS2 style guidelines
- Production-ready patterns
- Error handling included

### Lessons
- Clear learning objectives
- Theory with examples
- Visual aids when helpful
- Summary and next steps

### Projects
- Realistic scenarios
- Incremental complexity
- Complete documentation
- Tested and verified

## Getting Started

Ready to begin? Start here:

1. **[Installation Guide](00-getting-started/installation-guide.md)** - Set up your environment
2. **[Development Setup](00-getting-started/dev-environment-setup.md)** - Configure your IDE
3. **[First Commands](00-getting-started/first-commands.md)** - Learn ROS2 CLI
4. **[Module 1](01-cpp-refresher/)** - Begin C++ refresher

## Contributing

Found an error or have a suggestion? This course is designed to evolve:
- Report issues in lesson clarity
- Suggest additional examples
- Share your project variations
- Contribute improvements

## Additional Resources

- **ROS2 Documentation**: https://docs.ros.org/
- **Modern C++ Reference**: https://en.cppreference.com/
- **Gazebo**: https://gazebosim.org/
- **ROS Answers**: https://answers.ros.org/

## License

This course is provided for educational purposes. Code examples are free to use and modify for learning and projects.

---

## Quick Start

```bash
# 1. Clone or download this course
cd ~/
git clone <your-repo-url> ROS2-CPP-Course
cd ROS2-CPP-Course

# 2. Read getting started
cat 00-getting-started/README.md

# 3. Begin learning!
cd 01-cpp-refresher
```

**Happy learning, and welcome to the world of ROS2 robotics!**
