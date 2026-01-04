# ROS2 Workspace Guide

Learn about ROS2 workspace structure, colcon build system, and best practices for organizing your ROS2 projects.

## Table of Contents

- [What is a Workspace?](#what-is-a-workspace)
- [Workspace Structure](#workspace-structure)
- [Creating Your First Workspace](#creating-your-first-workspace)
- [Understanding Colcon](#understanding-colcon)
- [Building Packages](#building-packages)
- [Sourcing Workspaces](#sourcing-workspaces)
- [Workspace Overlays](#workspace-overlays)
- [Best Practices](#best-practices)

---

## What is a Workspace?

A **ROS2 workspace** is a directory where you develop, build, and install ROS2 packages. Think of it as your project container.

### Key Concepts

- **Source Space** (`src/`): Where your package source code lives
- **Build Space** (`build/`): Intermediate build files (auto-generated)
- **Install Space** (`install/`): Compiled binaries and scripts (auto-generated)
- **Log Space** (`log/`): Build and test logs (auto-generated)

### Why Use Workspaces?

- **Organization**: Keep related packages together
- **Isolation**: Separate projects don't interfere
- **Overlaying**: Extend ROS2 with custom packages
- **Version Control**: Easy to manage with git

---

## Workspace Structure

```
ros2_ws/                    # Workspace root
├── src/                    # SOURCE SPACE (you work here)
│   ├── package_1/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/
│   │   └── src/
│   ├── package_2/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── ...
│   └── ...
├── build/                  # BUILD SPACE (auto-generated)
│   ├── package_1/
│   ├── package_2/
│   └── ...
├── install/                # INSTALL SPACE (auto-generated)
│   ├── setup.bash          # Source this file!
│   ├── lib/
│   ├── share/
│   └── ...
└── log/                    # LOG SPACE (auto-generated)
    └── ...
```

### Important Notes

- **Only edit files in `src/`**
- `build/`, `install/`, `log/` are auto-generated
- Don't commit `build/`, `install/`, `log/` to git
- Each package in `src/` has its own subdirectory

---

## Creating Your First Workspace

### Step 1: Create Directory Structure

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Verify structure
tree -L 1
```

### Step 2: Source ROS2

```bash
# Source ROS2 installation
source /opt/ros/humble/setup.bash

# Verify ROS2 is sourced
echo $ROS_DISTRO
# Should output: humble
```

### Step 3: Build Empty Workspace

```bash
# Build the workspace (even though it's empty)
colcon build

# You should see:
# Starting >>> [no packages found]
# Summary: 0 packages finished
```

### Step 4: Verify Build Outputs

```bash
# Check created directories
ls
# Should show: build  install  log  src
```

### Step 5: Source the Workspace

```bash
# Source the workspace overlay
source install/setup.bash

# Check sourced workspace
echo $COLCON_PREFIX_PATH
# Should include: /home/<user>/ros2_ws/install
```

---

## Understanding Colcon

**Colcon** is the build tool for ROS2. It replaces ROS1's `catkin`.

### Why Colcon?

- Supports multiple build systems (CMake, Python setuptools, etc.)
- Parallel building (faster builds)
- Flexible and extensible
- Better dependency management

### Basic Colcon Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_package

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build up to a specific package
colcon build --packages-up-to my_package

# Clean build
rm -rf build install log
colcon build

# Test packages
colcon test

# Show test results
colcon test-result --verbose
```

### Common Build Options

| Option | Description | Example |
|--------|-------------|---------|
| `--symlink-install` | Use symlinks (no rebuild for Python/launch files) when chaning a file you do not need to run colcon build and rebuild the entirity (taking minutes per change) instead of rerunning just the changed node. | `colcon build --symlink-install` |
| `--packages-select` | Build specific packages (usefull when only altering one section (ex. robot with camera, motor etc. You don't want to run everything when just changing camera package)) | `colcon build --packages-select pkg1 pkg2` |
| `--packages-ignore` | Skip packages | `colcon build --packages-ignore pkg_broken` |
| `--packages-up-to` | Build package and dependencies | `colcon build --packages-up-to my_pkg` |
| `--cmake-args` | Pass arguments to CMake | `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` |
| `--parallel-workers N` | Limit parallel jobs and give certain limits to not freeze entire laptop | `colcon build --parallel-workers 4` |
| `--event-handlers` | Show detailed output (for debugging) | `colcon build --event-handlers console_direct+` |

---

## Building Packages

### Example: Create and Build a Package

```bash
# Navigate to src directory
cd ~/ros2_ws/src

# Create a C++ package
ros2 pkg create --build-type ament_cmake my_cpp_package \
    --dependencies rclcpp std_msgs

# Or create a Python package
ros2 pkg create --build-type ament_python my_py_package \
    --dependencies rclpy std_msgs

# Go back to workspace root
cd ~/ros2_ws

# Build the new package
colcon build --packages-select my_cpp_package

# Source the workspace
source install/setup.bash
```

### Build Output Explained

```
Starting >>> my_package
Finished <<< my_package [2.45s]

Summary: 1 package finished [2.67s]
```

- **Starting >>>**: Build began
- **Finished <<<**: Build completed with time
- **Summary**: Overall statistics

### Build Troubleshooting

```bash
# Clean and rebuild
rm -rf build install log
colcon build

# Verbose output
colcon build --event-handlers console_direct+

# Show package dependencies
colcon list --packages-up-to my_package

# Check for missing dependencies
rosdep install --from-paths src --ignore-src -y
```

---

## Sourcing Workspaces

**Sourcing** makes packages available to your shell.

### Why Source?

- Adds package executables to PATH
- Sets up ROS2 environment variables
- Enables package discovery

### How to Source (showing the order of operations/overlay) (Shows that the foundation is humble (ROS2) and the top layer being your project)

```bash
# (Layer 1/bottom) Source ROS2 installation (always first!)
source /opt/ros/humble/setup.bash

# (Layer 2/top) Then source your workspace
source ~/ros2_ws/install/setup.bash

# Or in one line
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
```

### For Different Shells

```bash
# Bash
source install/setup.bash

# Zsh
source install/setup.zsh

# Fish
source install/setup.fish
```

### Auto-Sourcing (Convenient) (automates the process so you don't have to type source... every time you open a new window)

Add to `~/.bashrc`:

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace (if it exists)
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi
```

Then reload:
```bash
source ~/.bashrc
```

### Verification

```bash
# Check ROS2 is sourced
echo $ROS_DISTRO
# Output: humble

# Check workspace is sourced
echo $COLCON_PREFIX_PATH
# Output: /home/<user>/ros2_ws/install:/opt/ros/humble

# Verify package is visible
ros2 pkg list | grep my_package
```

---

## Workspace Overlays

**Overlays** allow you to extend or modify ROS2 packages without changing the underlying installation.

### Underlay vs Overlay

```
┌─────────────────────────┐
│  Your Workspace         │  ← Overlay (highest priority)
├─────────────────────────┤
│  /opt/ros/humble        │  ← Underlay (base ROS2)
└─────────────────────────┘
```

### How Overlays Work

1. **Underlay**: Base ROS2 installation
2. **Overlay**: Your workspace
3. **Priority**: Overlay packages override underlay

### Example: Override a Package

```bash
# Suppose you want to modify 'turtlesim'

# 1. Create workspace
mkdir -p ~/overlay_ws/src && cd ~/overlay_ws/src

# 2. Clone package source
git clone https://github.com/ros/ros_tutorials.git -b humble

# 3. Modify the package as needed
# Edit files in src/ros_tutorials/turtlesim/...

# 4. Build
cd ~/overlay_ws
colcon build

# 5. Source (order matters!)
source /opt/ros/humble/setup.bash
source ~/overlay_ws/install/setup.bash

# Now your modified turtlesim overrides the system version
```

### Multiple Workspaces

You can have multiple overlays:

```bash
# Source order determines priority
source /opt/ros/humble/setup.bash           # Base
source ~/ws1/install/setup.bash             # Layer 1
source ~/ws2/install/setup.bash             # Layer 2 (highest priority)
```

**Note**: Later sourced workspaces have higher priority.

---

## Best Practices

### 1. Workspace Organization

```
ros2_ws/
├── src/
│   ├── my_project/              # Group related packages
│   │   ├── my_project_core/
│   │   ├── my_project_msgs/
│   │   └── my_project_bringup/
│   └── external_deps/           # Third-party packages
│       └── some_library/
├── .gitignore                   # Ignore build artifacts
└── README.md                    # Workspace documentation
```

### 2. Always Use `--symlink-install`

```bash
# Recommended for development
colcon build --symlink-install

# Why? Changes to Python/launch files don't require rebuild
```

### 3. .gitignore for Workspaces

Create `~/ros2_ws/.gitignore`:

```
# Ignore build artifacts
build/
install/
log/

# Ignore IDE files
.vscode/
.idea/

# Ignore OS files
.DS_Store
```

### 4. Naming Conventions

- **Workspaces**: `<project>_ws` (e.g., `robot_ws`, `ros2_ws`)
- **Packages**: `<project>_<function>` (e.g., `myrobot_navigation`)
- Use underscores, not hyphens

### 5. Dependency Management

```bash
# Install package dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y

# Before building a new workspace
```

### 6. Build Workflow

```bash
# 1. Pull latest code
cd ~/ros2_ws/src/my_package
git pull

# 2. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y

# 3. Build
colcon build --symlink-install

# 4. Source
source install/setup.bash

# 5. Test
ros2 run my_package my_node
```

### 7. Separate Development and Release

```bash
# Development workspace (frequently changing)
~/dev_ws/

# Stable workspace (tested, stable code)
~/stable_ws/

# Release workspace (frozen versions)
~/release_ws/
```

---

## Common Workflows

### Creating a New Package

```bash
# 1. Navigate to src
cd ~/ros2_ws/src

# 2. Create package
ros2 pkg create --build-type ament_cmake my_new_package \
    --dependencies rclcpp std_msgs geometry_msgs

# 3. Build
cd ~/ros2_ws
colcon build --packages-select my_new_package

# 4. Source
source install/setup.bash
```

### Importing External Packages

```bash
# 1. Clone into src
cd ~/ros2_ws/src
git clone https://github.com/user/repo.git

# 2. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y

# 3. Build
colcon build

# 4. Source
source install/setup.bash
```

### Using `.repos` Files

```bash
# Create dependencies.repos
cat << 'EOF' > ~/ros2_ws/dependencies.repos
repositories:
  navigation2:
    type: git
    url: https://github.com/ros-planning/navigation2.git
    version: humble
EOF

# Import using vcstool
cd ~/ros2_ws
vcs import src < dependencies.repos

# Install and build
rosdep install --from-paths src --ignore-src -y
colcon build
```

---

## Troubleshooting

### Build fails with "Package not found"
```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -y
```

### Changes not taking effect
```bash
# Did you source?
source install/setup.bash

# For Python/launch files, rebuild with symlink
colcon build --symlink-install
```

### Multiple workspaces conflict
```bash
# Check sourcing order
echo $COLCON_PREFIX_PATH

# Unsource and re-source in correct order
```

### Out of disk space
```bash
# Clean build artifacts
rm -rf build install log

# Or clean specific package
colcon build --packages-select my_package --cmake-clean-cache
```

---

## Quick Reference

```bash
# Create workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws

# Build workspace
colcon build --symlink-install

# Build specific package
colcon build --packages-select my_pkg

# Source workspace
source install/setup.bash

# Clean workspace
rm -rf build install log

# List packages
colcon list

# Install dependencies
rosdep install --from-paths src --ignore-src -y

# Create package
ros2 pkg create --build-type ament_cmake my_pkg
```

---

## Next Steps

Workspace ready? Excellent!

1. ✓ Understand workspace structure
2. ✓ Know how to build and source
3. → Learn essential commands: [First Commands](first-commands.md)
4. → Start coding: [Module 1: C++ Refresher](../01-cpp-refresher/)

---

**Congratulations!** You now understand ROS2 workspaces and are ready to start building packages!
