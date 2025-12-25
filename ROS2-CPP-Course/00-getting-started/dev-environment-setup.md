# Development Environment Setup

Set up your IDE and development tools for efficient ROS2 and C++ development.

## Table of Contents

- [IDE Options](#ide-options)
- [VSCode Setup (Recommended)](#vscode-setup-recommended)
- [CLion Setup (Alternative)](#clion-setup-alternative)
- [Essential Command Line Tools](#essential-command-line-tools)
- [Git Configuration](#git-configuration)
- [Optional Tools](#optional-tools)

---

## IDE Options

| IDE | Pros | Cons | Best For |
|-----|------|------|----------|
| **VSCode** | Free, lightweight, great extensions | Less powerful C++ features | Beginners, general use |
| **CLion** | Powerful C++ IDE, excellent debugging | Paid (free for students) | Advanced users, professionals |
| **Qt Creator** | Free, good ROS2 integration | Steeper learning curve | Qt-based projects |
| **Vim/Emacs** | Powerful, customizable | High learning curve | Experienced terminal users |

**Recommendation**: Start with VSCode. It's free, easy to learn, and has excellent ROS2 support.

---

## VSCode Setup (Recommended)

### Step 1: Install VSCode

#### Ubuntu
```bash
# Download and install from Microsoft
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'

sudo apt update
sudo apt install code

# Or snap
sudo snap install --classic code
```

#### Windows
Download from [code.visualstudio.com](https://code.visualstudio.com/)

#### macOS
```bash
brew install --cask visual-studio-code
```

### Step 2: Install Essential Extensions

Open VSCode and install these extensions (Ctrl+Shift+X):

#### C++ Development
```
C/C++ (ms-vscode.cpptools)
C/C++ Extension Pack (ms-vscode.cpptools-extension-pack)
CMake (twxs.cmake)
CMake Tools (ms-vscode.cmake-tools)
```

#### ROS2 Specific
```
ROS (ms-iot.vscode-ros)
URDF (smilerobotics.urdf)
XML Tools (dotjoshjohnson.xml)
```

#### Code Quality
```
clangd (llvm-vs-code-extensions.vscode-clangd)
Better C++ Syntax (jeff-hykin.better-cpp-syntax)
Error Lens (usernamehw.errorlens)
```

#### General Productivity
```
GitLens (eamodio.gitlens)
Markdown All in One (yzhang.markdown-all-in-one)
Todo Tree (gruntfuggly.todo-tree)
Bracket Pair Colorizer 2 (coenraads.bracket-pair-colorizer-2)
```

**Install via command line**:
```bash
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-iot.vscode-ros
code --install-extension llvm-vs-code-extensions.vscode-clangd
```

### Step 3: Configure VSCode for ROS2

Create workspace settings file: `.vscode/settings.json`

```json
{
    "ros.distro": "humble",
    "python.autoComplete.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages"
    ],
    "C_Cpp.default.includePath": [
        "${workspaceFolder}/**",
        "/opt/ros/humble/include/**"
    ],
    "C_Cpp.default.compilerPath": "/usr/bin/g++",
    "C_Cpp.default.cStandard": "c11",
    "C_Cpp.default.cppStandard": "c++17",
    "C_Cpp.default.intelliSenseMode": "linux-gcc-x64",
    "cmake.configureOnOpen": false,
    "files.associations": {
        "*.repos": "yaml",
        "*.world": "xml",
        "*.xacro": "xml"
    },
    "[cpp]": {
        "editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd",
        "editor.formatOnSave": true
    },
    "editor.formatOnSave": true,
    "editor.rulers": [100],
    "files.trimTrailingWhitespace": true
}
```

### Step 4: Create Build Tasks

Create `.vscode/tasks.json`:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "colcon: build",
            "type": "shell",
            "command": "colcon build --symlink-install",
            "problemMatcher": [],
            "group": {
                "kind": "build",
                "isDefault": true
            }
        },
        {
            "label": "colcon: test",
            "type": "shell",
            "command": "colcon test && colcon test-result --verbose",
            "problemMatcher": []
        },
        {
            "label": "colcon: clean",
            "type": "shell",
            "command": "rm -rf build install log"
        },
        {
            "label": "source workspace",
            "type": "shell",
            "command": "source install/setup.bash"
        }
    ]
}
```

### Step 5: Configure Debugging

Create `.vscode/launch.json`:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS: Launch",
            "type": "ros",
            "request": "launch",
            "target": "${workspaceFolder}/install/lib/<package_name>/<executable_name>"
        },
        {
            "name": "C++: Debug",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/install/lib/${input:package}/${input:program}",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ],
    "inputs": [
        {
            "id": "package",
            "type": "promptString",
            "description": "Package name"
        },
        {
            "id": "program",
            "type": "promptString",
            "description": "Program name"
        }
    ]
}
```

### Step 6: Install clangd (Better IntelliSense)

```bash
sudo apt install clangd-12
sudo update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-12 100
```

Create `.clangd` in workspace root:

```yaml
CompileFlags:
  Add:
    - "-std=c++17"
    - "-I/opt/ros/humble/include"
  Remove:
    - "-march=*"
    - "-mtune=*"
```

---

## CLion Setup (Alternative)

### Step 1: Install CLion

Download from [JetBrains](https://www.jetbrains.com/clion/)

**Free for students**: Get education license at jetbrains.com/student

### Step 2: Install ROS2 Plugin

1. Open CLion
2. Go to `File` > `Settings` > `Plugins`
3. Search "ROS2"
4. Install "ROS-Support" plugin
5. Restart CLion

### Step 3: Configure CMake

1. `File` > `Settings` > `Build, Execution, Deployment` > `CMake`
2. Add environment variable:
   ```
   CMAKE_PREFIX_PATH=/opt/ros/humble
   ```

### Step 4: Open ROS2 Workspace

1. `File` > `Open`
2. Select your workspace root directory
3. CLion will auto-detect CMake projects

### Step 5: Configure Run Configurations

1. `Run` > `Edit Configurations`
2. Add new "ROS2 Node" configuration
3. Select package and executable

---

## Essential Command Line Tools

### Install Development Tools

```bash
# Compiler and build tools
sudo apt install -y \
    build-essential \
    cmake \
    g++ \
    gcc \
    gdb \
    make

# Python tools
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-setuptools

# Version control
sudo apt install -y \
    git \
    git-lfs

# Utilities
sudo apt install -y \
    curl \
    wget \
    vim \
    htop \
    tree \
    tmux
```

### Install ROS2 Specific Tools

```bash
# Already installed if you followed installation guide
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete

# Additional useful tools
sudo apt install -y \
    ros-humble-rqt* \
    ros-humble-plotjuggler-ros
```

### Install Code Quality Tools

```bash
# Formatting and linting
sudo apt install -y \
    clang-format \
    clang-tidy \
    cppcheck

# Static analysis
pip3 install cpplint
pip3 install lizard  # Complexity analyzer
```

---

## Git Configuration

### Basic Git Setup

```bash
# Set your identity
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Set default editor
git config --global core.editor "code --wait"  # VSCode
# or
git config --global core.editor "vim"  # Vim

# Useful aliases
git config --global alias.st status
git config --global alias.co checkout
git config --global alias.br branch
git config --global alias.lg "log --graph --oneline --decorate --all"

# Use color in git output
git config --global color.ui true

# Default branch name
git config --global init.defaultBranch main
```

### Create `.gitignore` Template

Create `~/.gitignore_global`:

```bash
# ROS2
build/
install/
log/
.colcon/

# C++
*.o
*.so
*.a
*.exe
*.out

# IDE
.vscode/
.idea/
*.swp
*.swo
*~

# OS
.DS_Store
Thumbs.db

# Python
__pycache__/
*.pyc
*.pyo
*.egg-info/
```

Apply globally:
```bash
git config --global core.excludesfile ~/.gitignore_global
```

---

## Optional Tools

### Terminal Enhancements

#### Terminator (Multi-panel terminal)
```bash
sudo apt install terminator
```

#### Tmux (Terminal multiplexer)
```bash
sudo apt install tmux

# Basic tmux config (~/.tmux.conf)
cat << 'EOF' > ~/.tmux.conf
# Enable mouse
set -g mouse on

# Split panes with | and -
bind | split-window -h
bind - split-window -v

# Vim-like pane navigation
bind h select-pane -L
bind j select-pane -D
bind k select-pane -U
bind l select-pane -R
EOF
```

### Code Analysis Tools

```bash
# Valgrind (memory checker)
sudo apt install valgrind

# Perf (profiler)
sudo apt install linux-tools-common linux-tools-generic

# PlotJuggler (data visualization)
sudo apt install ros-humble-plotjuggler-ros
```

### Documentation Tools

```bash
# Doxygen (code documentation)
sudo apt install doxygen graphviz

# Sphinx (documentation generator)
pip3 install sphinx sphinx-rtd-theme
```

---

## Workspace Template

Create a new ROS2 workspace with this structure:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Create workspace README
cat << 'EOF' > README.md
# ROS2 Workspace

## Build
```bash
colcon build --symlink-install
```

## Source
```bash
source install/setup.bash
```

## Clean
```bash
rm -rf build install log
```
EOF

# Initialize git
git init
```

---

## Keyboard Shortcuts Reference

### VSCode
- **Build**: Ctrl+Shift+B
- **Command Palette**: Ctrl+Shift+P
- **Terminal**: Ctrl+`
- **File Explorer**: Ctrl+Shift+E
- **Search**: Ctrl+Shift+F
- **Go to Definition**: F12
- **Go Back**: Alt+Left
- **Format Document**: Shift+Alt+F

### CLion
- **Build**: Ctrl+F9
- **Run**: Shift+F10
- **Debug**: Shift+F9
- **Find**: Ctrl+Shift+F
- **Go to Declaration**: Ctrl+B
- **Refactor**: Ctrl+Alt+Shift+T

---

## Verification

Test your setup:

### 1. Check Tools
```bash
code --version
git --version
g++ --version
cmake --version
colcon version-check
```

### 2. Test VSCode
```bash
# Open workspace in VSCode
cd ~/ros2_ws
code .
```

### 3. Test Build
```bash
# In VSCode, press Ctrl+Shift+B to build
# Or in terminal:
colcon build
```

---

## Troubleshooting

### VSCode can't find ROS2 headers
**Solution**: Check `c_cpp_properties.json` includes `/opt/ros/humble/include`

### clangd conflicts with C/C++ extension
**Solution**: Disable C/C++ IntelliSense in settings:
```json
{
    "C_Cpp.intelliSenseEngine": "Disabled"
}
```

### Build tasks not working
**Solution**: Ensure you're in workspace root when running tasks

---

## Next Steps

Environment ready? Great!

1. ✓ IDE configured
2. ✓ Tools installed
3. → Continue to [ROS2 Workspace Guide](ros2-workspace-guide.md)
4. → Or check [First Commands](first-commands.md) cheat sheet

---

**Happy coding!** Your development environment is ready for ROS2 development.
