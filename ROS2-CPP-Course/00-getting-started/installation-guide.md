# ROS2 Installation Guide

This guide covers installing **ROS2 Humble Hawksbill** (LTS release) on different platforms. Choose the method that best fits your system.

## Quick Platform Selection

| Platform | Recommended Method | Difficulty | Notes |
|----------|-------------------|------------|-------|
| Ubuntu 22.04 | Native Install | Easy | Best performance, most compatible |
| Windows 10/11 | WSL2 + Ubuntu | Medium | Good performance, Windows integration |
| macOS | Docker | Medium | Containerized, some limitations |
| Other Linux | Docker | Medium | Portable, isolated environment |

## Table of Contents

- [Option 1: Ubuntu Native Installation](#option-1-ubuntu-native-installation)
- [Option 2: Windows with WSL2](#option-2-windows-with-wsl2)
- [Option 3: Docker Installation](#option-3-docker-installation)
- [Post-Installation Setup](#post-installation-setup)
- [Verification](#verification)

---

## Option 1: Ubuntu Native Installation

**Requirements**: Ubuntu 22.04 (Jammy Jellyfish)

### Step 1: Set Locale
Ensure UTF-8 locale is set:

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Step 2: Setup Sources

Add ROS2 apt repository:

```bash
# Enable Ubuntu Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS2 Packages

Update apt cache and install:

```bash
# Update package index
sudo apt update

# Upgrade existing packages (optional but recommended)
sudo apt upgrade

# Install ROS2 Humble Desktop (full installation)
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools
```

**Package Options**:
- `ros-humble-desktop`: ROS2, RViz, demos, tutorials (Recommended)
- `ros-humble-ros-base`: Bare-bones, no GUI tools
- `ros-humble-desktop-full`: Everything including Gazebo

### Step 4: Install Additional Tools

```bash
# Install colcon build tool
sudo apt install python3-colcon-common-extensions

# Install rosdep dependency manager
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update

# Install vcstool for repository management
sudo apt install python3-vcstool

# Install argcomplete for command auto-completion
sudo apt install python3-argcomplete
```

### Step 5: Environment Setup

Add ROS2 to your shell environment:

```bash
# Source ROS2 setup file
source /opt/ros/humble/setup.bash

# Add to bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

For zsh users:
```bash
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

**Proceed to [Verification](#verification)**

---

## Option 2: Windows with WSL2

Windows users can run ROS2 through Windows Subsystem for Linux 2 (WSL2).

### Step 1: Enable WSL2

Open PowerShell as Administrator:

```powershell
# Enable WSL
wsl --install

# Or manually:
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```

Restart your computer.

### Step 2: Install Ubuntu 22.04

```powershell
# List available distributions
wsl --list --online

# Install Ubuntu 22.04
wsl --install -d Ubuntu-22.04

# Set as default
wsl --set-default Ubuntu-22.04
```

### Step 3: Launch Ubuntu

1. Open "Ubuntu 22.04" from Start Menu
2. Create username and password when prompted
3. Update packages:

```bash
sudo apt update && sudo apt upgrade -y
```

### Step 4: Install ROS2

Follow the [Ubuntu Native Installation](#option-1-ubuntu-native-installation) steps inside your WSL2 Ubuntu environment.

### Step 5: GUI Support (Optional)

For running RViz and Gazebo, you need GUI support:

#### Windows 11 (Built-in WSLg)
Windows 11 has built-in GUI support. No extra steps needed!

#### Windows 10
Install VcXsrv or X410:

1. Download and install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. Run XLaunch with default settings
3. Add to ~/.bashrc in WSL:

```bash
echo 'export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk "{print \$2}"):0' >> ~/.bashrc
echo 'export LIBGL_ALWAYS_INDIRECT=1' >> ~/.bashrc
source ~/.bashrc
```

### Step 6: Performance Optimization

```bash
# Add to ~/.bashrc for better performance
echo 'export GAZEBO_IP=127.0.0.1' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc
```

**Proceed to [Verification](#verification)**

---

## Option 3: Docker Installation

Use Docker for a portable, isolated ROS2 environment.

### Step 1: Install Docker

#### Ubuntu
```bash
# Install Docker
sudo apt update
sudo apt install docker.io

# Add user to docker group
sudo usermod -aG docker $USER

# Log out and back in, or run:
newgrp docker

# Test Docker
docker --version
```

#### Windows/macOS
Download and install [Docker Desktop](https://www.docker.com/products/docker-desktop)

### Step 2: Pull ROS2 Image

```bash
# Pull official ROS2 Humble image
docker pull osrf/ros:humble-desktop

# Verify image
docker images
```

### Step 3: Create Docker Container

```bash
# Create a container with GUI support (Linux)
docker run -it \
    --name ros2-humble \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/ros2_docker_ws:/root/ros2_ws \
    osrf/ros:humble-desktop \
    bash

# For macOS/Windows, omit GUI options:
docker run -it \
    --name ros2-humble \
    -v ~/ros2_docker_ws:/root/ros2_ws \
    osrf/ros:humble-desktop \
    bash
```

### Step 4: Inside Container

```bash
# Update packages
apt update && apt upgrade -y

# Install development tools
apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    vim

# Initialize rosdep
rosdep update
```

### Step 5: Create Helper Scripts

Create `~/start-ros2.sh`:

```bash
#!/bin/bash
# Start or attach to ROS2 container

if [ "$(docker ps -aq -f name=ros2-humble)" ]; then
    if [ "$(docker ps -q -f name=ros2-humble)" ]; then
        echo "Container is running. Attaching..."
        docker exec -it ros2-humble bash
    else
        echo "Starting existing container..."
        docker start -ai ros2-humble
    fi
else
    echo "Creating new container..."
    docker run -it \
        --name ros2-humble \
        --privileged \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v ~/ros2_docker_ws:/root/ros2_ws \
        osrf/ros:humble-desktop \
        bash
fi
```

Make it executable:
```bash
chmod +x ~/start-ros2.sh
```

**Proceed to [Verification](#verification)**

---

## Post-Installation Setup

### Install Common Dependencies

Regardless of installation method:

```bash
# Install essential build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    wget

# Install common ROS2 packages
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# Install C++ related tools
sudo apt install -y \
    clang \
    clang-format \
    clang-tidy \
    gdb
```

### Configure ROS Domain ID (Optional)

If working with multiple ROS2 systems:

```bash
# Add to ~/.bashrc
echo "export ROS_DOMAIN_ID=<your_id>" >> ~/.bashrc  # 0-101
source ~/.bashrc
```

### Enable Command Auto-completion

```bash
# For bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# For zsh
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh" >> ~/.zshrc

source ~/.bashrc  # or ~/.zshrc
```

---

## Verification

Test your installation with these commands:

### 1. Check ROS2 Version
```bash
ros2 --version
# Expected: ros2 cli version: 0.X.X
```

### 2. Run Demo Nodes

Open two terminals:

**Terminal 1:**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2:**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

You should see messages being published and received.

### 3. Test RViz

```bash
source /opt/ros/humble/setup.bash
rviz2
```

RViz window should open (requires GUI support).

### 4. Test Colcon Build

```bash
mkdir -p ~/test_ws/src
cd ~/test_ws
colcon build
# Should complete successfully
```

### 5. List Available Packages

```bash
ros2 pkg list | head -20
# Should show installed packages
```

---

## Troubleshooting

### Command not found: ros2
**Solution**: Source your ROS2 installation
```bash
source /opt/ros/humble/setup.bash
```

### Permission denied errors
**Solution**: Fix ownership
```bash
sudo chown -R $USER:$USER ~/ros2_ws
```

### GPG key errors during installation
**Solution**: Re-add the key
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### RViz/Gazebo won't start (WSL2)
**Solution**: Check GUI support
- Windows 11: Should work automatically
- Windows 10: Ensure X server (VcXsrv) is running

### Docker container won't start
**Solution**: Check Docker daemon
```bash
sudo systemctl status docker
sudo systemctl start docker
```

For more issues, see [../resources/troubleshooting.md](../resources/troubleshooting.md)

---

## Next Steps

Installation complete? Great!

1. ✓ Verify all tests pass
2. → Continue to [Development Environment Setup](dev-environment-setup.md)
3. → Or jump to [ROS2 Workspace Guide](ros2-workspace-guide.md)

## Additional Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS Answers](https://answers.ros.org/)
- [Ubuntu Installation](https://ubuntu.com/tutorials/install-ubuntu-desktop)

---

**Installation successful?** You're ready to start developing with ROS2!
