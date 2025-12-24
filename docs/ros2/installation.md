---
sidebar_position: 2
---

# ROS2 Installation and Setup

## System Requirements

Before installing ROS2, ensure your system meets the following requirements:

### Operating System Support
- **Ubuntu**: 22.04 (Jammy), 20.04 (Focal)
- **Windows**: 10, 11 (with WSL2 recommended)
- **macOS**: Not officially supported for robotics applications
- **Real-time systems**: RT Linux variants

### Hardware Requirements
- **CPU**: Multi-core processor (4+ cores recommended)
- **RAM**: 8GB minimum, 16GB+ recommended
- **Storage**: 10GB+ free space for core installation
- **Network**: Ethernet or Wi-Fi for distributed systems

## Installation Methods

### 1. Debian Packages (Recommended)

The most common and recommended method for installing ROS2 is through Debian packages:

```bash
# Add the ROS2 GPG key
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list and install ROS2
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. Binary Packages

For systems where Debian packages aren't available:

1. Download the appropriate binary package for your platform
2. Extract to a desired location
3. Source the setup script in your shell profile

### 3. Building from Source

For development or when packages aren't available:

```bash
# Install development tools
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions python3-rosdep python3-vcstool

# Create a workspace
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble

# Install dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
```

## Setting Up Your Environment

After installation, you'll need to source the ROS2 environment:

```bash
# For Debian/binary installations
source /opt/ros/humble/setup.bash

# For source builds
source ~/ros2_humble/install/setup.bash
```

To make this permanent, add the source command to your shell profile:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Essential Tools Installation

Install additional tools that will be useful for development:

```bash
# Install development tools
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Install visualization tools
sudo apt install ros-humble-rviz2 ros-humble-ros-base

# Install common packages for robotics
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-moveit ros-humble-moveit-visual-tools
```

## Docker Installation (Alternative)

For isolated development or avoiding system conflicts:

```bash
# Pull the official ROS2 Docker image
docker pull osrf/ros:humble-desktop

# Run with GUI support (Linux)
xhost +local:docker
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix osrf/ros:humble-desktop
```

## Verification

Test your installation by running a simple ROS2 command:

```bash
# Check ROS2 version
ros2 --version

# Run a simple demo
ros2 run demo_nodes_cpp talker
```

In another terminal, run the listener:

```bash
ros2 run demo_nodes_py listener
```

You should see messages passing between the talker and listener nodes.

## Setting up a Workspace

Create a workspace for your humanoid robotics projects:

```bash
# Create workspace directory
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build the workspace (even though it's empty)
colcon build

# Source the workspace
source install/setup.bash
```

## Troubleshooting Common Issues

### 1. Permission Errors
If you encounter permission errors, ensure you're using the correct package manager commands with sudo.

### 2. Network Issues
ROS2 requires proper network configuration for multi-machine setups. Ensure firewall settings allow ROS2 communication.

### 3. Library Conflicts
If you have multiple ROS versions installed, make sure you're sourcing the correct setup file.

## Next Steps

With ROS2 installed, you're ready to explore the basic concepts in the next chapter. The installation provides you with all the core tools needed to develop humanoid robotics applications.