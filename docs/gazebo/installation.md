---
sidebar_position: 2
---

# Gazebo Installation and Setup for Humanoid Robotics

## System Requirements

Before installing Gazebo, ensure your system meets the following requirements:

### Minimum Requirements
- **OS**: Ubuntu 20.04/22.04, macOS 10.14+, Windows 10/11 (with WSL2)
- **CPU**: Multi-core processor (4+ cores recommended)
- **RAM**: 8GB minimum, 16GB+ recommended for complex humanoid models
- **GPU**: Dedicated GPU with OpenGL 3.3+ support (NVIDIA/AMD recommended)
- **Storage**: 5GB+ free space for core installation

### Recommended Requirements for Humanoid Robotics
- **CPU**: 8+ cores for real-time simulation of complex models
- **RAM**: 32GB+ for multi-robot simulations
- **GPU**: NVIDIA RTX series or equivalent AMD card with 8GB+ VRAM
- **Storage**: SSD with 20GB+ free space for models and environments

## Installation Methods

### 1. Ubuntu Package Installation (Recommended)

For Ubuntu systems, install Gazebo through the package manager:

```bash
# Update package list
sudo apt update

# Install Gazebo and ROS integration
sudo apt install gazebo libgazebo-dev
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control

# Install additional useful packages
sudo apt install gazebo-utils gazebo-plugin-base
```

### 2. Docker Installation (Isolated Environment)

For isolated development or avoiding system conflicts:

```bash
# Pull the official Gazebo Docker image with ROS2
docker pull osrf/ros:humble-desktop-full-gazebo

# Run with GUI support (Linux)
xhost +local:docker
docker run -it --rm \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device=/dev/dri:/dev/dri \
  --name=gazebo_ros \
  osrf/ros:humble-desktop-full-gazebo

# Test Gazebo
gz sim --version
```

### 3. Building from Source (Latest Features)

For the latest features or development work:

```bash
# Install dependencies
sudo apt update
sudo apt install -y cmake cppcheck curl file g++ git lcov libbenchmark-dev \
  libeigen3-dev libfreeimage-dev libgts-dev libignition-cmake0-dev \
  libignition-modularscripts-dev libignition-tools-dev libjsoncpp-dev \
  liblua5.3-dev liboctomap-dev libogre-1.12-dev libogre-1.12.12v5 \
  libogre-2.2-dev libpcl-dev libprotoc-dev libqt5core5a libqt5gui5 \
  libqt5opengl5 libqt5widgets5 libsdformat13-dev libsdformat13 \
  libtinyxml2-dev libxml2-utils libzip-dev ninja-build pkg-config \
  protobuf-compiler python3-igraph python3-matplotlib python3-nose \
  python3-pip python3-pyqt5.qtopengl python3-setuptools python3-tk \
  python3-venv ruby ruby-dev ruby-nokogiri ruby-ronn uncrustify \
  wget

# Clone the source code
git clone https://github.com/gazebosim/gz-sim.git
cd gz-sim

# Create build directory
mkdir build
cd build

# Configure and build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Install
sudo make install
```

## ROS2 Integration Setup

To use Gazebo with ROS2 for humanoid robotics:

```bash
# Install ROS2-Gazebo integration packages
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros-control ros-humble-gazebo-plugins
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-joint-state-broadcaster ros-humble-velocity-controllers
sudo apt install ros-humble-position-controllers ros-humble-effort-controllers
```

## Environment Configuration

### Setting up Environment Variables

Add the following to your `~/.bashrc` or `~/.zshrc`:

```bash
# Gazebo settings
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models:/usr/share/gazebo-11/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo/models:/usr/share/gazebo-11
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/.gazebo/plugins:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins

# For ROS2 integration
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
```

### Creating a Workspace for Humanoid Models

Set up a workspace specifically for humanoid robotics simulation:

```bash
# Create a workspace
mkdir -p ~/humanoid_sim_ws/src
cd ~/humanoid_sim_ws

# Source ROS2 and Gazebo
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh

# Build the workspace (even if empty)
colcon build

# Source the workspace
source install/setup.bash
```

## Testing the Installation

Verify that Gazebo is properly installed:

```bash
# Test basic Gazebo functionality
gz sim -v

# Launch a simple world
gz sim -r -s -v 4 empty.sdf

# If using the older gazebo command:
gazebo --version

# Test with a simple world (older version)
gazebo --verbose worlds/empty.world
```

## Installing Humanoid-Specific Models and Tools

### Download Common Humanoid Robot Models

```bash
# Create directory for humanoid models
mkdir -p ~/.gazebo/models

# Download commonly used humanoid robot models
# Atlas Robot
git clone https://github.com/RobotLocomotion/models.git ~/.gazebo/models/atlas

# NASA Valkyrie
git clone https://github.com/nasa/val_model.git ~/.gazebo/models/valkyrie

# NASA Robonaut2
git clone https://github.com/ros-simulation/gazebo_ros_demos.git
cp -r gazebo_ros_demos/rrbot_description ~/.gazebo/models/rrbot

# For custom humanoid models, create your own URDF/SDF files
mkdir -p ~/humanoid_sim_ws/src/humanoid_models/models
```

### Install Additional Physics and Simulation Tools

```bash
# Install tools for physics tuning
sudo apt install ros-humble-ros-controllers ros-humble-effort-controllers
sudo apt install ros-humble-position-controllers ros-humble-joint-state-controller

# Install visualization tools
sudo apt install ros-humble-rviz2 ros-humble-joint-state-publisher-gui

# Install robot modeling tools
sudo apt install ros-humble-xacro ros-humble-urdf ros-humble-urdf-tutorial
```

## GPU Configuration for Optimal Performance

### NVIDIA GPU Setup

If using an NVIDIA GPU, ensure proper drivers are installed:

```bash
# Check if NVIDIA GPU is detected
nvidia-smi

# For optimal Gazebo performance, consider installing proprietary drivers
sudo apt install nvidia-driver-470  # or latest version
sudo reboot
```

### Troubleshooting Graphics Issues

If you encounter graphics-related issues:

```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"

# Run Gazebo with software rendering (slower but more compatible)
export LIBGL_ALWAYS_SOFTWARE=1
gazebo

# Or try with different graphics drivers
export __GLX_VENDOR_LIBRARY_NAME=mesa
```

## Verification and First Simulation

Test your setup with a simple humanoid-like robot:

```bash
# Create a simple humanoid model directory
mkdir -p ~/.gazebo/models/simple_humanoid
cd ~/.gazebo/models/simple_humanoid

# Create model.config file
cat > model.config << EOF
<?xml version="1.0"?>
<model>
  <name>simple_humanoid</name>
  <version>1.0</version>
  <sdf version='1.6'>model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>A simple humanoid robot model for testing.</description>
</model>
EOF

# Create a simple SDF model file
cat > model.sdf << EOF
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="simple_humanoid">
    <link name="base_link">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.4</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.4</iyy>
          <iyz>0.0</iyz>
          <izz>0.4</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.3 0.6</size>
          </box>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.3 0.6</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF

# Test the model in Gazebo
gz sim -r -s -v 4 empty.sdf
# Then insert the 'simple_humanoid' model from the GUI
```

## Common Installation Issues and Solutions

### Issue: Gazebo fails to start with graphics error
**Solution**: Try running with software rendering:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
gz sim
```

### Issue: Cannot find Gazebo libraries
**Solution**: Check if Gazebo is properly installed and environment variables are set:
```bash
echo $GAZEBO_MODEL_PATH
pkg-config --modversion gazebo
```

### Issue: ROS2 packages not found
**Solution**: Ensure ROS2 environment is sourced before Gazebo:
```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
```

With Gazebo properly installed and configured, you're ready to explore simulation basics in the next chapter.