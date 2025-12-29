# Gazebo Installation and Configuration

## Overview

Gazebo is a physics-based simulation environment that enables accurate and efficient testing of robotics algorithms, designs, and scenarios. This chapter provides detailed instructions for installing and configuring Gazebo with ROS2 integration, which is essential for the digital twin module.

## System Requirements

Before installing Gazebo, ensure your system meets the following requirements:

### Minimum Requirements
- **Operating System**: Ubuntu 20.04 LTS or 22.04 LTS (recommended)
- **Processor**: Multi-core processor (Intel i5 or AMD equivalent)
- **RAM**: 8 GB minimum, 16 GB recommended
- **Graphics**: OpenGL 2.1 compatible GPU with dedicated VRAM
- **Storage**: 10 GB free space
- **Network**: Internet connection for installation

### Recommended Requirements
- **Operating System**: Ubuntu 22.04 LTS
- **Processor**: Multi-core processor (Intel i7 or AMD Ryzen equivalent)
- **RAM**: 16 GB or more
- **Graphics**: Dedicated GPU with 4GB+ VRAM (NVIDIA/AMD recommended)
- **Storage**: SSD with 20 GB free space
- **Network**: Stable internet connection

## Installation Methods

### Method 1: Binary Installation (Recommended)

This method installs pre-compiled binaries and is the fastest way to get started.

1. **Update your system packages**:
   ```bash
   sudo apt update
   ```

2. **Install Gazebo using apt**:
   ```bash
   sudo apt install gazebo
   ```

3. **For ROS2 integration, install the ROS2 Gazebo packages**:
   ```bash
   # First, ensure ROS2 is installed (Humble Hawksbill or later recommended)
   sudo apt install ros-humble-gazebo-*
   ```

4. **Install additional dependencies**:
   ```bash
   sudo apt install gazebo-plugin-base libgazebo-dev
   ```

### Method 2: ROS2 Ecosystem Installation

If you're using ROS2, install Gazebo through the ROS2 ecosystem:

1. **Install ROS2 Humble Hawksbill** (if not already installed):
   ```bash
   # Add ROS2 repository
   sudo apt update && sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   
   # Add ROS2 GPG key
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   
   # Add ROS2 repository to apt sources
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   
   # Install ROS2 packages
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

2. **Install Gazebo with ROS2 integration**:
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control
   ```

### Method 3: Building from Source (Advanced Users)

For the latest features or development work:

1. **Clone the Gazebo repositories**:
   ```bash
   # Create a workspace
   mkdir -p ~/gazebo_ws/src
   cd ~/gazebo_ws/src
   
   # Clone Gazebo repositories
   git clone https://github.com/gazebo/gz-sim.git
   git clone https://github.com/gazebo/gz-fuel-tools.git
   git clone https://github.com/gazebo/gz-gui.git
   git clone https://github.com/gazebo/gz-physics.git
   git clone https://github.com/gazebo/gz-sensors.git
   git clone https://github.com/gazebo/gz-transport.git
   git clone https://github.com/gazebo/gz-math.git
   git clone https://github.com/gazebo/gz-common.git
   git clone https://github.com/gazebo/gz-msgs.git
   git clone https://github.com/gazebo/gz-tools.git
   ```

2. **Install build dependencies**:
   ```bash
   sudo apt update
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**:
   ```bash
   cd ~/gazebo_ws
   colcon build --merge-install
   ```

## Verification Steps

After installation, verify that Gazebo is properly installed:

1. **Launch Gazebo**:
   ```bash
   gazebo
   ```
   You should see the Gazebo interface with a default environment.

2. **Check Gazebo version**:
   ```bash
   gazebo --version
   ```

3. **Test with a simple world**:
   ```bash
   gazebo --verbose worlds/empty.world
   ```

4. **If using ROS2, test the integration**:
   ```bash
   # Source ROS2
   source /opt/ros/humble/setup.bash
   
   # Launch Gazebo with ROS2 bridge
   ros2 launch gazebo_ros gazebo.launch.py
   ```

## Common Installation Issues and Troubleshooting

### Issue 1: Graphics Driver Problems
**Symptoms**: Gazebo fails to start or crashes immediately
**Solutions**:
- Update your graphics drivers
- For NVIDIA cards: `sudo apt install nvidia-driver-XXX` (replace XXX with appropriate version)
- For AMD cards: `sudo apt install mesa-vulkan-drivers xserver-xorg-video-amdgpu`
- For Intel integrated graphics: `sudo apt install mesa-vulkan-drivers`

### Issue 2: Missing Dependencies
**Symptoms**: Error messages about missing libraries
**Solutions**:
- Run `sudo apt update && sudo apt upgrade`
- Install missing packages as indicated in error messages
- For ROS2 integration: `sudo apt install ros-humble-gazebo-*`

### Issue 3: Permission Errors
**Symptoms**: Cannot create or access Gazebo directories
**Solutions**:
- Check that your user is in the correct groups: `groups $USER`
- Add to dialout group if needed: `sudo usermod -a -G dialout $USER`
- Log out and log back in for changes to take effect

### Issue 4: Performance Problems
**Symptoms**: Slow simulation, low frame rates
**Solutions**:
- Reduce visual quality in Gazebo settings
- Close other applications to free up resources
- Ensure you're using a dedicated GPU rather than integrated graphics
- Check that hardware acceleration is enabled

## Configuration

### Environment Variables

Set up environment variables for optimal Gazebo performance:

1. **Add to your ~/.bashrc file**:
   ```bash
   # Gazebo settings
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models
   export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo/worlds
   export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/.gazebo/plugins
   ```

2. **Apply the changes**:
   ```bash
   source ~/.bashrc
   ```

### Gazebo Configuration File

Create or modify the Gazebo configuration file at `~/.gazebo/config`:

```xml
<gazebo>
  <cache_location>~/.gazebo/cache</cache_location>
  <http_proxy></http_proxy>
  <https_proxy></https_proxy>
  <plugins_path>~/.gazebo/plugins</plugins_path>
  <uri_path>~/.gazebo/models</uri_path>
  <uri_path>~/.gazebo/worlds</uri_path>
  <server_config>~/.gazebo/server.config</server_config>
  <gui_config>~/.gazebo/gui.config</gui_config>
</gazebo>
```

## ROS2 Integration Setup

To properly integrate Gazebo with ROS2:

1. **Source ROS2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Create a workspace for Gazebo-ROS projects**:
   ```bash
   mkdir -p ~/gazebo_ros_ws/src
   cd ~/gazebo_ros_ws
   colcon build --packages-select gazebo_ros_pkgs
   source install/setup.bash
   ```

3. **Test ROS2-Gazebo integration**:
   ```bash
   ros2 launch gazebo_ros empty_world.launch.py
   ```

## Testing Your Installation

Create a simple test to verify everything works:

1. **Create a test directory**:
   ```bash
   mkdir ~/gazebo_test
   cd ~/gazebo_test
   ```

2. **Create a simple world file (test.world)**:
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.6">
     <world name="test_world">
       <include>
         <uri>model://ground_plane</uri>
       </include>
       <include>
         <uri>model://sun</uri>
       </include>
       <model name="box">
         <pose>0 0 0.5 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>1 1 1</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>1 1 1</size>
               </box>
             </geometry>
           </visual>
         </link>
       </model>
     </world>
   </sdf>
   ```

3. **Run the test world**:
   ```bash
   gazebo test.world
   ```

If you see a box on a ground plane with lighting, your installation is successful!

## Next Steps

Now that Gazebo is installed and configured, continue to the next chapter to learn about robot modeling in Gazebo. You'll learn how to create accurate 3D models of robots that can be simulated in the environment you've just set up.

## Diagram Placeholders

[Image: Gazebo Installation Process diagram showing the Gazebo installation steps]

[Image: System Requirements diagram showing system requirements for Gazebo]