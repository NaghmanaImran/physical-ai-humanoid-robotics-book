---
sidebar_position: 5
---

# ROS2 Packages and Workspaces

## Understanding ROS2 Packages

A package is the fundamental unit of code organization in ROS2. It contains nodes, libraries, configuration files, and other resources needed for a specific functionality.

### Package Structure

A typical ROS2 package follows this structure:

```
my_humanoid_package/
├── CMakeLists.txt          # Build instructions for C++
├── package.xml             # Package metadata
├── src/                    # Source code (C++)
│   ├── controller.cpp
│   └── sensor_processor.cpp
├── scripts/                # Python scripts
│   ├── joint_publisher.py
│   └── balance_controller.py
├── launch/                 # Launch files
│   ├── robot.launch.py
│   └── simulation.launch.py
├── config/                 # Configuration files
│   ├── controllers.yaml
│   └── sensors.yaml
├── msg/                    # Custom message definitions
│   ├── HumanoidJointState.msg
│   └── BalanceCommand.msg
├── srv/                    # Custom service definitions
│   └── WalkToGoal.srv
├── action/                 # Custom action definitions
│   └── Manipulate.action
├── test/                   # Unit tests
│   ├── test_controller.py
│   └── test_sensor_processor.cpp
└── include/my_humanoid_package/  # Header files (C++)
    ├── controller.hpp
    └── sensor_processor.hpp
```

### Creating a Package

To create a new package for your humanoid robotics project:

```bash
# In your workspace source directory
cd ~/humanoid_ws/src

# Create a C++ package
ros2 pkg create --build-type ament_cmake my_humanoid_controller --dependencies rclcpp std_msgs geometry_msgs sensor_msgs

# Create a Python package
ros2 pkg create --build-type ament_python py_humanoid_bringup --dependencies rclpy std_msgs geometry_msgs
```

### Package.xml File

The `package.xml` file contains metadata about your package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_humanoid_controller</name>
  <version>0.0.1</version>
  <description>Humanoid robot controller package</description>
  <maintainer email="developer@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>control_msgs</depend>
  <depend>trajectory_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt File

The `CMakeLists.txt` file contains build instructions for C++ packages:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_humanoid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages/services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HumanoidJointState.msg"
  "srv/BalanceCommand.srv"
  "action/Manipulate.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Create executable
add_executable(controller_node src/controller.cpp)
ament_target_dependencies(controller_node 
  rclcpp std_msgs geometry_msgs sensor_msgs control_msgs trajectory_msgs)

# Install targets
install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME})

# Install other directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

## Workspaces

A workspace is a directory containing one or more packages that you want to build together.

### Creating a Workspace

```bash
# Create workspace directory
mkdir -p ~/humanoid_ws/src

# Navigate to workspace
cd ~/humanoid_ws

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build the workspace (even if empty)
colcon build

# Source the workspace
source install/setup.bash
```

### Building Packages

```bash
# Build all packages in workspace
colcon build

# Build a specific package
colcon build --packages-select my_humanoid_controller

# Build with verbose output
colcon build --event-handlers console_direct+

# Build with symlinks (faster rebuilds)
colcon build --symlink-install

# Build with specific build type (debug, release, etc.)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Workspace Overlaying

ROS2 allows you to overlay workspaces, where packages in newer workspaces take precedence:

```bash
# Source base ROS2 installation
source /opt/ros/humble/setup.bash

# Source your humanoid workspace
source ~/humanoid_ws/install/setup.bash

# Now you can create another workspace that builds on top
mkdir -p ~/advanced_humanoid_ws/src
cd ~/advanced_humanoid_ws
colcon build
source install/setup.bash
```

## Package Dependencies

### Managing Dependencies

Dependencies are specified in `package.xml` and resolved using `rosdep`:

```bash
# Install dependencies for all packages in workspace
rosdep install --from-paths src --ignore-src -r -y

# Install dependencies for a specific package
rosdep install --from-paths src/my_humanoid_controller --ignore-src -r -y
```

### Common Dependencies for Humanoid Robotics

```xml
<depend>rclcpp</depend>                    <!-- C++ client library -->
<depend>rclpy</depend>                     <!-- Python client library -->
<depend>std_msgs</depend>                  <!-- Standard message types -->
<depend>geometry_msgs</depend>             <!-- Geometry-related messages -->
<depend>sensor_msgs</depend>               <!-- Sensor message types -->
<depend>nav_msgs</depend>                  <!-- Navigation messages -->
<depend>tf2</depend>                       <!-- Transform library -->
<depend>tf2_ros</depend>                   <!-- TF2 ROS integration -->
<depend>control_msgs</depend>              <!-- Control-related messages -->
<depend>trajectory_msgs</depend>           <!-- Trajectory messages -->
<depend>builtin_interfaces</depend>        <!-- Built-in message types -->
<depend>message_filters</depend>           <!-- Message filtering -->
<depend>cv_bridge</depend>                 <!-- OpenCV bridge -->
<depend>image_transport</depend>           <!-- Image transport -->
<depend>robot_state_publisher</depend>     <!-- Robot state publishing -->
<depend>joint_state_publisher</depend>     <!-- Joint state publishing -->
<depend>gazebo_ros_pkgs</depend>           <!-- Gazebo integration -->
<depend>moveit_core</depend>               <!-- Motion planning core -->
<depend>moveit_ros_planning_interface</depend> <!-- Motion planning interface -->
```

## Launch Files

Launch files allow you to start multiple nodes with a single command:

### Python Launch Files

```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # Robot state publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': open('/path/to/robot.urdf').read()
            }]),
        
        # Joint state publisher node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]),
        
        # Humanoid controller node
        Node(
            package='my_humanoid_controller',
            executable='controller_node',
            name='humanoid_controller',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'walking_speed': 0.5,
                'step_height': 0.05
            }],
            remappings=[
                ('/joint_states', '/robot/joint_states'),
                ('/cmd_vel', '/robot/cmd_vel')
            ])
    ])
```

### YAML Launch Files (Alternative)

```yaml
# launch/robot.yaml
launch:
  - node:
      pkg: "robot_state_publisher"
      exec: "robot_state_publisher"
      name: "robot_state_publisher"
      parameters:
        - use_sim_time: false
        - robot_description: "$(file /path/to/robot.urdf)"
  
  - node:
      pkg: "my_humanoid_controller"
      exec: "controller_node"
      name: "humanoid_controller"
      parameters:
        - use_sim_time: false
        - walking_speed: 0.5
        - step_height: 0.05
```

## Configuration Files

Configuration files in ROS2 are typically in YAML format:

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

# Joint trajectory controller
joint_trajectory_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint
      - right_hip_joint
      - right_knee_joint
      - right_ankle_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

# Balance controller
balance_controller:
  ros__parameters:
    kp: 10.0
    ki: 0.1
    kd: 0.5
    max_torque: 50.0
    com_threshold: 0.02
```

## Managing Multiple Packages

### Package Groups

For complex humanoid robotics projects, you might want to create metapackages that group related functionality:

```xml
<!-- humanoid_bringup/package.xml -->
<?xml version="1.0"?>
<package format="3">
  <name>humanoid_bringup</name>
  <version>0.0.1</version>
  <description>Metapackage for humanoid robot bringup</description>
  <maintainer email="developer@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <exec_depend>my_humanoid_controller</exec_depend>
  <exec_depend>humanoid_description</exec_depend>
  <exec_depend>humanoid_gazebo</exec_depend>
  <exec_depend>humanoid_navigation</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Build Ordering

Use `colcon` with specific options to control build order:

```bash
# Build in dependency order
colcon build --continue-on-error

# Build packages in parallel (default is CPU cores - 2)
colcon build --parallel-workers 4

# Build with specific packages first
colcon build --packages-up-to my_humanoid_controller
```

## Best Practices

1. **Modular Design**: Create focused packages that do one thing well
2. **Consistent Naming**: Use descriptive names that indicate package purpose
3. **Documentation**: Include README files and API documentation
4. **Testing**: Include unit and integration tests
5. **Version Control**: Use Git for source code management
6. **CI/CD**: Set up continuous integration for your packages
7. **Reusability**: Design packages to be reusable across different robots

Understanding packages and workspaces is crucial for organizing your humanoid robotics codebase effectively. In the next chapter, we'll look at practical examples of ROS2 in humanoid robotics applications.