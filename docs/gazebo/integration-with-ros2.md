---
sidebar_position: 6
---

# Gazebo Integration with ROS2 for Humanoid Robotics

## Overview of ROS2-Gazebo Integration

The integration between ROS2 and Gazebo enables seamless simulation of humanoid robots with all the tools and capabilities of the ROS2 ecosystem. This integration allows for developing, testing, and validating humanoid robotics applications in a safe, cost-effective virtual environment before deploying on real hardware.

## Core Integration Components

### Gazebo ROS Packages

The primary integration is provided by several key packages:

- **gazebo_ros_pkgs**: Core ROS2-Gazebo integration
- **gazebo_ros_control**: Integration with ROS2 Control framework
- **gazebo_plugins**: Various sensor and actuator plugins for Gazebo

### Installation and Setup

Ensure you have the necessary packages installed:

```bash
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros-control ros-humble-gazebo-plugins
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-joint-state-broadcaster ros-humble-velocity-controllers
```

## Launching Gazebo with ROS2

### Basic Launch File

Create a launch file to start Gazebo with ROS2 integration:

```python
# launch/humanoid_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='empty')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('humanoid_gazebo'),
                'worlds',
                world
            ]),
            'verbose': 'true'
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='Choose one of the world files from `/humanoid_gazebo/worlds`'
        ),
        gazebo
    ])
```

### Robot State Publisher Integration

The robot state publisher bridges the gap between Gazebo and ROS2 TF:

```xml
<!-- In your URDF or launch file -->
<node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(var robot_description)"/>
    <param name="use_sim_time" value="true"/>
</node>
```

## Controlling Robots in Gazebo with ROS2

### ROS2 Control Integration

ROS2 Control provides a standardized interface for controlling robots in simulation:

```xml
<!-- In your URDF -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find humanoid_description)/config/hardware_control.yaml</parameters>
  </plugin>
</gazebo>
```

### Hardware Control Configuration

Create a control configuration file:

```yaml
# config/hardware_control.yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    # Joint trajectory controller
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    # Joint state broadcaster
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

# Joint trajectory controller configuration
joint_trajectory_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint
      - right_hip_joint
      - right_knee_joint
      - right_ankle_joint
      - left_shoulder_joint
      - left_elbow_joint
      - right_shoulder_joint
      - right_elbow_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

# Individual joint controllers
left_leg_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint
    interface_name: position
```

## Sensor Integration

### IMU Sensor

Integrate IMU sensors for balance and orientation:

```xml
<!-- In your URDF -->
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

### Force/Torque Sensors

For humanoid manipulation and balance, add force/torque sensors:

```xml
<!-- In your URDF -->
<gazebo reference="left_foot">
  <sensor name="left_foot_ft_sensor" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <force_torque>
      <frame>sensor</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
  </sensor>
</gazebo>
```

## Launching a Complete Humanoid Simulation

### Complete Launch File

```python
# launch/humanoid_complete.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_description_path = LaunchConfiguration('robot_description_path', 
        default=PathJoinSubstitution([
            FindPackageShare('humanoid_description'),
            'urdf',
            'humanoid.urdf.xacro'
        ]))
    
    # Get the robot description
    robot_description_content = Command([
        'xacro', ' ', LaunchConfiguration('robot_description_path')
    ])
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '1.0'
        ],
        output='screen'
    )
    
    # Load and activate controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )
    
    # Delay loading the position controller until the joint_state_broadcaster is running
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'robot_description_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_description'),
                'urdf',
                'humanoid.urdf.xacro'
            ]),
            description='Absolute path to robot urdf file'
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ])
```

## Running Humanoid Controllers in Simulation

### Example Controller Node

```python
# scripts/humanoid_walking_controller.py
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import math

class HumanoidWalkingController(Node):
    def __init__(self):
        super().__init__('humanoid_walking_controller')
        
        # Publisher for joint trajectories
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Timer for walking pattern generation
        self.timer = self.create_timer(0.1, self.generate_walking_pattern)
        
        # Walking parameters
        self.step_phase = 0.0
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]
        
        self.get_logger().info('Humanoid walking controller initialized')
    
    def generate_walking_pattern(self):
        """Generate walking pattern and publish as joint trajectory"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        
        # Update walking phase
        self.step_phase += 0.02  # Increment phase
        if self.step_phase > 2 * math.pi:
            self.step_phase = 0.0
        
        # Calculate joint positions for walking gait
        # Simplified walking pattern - in practice, this would be more complex
        left_hip = 0.1 * math.sin(self.step_phase)
        left_knee = 0.15 * max(0, math.sin(self.step_phase))
        left_ankle = -0.1 * math.sin(self.step_phase)
        
        right_hip = 0.1 * math.sin(self.step_phase + math.pi)
        right_knee = 0.15 * max(0, math.sin(self.step_phase + math.pi))
        right_ankle = -0.1 * math.sin(self.step_phase + math.pi)
        
        # Set positions
        point.positions = [left_hip, left_knee, left_ankle, 
                          right_hip, right_knee, right_ankle]
        
        # Set time from start (0.1 seconds for this step)
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        
        msg.points = [point]
        
        # Publish trajectory
        self.joint_trajectory_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidWalkingController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simulation Best Practices

### 1. Use Proper Time Handling

Always use simulation time in your nodes when running in Gazebo:

```python
# In your launch files
param name="use_sim_time" value="true"

# In your nodes
use_sim_time = self.declare_parameter('use_sim_time', True).value
```

### 2. Implement Proper Error Handling

Handle cases where simulation might be paused or reset:

```python
def timer_callback(self):
    # Check if simulation is running
    current_time = self.get_clock().now()
    if current_time.nanoseconds == 0:
        # Simulation might be paused or just started
        return
    
    # Continue with normal operation
    self.execute_control_step()
```

### 3. Validate Sensor Data

Verify that sensor data is realistic before using it:

```python
def imu_callback(self, msg):
    # Check for invalid data
    if abs(msg.linear_acceleration.z) < 1.0:
        self.get_logger().warn('Unrealistic IMU data detected')
        return
    
    # Process valid data
    self.process_imu_data(msg)
```

## Debugging Simulation Issues

### Common Issues and Solutions

1. **Robot falls through the ground**:
   - Check collision properties and surface parameters
   - Verify inertial properties are correctly configured
   - Adjust physics parameters (ERP, CFM, stiffness)

2. **Jittery or unstable movement**:
   - Increase solver iterations
   - Reduce time step
   - Check joint limits and dynamics

3. **Controllers not responding**:
   - Verify controller manager is running
   - Check that controllers are properly loaded and activated
   - Confirm correct topic/service names

### Debugging Commands

```bash
# Check running nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Check controller status
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

# Monitor simulation stats
gz topic -e /stats
```

## Performance Optimization

### Optimizing Simulation Performance

For complex humanoid models, consider these optimizations:

1. **Reduce update rates** for non-critical sensors
2. **Use simplified collision meshes**
3. **Optimize physics parameters** for your specific use case
4. **Use multi-threaded executors** for your ROS2 nodes

```python
# Multi-threaded executor example
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    
    controller = HumanoidWalkingController()
    
    # Use multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()
```

The integration of Gazebo with ROS2 provides a powerful platform for developing humanoid robotics applications. This setup allows for safe, cost-effective development and testing before deployment on real hardware.