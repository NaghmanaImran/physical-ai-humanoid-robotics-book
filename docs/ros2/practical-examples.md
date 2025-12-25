---
sidebar_position: 6
---

# ROS2 Practical Examples for Humanoid Robotics

## Humanoid Robot State Publisher

The robot state publisher is essential for humanoid robotics as it publishes the transformations between different links of the robot:

```python
# scripts/humanoid_state_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class HumanoidStatePublisher(Node):
    def __init__(self):
        super().__init__('humanoid_state_publisher')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing transforms
        self.timer = self.create_timer(0.02, self.publish_transforms)  # 50Hz
        
        # Initialize joint positions
        self.joint_positions = {}
        
    def joint_callback(self, msg):
        """Update joint positions from joint state message"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
    
    def publish_transforms(self):
        """Calculate and publish transforms for all robot links"""
        # Base link to world (for this example, assume base is at origin)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.8  # Height of robot base
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        # Calculate and publish transforms for leg joints
        self.publish_leg_transforms('left', self.joint_positions.get('left_hip_joint', 0.0), 
                                   self.joint_positions.get('left_knee_joint', 0.0), 
                                   self.joint_positions.get('left_ankle_joint', 0.0))
        self.publish_leg_transforms('right', self.joint_positions.get('right_hip_joint', 0.0), 
                                   self.joint_positions.get('right_knee_joint', 0.0), 
                                   self.joint_positions.get('right_ankle_joint', 0.0))
        
        # Calculate and publish transforms for arm joints
        self.publish_arm_transforms('left', self.joint_positions.get('left_shoulder_joint', 0.0), 
                                   self.joint_positions.get('left_elbow_joint', 0.0))
        self.publish_arm_transforms('right', self.joint_positions.get('right_shoulder_joint', 0.0), 
                                   self.joint_positions.get('right_elbow_joint', 0.0))
    
    def publish_leg_transforms(self, side, hip_pos, knee_pos, ankle_pos):
        """Publish transforms for leg joints"""
        # Hip joint
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = f'{side}_hip_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.1 if side == 'left' else -0.1  # Offset for left/right
        t.transform.translation.z = 0.0
        # Convert joint position to quaternion rotation (simplified)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(hip_pos / 2.0)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(hip_pos / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # Knee joint
        t.header.frame_id = f'{side}_hip_link'
        t.child_frame_id = f'{side}_knee_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = -0.3  # Thigh length
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(knee_pos / 2.0)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(knee_pos / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # Ankle joint
        t.header.frame_id = f'{side}_knee_link'
        t.child_frame_id = f'{side}_ankle_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = -0.3  # Shin length
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(ankle_pos / 2.0)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(ankle_pos / 2.0)
        self.tf_broadcaster.sendTransform(t)
    
    def publish_arm_transforms(self, side, shoulder_pos, elbow_pos):
        """Publish transforms for arm joints"""
        # Shoulder joint
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = f'{side}_shoulder_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.15 if side == 'left' else -0.15
        t.transform.translation.z = 0.2  # Shoulder height
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(shoulder_pos / 2.0)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(shoulder_pos / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # Elbow joint
        t.header.frame_id = f'{side}_shoulder_link'
        t.child_frame_id = f'{side}_elbow_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = -0.25  # Upper arm length
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(elbow_pos / 2.0)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(elbow_pos / 2.0)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Walking Controller for Humanoid Robot

A basic walking controller implementation:

```python
# scripts/walking_controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')
        
        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/joint_commands', 10)
        
        # Subscriber for velocity commands
        self.vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.velocity_callback, 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        
        # Walking parameters
        self.step_length = 0.3  # meters
        self.step_height = 0.05  # meters
        self.step_duration = 1.0  # seconds
        self.current_phase = 0.0  # 0.0 to 1.0
        self.is_walking = False
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Initialize joint positions
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]
        self.joint_positions = [0.0] * len(self.joint_names)
        
        self.get_logger().info('Walking controller initialized')
    
    def velocity_callback(self, msg):
        """Handle velocity commands"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
        # Determine if we should be walking
        self.is_walking = abs(self.linear_vel) > 0.01 or abs(self.angular_vel) > 0.01
    
    def control_loop(self):
        """Main control loop"""
        if self.is_walking:
            # Update walking phase
            self.current_phase += 0.02 / self.step_duration  # Increment based on timer
            if self.current_phase > 1.0:
                self.current_phase -= 1.0  # Wrap around
            
            # Calculate joint positions for walking pattern
            self.calculate_walking_pattern()
        
        # Publish joint commands
        self.publish_joint_commands()
    
    def calculate_walking_pattern(self):
        """Calculate joint positions for walking gait"""
        # Simplified walking pattern - in a real implementation, this would be more complex
        phase = self.current_phase * 2 * math.pi  # Convert to radians
        
        # Left leg pattern
        left_hip = 0.1 * math.sin(phase)
        left_knee = 0.15 * max(0, math.sin(phase))  # Only positive for knee
        left_ankle = -0.1 * math.sin(phase)
        
        # Right leg pattern (opposite phase)
        right_hip = 0.1 * math.sin(phase + math.pi)
        right_knee = 0.15 * max(0, math.sin(phase + math.pi))  # Only positive for knee
        right_ankle = -0.1 * math.sin(phase + math.pi)
        
        # Adjust for forward velocity
        speed_factor = abs(self.linear_vel) / 0.5  # Normalize to 0.5 m/s max
        left_hip *= speed_factor
        right_hip *= speed_factor
        
        # Store calculated positions
        for i, name in enumerate(self.joint_names):
            if name == 'left_hip_joint':
                self.joint_positions[i] = left_hip
            elif name == 'left_knee_joint':
                self.joint_positions[i] = left_knee
            elif name == 'left_ankle_joint':
                self.joint_positions[i] = left_ankle
            elif name == 'right_hip_joint':
                self.joint_positions[i] = right_hip
            elif name == 'right_knee_joint':
                self.joint_positions[i] = right_knee
            elif name == 'right_ankle_joint':
                self.joint_positions[i] = right_ankle
            # Arms could be used for balance, but simplified here
    
    def publish_joint_commands(self):
        """Publish calculated joint positions"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_names)  # For simplicity
        msg.effort = [0.0] * len(self.joint_names)    # For simplicity
        
        self.joint_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WalkingController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Balance Controller

A controller to maintain the robot's balance:

```python
# scripts/balance_controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import numpy as np
import math

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Publisher for balance corrections
        self.balance_cmd_pub = self.create_publisher(
            JointState, '/balance_corrections', 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.01, self.balance_control_loop)  # 100Hz
        
        # PID controller parameters
        self.kp = 50.0  # Proportional gain
        self.ki = 0.1   # Integral gain
        self.kd = 0.5   # Derivative gain
        
        # PID error terms
        self.prev_error = {'roll': 0.0, 'pitch': 0.0}
        self.integral_error = {'roll': 0.0, 'pitch': 0.0}
        
        # Robot state
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.angular_velocity = Vector3()
        self.linear_acceleration = Vector3()
        
        # Target balance state
        self.target_roll = 0.0
        self.target_pitch = 0.0
        
        # Joint information
        self.joint_names = []
        self.joint_positions = []
        self.joint_velocities = []
        self.joint_efforts = []
        
        self.get_logger().info('Balance controller initialized')
    
    def imu_callback(self, msg):
        """Process IMU data to determine robot orientation"""
        # Convert quaternion to Euler angles
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            self.pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Store angular velocity and linear acceleration
        self.angular_velocity = msg.angular_velocity
        self.linear_acceleration = msg.linear_acceleration
    
    def joint_state_callback(self, msg):
        """Update joint state information"""
        self.joint_names = msg.name
        self.joint_positions = list(msg.position) if len(msg.position) > 0 else self.joint_positions
        self.joint_velocities = list(msg.velocity) if len(msg.velocity) > 0 else self.joint_velocities
        self.joint_efforts = list(msg.effort) if len(msg.effort) > 0 else self.joint_efforts
    
    def balance_control_loop(self):
        """Main balance control loop using PID"""
        # Calculate errors
        roll_error = self.target_roll - self.roll
        pitch_error = self.target_pitch - self.pitch
        
        # Update integral terms
        self.integral_error['roll'] += roll_error * 0.01  # dt = 0.01s
        self.integral_error['pitch'] += pitch_error * 0.01
        
        # Calculate derivative terms
        derivative_roll = (roll_error - self.prev_error['roll']) / 0.01
        derivative_pitch = (pitch_error - self.prev_error['pitch']) / 0.01
        
        # Calculate PID outputs
        roll_correction = (
            self.kp * roll_error + 
            self.ki * self.integral_error['roll'] + 
            self.kd * derivative_roll
        )
        
        pitch_correction = (
            self.kp * pitch_error + 
            self.ki * self.integral_error['pitch'] + 
            self.kd * derivative_pitch
        )
        
        # Store current errors for next iteration
        self.prev_error['roll'] = roll_error
        self.prev_error['pitch'] = pitch_error
        
        # Apply limits to corrections
        roll_correction = max(min(roll_correction, 0.2), -0.2)  # Limit to ±0.2 rad
        pitch_correction = max(min(pitch_correction, 0.2), -0.2)  # Limit to ±0.2 rad
        
        # Generate balance correction commands
        self.publish_balance_corrections(roll_correction, pitch_correction)
    
    def publish_balance_corrections(self, roll_corr, pitch_corr):
        """Publish balance correction commands"""
        # In a real implementation, this would calculate specific joint adjustments
        # For this example, we'll create a simplified correction
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names.copy()
        msg.position = self.joint_positions.copy()
        msg.velocity = [0.0] * len(self.joint_positions)
        msg.effort = [0.0] * len(self.joint_positions)
        
        # Apply corrections to ankle joints for balance
        for i, name in enumerate(msg.name):
            if 'ankle' in name:
                # Apply roll and pitch corrections to ankle joints
                if 'left' in name and name.endswith('roll'):
                    msg.position[i] += roll_corr * 0.5  # Scale factor
                elif 'right' in name and name.endswith('roll'):
                    msg.position[i] -= roll_corr * 0.5  # Opposite for right foot
                elif 'left' in name and name.endswith('pitch'):
                    msg.position[i] += pitch_corr * 0.5
                elif 'right' in name and name.endswith('pitch'):
                    msg.position[i] -= pitch_corr * 0.5
        
        self.balance_cmd_pub.publish(msg)
        
        # Log balance state
        self.get_logger().debug(
            f'Balance - Roll: {math.degrees(self.roll):.2f}°, '
            f'Pitch: {math.degrees(self.pitch):.2f}°, '
            f'Corrections - Roll: {math.degrees(roll_corr):.2f}°, '
            f'Pitch: {math.degrees(pitch_corr):.2f}°'
        )

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File for Humanoid Demo

```python
# launch/humanoid_demo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': open('/path/to/humanoid.urdf').read()
            }]
        ),
        
        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Humanoid state publisher (custom)
        Node(
            package='my_humanoid_package',
            executable='humanoid_state_publisher',
            name='humanoid_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Walking controller
        Node(
            package='my_humanoid_package',
            executable='walking_controller',
            name='walking_controller',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Balance controller
        Node(
            package='my_humanoid_package',
            executable='balance_controller',
            name='balance_controller',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Teleoperation node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            prefix='xterm -e',  # Opens in a new terminal window
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('/cmd_vel', '/humanoid/cmd_vel')
            ]
        )
    ])
```

## Running the Example

To run these examples:

1. Build your workspace:
```bash
cd ~/humanoid_ws
colcon build --packages-select my_humanoid_package
source install/setup.bash
```

2. Run the launch file:
```bash
ros2 launch my_humanoid_package humanoid_demo.launch.py
```

3. In another terminal, teleoperate the robot:
```bash
# Use teleop_twist_keyboard to send velocity commands
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Integration with Gazebo Simulation

To integrate with Gazebo simulation, you'll need to set up the robot description with appropriate plugins:

```xml
<!-- In your robot URDF -->
<link name="base_link">
  <inertial>
    <mass value="10.0" />
    <origin xyz="0 0 0.5" />
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
  </inertial>
  
  <visual>
    <origin xyz="0 0 0.5" rpy="0 0 0" />
    <geometry>
      <box size="0.5 0.3 1.0" />
    </geometry>
  </visual>
  
  <collision>
    <origin xyz="0 0 0.5" rpy="0 0 0" />
    <geometry>
      <box size="0.5 0.3 1.0" />
    </geometry>
  </collision>
</link>

<!-- Joint definitions -->
<joint name="left_hip_joint" type="revolute">
  <parent link="base_link" />
  <child link="left_hip_link" />
  <origin xyz="0 0.1 0" rpy="0 0 0" />
  <axis xyz="0 1 0" />
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0" />
</joint>

<!-- Gazebo plugin for ros2_control -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_humanoid_package)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

These practical examples demonstrate how ROS2 can be used to implement essential humanoid robotics functionality. In the next module, we'll explore Gazebo simulation in detail.