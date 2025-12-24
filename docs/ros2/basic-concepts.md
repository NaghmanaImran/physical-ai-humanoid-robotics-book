---
sidebar_position: 3
---

# ROS2 Basic Concepts

## Nodes

A node is an executable that uses ROS2 to communicate with other nodes. In humanoid robotics, nodes might represent:

- Sensor drivers (IMU, cameras, LiDAR)
- Control algorithms (walking controllers, arm controllers)
- Perception systems (object detection, SLAM)
- Planning modules (path planning, motion planning)
- High-level behaviors (task planning, decision making)

### Creating a Node

Here's a basic example of a ROS2 node in Python:

```python
import rclpy
from rclpy.node import Node

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.get_logger().info('Humanoid Controller node initialized')

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Messages

Topics provide a way for nodes to exchange data through a publish-subscribe model. In humanoid robotics, common topics include:

- `/joint_states`: Current positions, velocities, and efforts of all joints
- `/tf`: Transformations between coordinate frames
- `/imu/data`: Inertial measurement unit data
- `/camera/color/image_raw`: Camera images
- `/cmd_vel`: Velocity commands for base movement

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(String, 'joint_commands', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Joint positions: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointStatePublisher()
    
    try:
        rclpy.spin(joint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_publisher.destroy_node()
        rclpy.shutdown()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            String,
            'joint_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint command: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    joint_subscriber = JointStateSubscriber()
    
    try:
        rclpy.spin(joint_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joint_subscriber.destroy_node()
        rclpy.shutdown()
```

## Services

Services provide a request-response communication pattern. Common services in humanoid robotics:

- `/get_joint_state`: Request current joint positions
- `/set_joint_position`: Set specific joint positions
- `/save_map`: Save a map in SLAM applications
- `/load_map`: Load a previously saved map

### Service Server Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    
    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()
```

## Actions

Actions are used for long-running tasks that require feedback. In humanoid robotics:

- `/move_base`: Navigate to a goal position
- `/pick_object`: Perform a grasping action
- `/walk_to`: Execute walking to a specific location
- `/look_at`: Direct gaze toward a point

### Action Server Example

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
        
        goal_result = Fibonacci.Result()
        goal_result.sequence = feedback_msg.sequence
        goal_handle.succeed()
        
        return goal_result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    
    try:
        rclpy.spin(fibonacci_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        fibonacci_action_server.destroy_node()
        rclpy.shutdown()
```

## Parameters

Parameters allow nodes to be configured at runtime:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('walking_speed', 0.5)
        self.declare_parameter('step_height', 0.05)
        self.declare_parameter('max_torque', 10.0)
        
        # Access parameter values
        self.walking_speed = self.get_parameter('walking_speed').value
        self.step_height = self.get_parameter('step_height').value
        self.max_torque = self.get_parameter('max_torque').value
        
        self.get_logger().info(
            f'Parameters set - Speed: {self.walking_speed}, '
            f'Step height: {self.step_height}, Max torque: {self.max_torque}'
        )

def main(args=None):
    rclpy.init(args=args)
    param_node = ParameterNode()
    
    try:
        rclpy.spin(param_node)
    except KeyboardInterrupt:
        pass
    finally:
        param_node.destroy_node()
        rclpy.shutdown()
```

## Quality of Service (QoS)

QoS settings allow you to control the reliability and durability of communication:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create a QoS profile for real-time critical data
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
    durability=DurabilityPolicy.VOLATILE,    # or TRANSIENT_LOCAL
)

# Use the QoS profile when creating publishers/subscribers
publisher = self.create_publisher(String, 'critical_data', qos_profile)
```

## Launch Files

Launch files allow you to start multiple nodes with a single command:

```xml
<launch>
  <!-- Robot state publisher -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(var robot_description)"/>
  </node>
  
  <!-- Joint state publisher -->
  <node pkg="joint_state_publisher" exec="joint_state_publisher" name="joint_state_publisher"/>
  
  <!-- Humanoid controller -->
  <node pkg="humanoid_controller" exec="controller_node" name="humanoid_controller">
    <param name="walking_speed" value="0.5"/>
    <param name="step_height" value="0.05"/>
  </node>
</launch>
```

Understanding these basic concepts is crucial for developing humanoid robotics applications with ROS2. In the next chapter, we'll explore more advanced concepts like nodes, topics, services, and actions in greater detail.