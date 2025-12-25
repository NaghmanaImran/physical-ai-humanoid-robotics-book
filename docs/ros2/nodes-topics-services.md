---
sidebar_position: 4
---

# ROS2 Nodes, Topics, Services, and Actions

## Nodes in Depth

Nodes are the fundamental building blocks of ROS2 applications. In humanoid robotics, nodes often represent specific components or capabilities of the robot.

### Node Lifecycle

ROS2 introduces a lifecycle concept that provides more control over node states:

```python
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

class HumanoidLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('humanoid_lifecycle_node')
        self.get_logger().info('Node created in unconfigured state')

    def on_configure(self, state):
        self.get_logger().info('Configuring node')
        # Initialize resources
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating node')
        # Start processes
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating node')
        # Pause processes
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up node')
        # Release resources
        return TransitionCallbackReturn.SUCCESS
```

### Node Composition

For better performance and reduced communication overhead, ROS2 allows multiple nodes to be composed into a single process:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode

class JointStateNode(Node):
    def __init__(self):
        super().__init__('joint_state_node')
        # Implementation for joint state management

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        # Implementation for IMU data processing

class CompositeHumanoidNode(Node):
    def __init__(self):
        super().__init__('composite_humanoid_node')
        
        # Create nodes to compose
        self.joint_state_node = JointStateNode()
        self.imu_node = IMUNode()
        
        # Create executor and add nodes
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.joint_state_node)
        self.executor.add_node(self.imu_node)
        
        # Spin executor in a separate thread
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.start()
```

## Advanced Topic Communication

### Custom Message Types

For humanoid robotics applications, you'll often need custom message types:

```python
# In your package's msg directory, create HumanoidJointState.msg:
# string name
# float64 position
# float64 velocity
# float64 effort
# float64 commanded_position
# float64 commanded_effort
# bool is_safe
```

Then use it in your nodes:

```python
from my_humanoid_msgs.msg import HumanoidJointState

class AdvancedJointController(Node):
    def __init__(self):
        super().__init__('advanced_joint_controller')
        self.joint_pub = self.create_publisher(
            HumanoidJointState, 'advanced_joint_states', 10)
        
        self.joint_sub = self.create_subscription(
            HumanoidJointState, 
            'joint_command', 
            self.joint_command_callback, 
            10)
    
    def joint_command_callback(self, msg):
        # Process advanced joint state with safety checks
        if msg.is_safe:
            # Execute joint command
            self.execute_joint_command(msg)
        else:
            self.get_logger().error(f'Unsafe joint command for {msg.name}')
```

### Publisher/Subscriber Options

Different QoS settings for various use cases:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# For real-time control data
control_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,  # Only keep the latest message
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# For logging/visualization
logging_qos = QoSProfile(
    history=HistoryPolicy.KEEP_ALL,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

# For sensor data
sensor_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# Create publishers with appropriate QoS
self.control_pub = self.create_publisher(JointCommand, 'control_commands', control_qos)
self.log_pub = self.create_publisher(LogMessage, 'logs', logging_qos)
self.sensor_pub = self.create_publisher(SensorData, 'sensor_data', sensor_qos)
```

## Services in Humanoid Robotics

### Advanced Service Implementation

Services for humanoid-specific tasks:

```python
# In your srv directory, create Balance.srv:
# float64 center_of_mass_x
# float64 center_of_mass_y
# ---
# bool is_balanced
# float64 error_x
# float64 error_y

from my_humanoid_msgs.srv import Balance

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        self.srv = self.create_service(Balance, 'balance_robot', self.balance_callback)
    
    def balance_callback(self, request, response):
        # Calculate balance based on CoM position
        current_com = self.get_current_com()
        error_x = request.center_of_mass_x - current_com.x
        error_y = request.center_of_mass_y - current_com.y
        
        # Attempt to balance
        success = self.adjust_balance(error_x, error_y)
        
        response.is_balanced = success
        response.error_x = error_x
        response.error_y = error_y
        
        return response
```

### Service Client Implementation

```python
from my_humanoid_msgs.srv import Balance
import rclpy
from rclpy.node import Node

class BalanceClient(Node):
    def __init__(self):
        super().__init__('balance_client')
        self.cli = self.create_client(Balance, 'balance_robot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Balance service not available, waiting again...')
        
        self.request = Balance.Request()
    
    def balance_robot(self, com_x, com_y):
        self.request.center_of_mass_x = com_x
        self.request.center_of_mass_y = com_y
        
        future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()
```

## Actions for Complex Tasks

### Advanced Action Implementation

For humanoid-specific long-running tasks:

```python
# In your action directory, create Walk.action:
# # Goal: Target position and orientation
# float64 target_x
# float64 target_y
# float64 target_theta
# ---
# # Result: Success or failure
# bool success
# string message
# float64 final_x
# float64 final_y
# float64 final_theta
# ---
# # Feedback: Current progress
# float64 current_x
# float64 current_y
# float64 current_theta
# float64 distance_remaining
# string status

from my_humanoid_msgs.action import Walk
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle

class WalkActionServer(Node):
    def __init__(self):
        super().__init__('walk_action_server')
        self._action_server = ActionServer(
            self,
            Walk,
            'walk_to_goal',
            execute_callback=self.execute_walk,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
    
    def goal_callback(self, goal_request):
        # Accept or reject the goal
        if self.is_walk_valid(goal_request.target_x, goal_request.target_y):
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT
    
    def cancel_callback(self, goal_handle):
        # Accept or reject the cancel request
        return CancelResponse.ACCEPT
    
    def execute_walk(self, goal_handle):
        self.get_logger().info('Executing walk to goal...')
        
        # Initial feedback
        feedback_msg = Walk.Feedback()
        feedback_msg.current_x = self.get_robot_x()
        feedback_msg.current_y = self.get_robot_y()
        feedback_msg.current_theta = self.get_robot_theta()
        feedback_msg.distance_remaining = self.calculate_distance_remaining(
            goal_handle.request.target_x,
            goal_handle.request.target_y,
            feedback_msg.current_x,
            feedback_msg.current_y
        )
        feedback_msg.status = 'Walking to goal'
        
        # Walk to goal with feedback
        success = self.walk_to_position_with_feedback(
            goal_handle.request.target_x,
            goal_handle.request.target_y,
            goal_handle.request.target_theta,
            feedback_msg,
            goal_handle
        )
        
        # Populate result
        result = Walk.Result()
        result.success = success
        result.final_x = self.get_robot_x()
        result.final_y = self.get_robot_y()
        result.final_theta = self.get_robot_theta()
        result.message = 'Walk completed' if success else 'Walk failed'
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        return result
    
    def walk_to_position_with_feedback(self, target_x, target_y, target_theta, feedback_msg, goal_handle):
        # Implementation of walking algorithm with feedback publishing
        while not self.reached_target(target_x, target_y) and not goal_handle.is_cancel_requested:
            # Update robot position
            self.update_robot_position()
            
            # Update feedback
            feedback_msg.current_x = self.get_robot_x()
            feedback_msg.current_y = self.get_robot_y()
            feedback_msg.current_theta = self.get_robot_theta()
            feedback_msg.distance_remaining = self.calculate_distance_remaining(
                target_x, target_y, feedback_msg.current_x, feedback_msg.current_y)
            feedback_msg.status = f'Walking: {feedback_msg.distance_remaining:.2f}m remaining'
            
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            
            # Small delay to allow other processes
            self.get_clock().sleep_for(Duration(seconds=0.1))
        
        return self.reached_target(target_x, target_y)
```

### Action Client Implementation

```python
from my_humanoid_msgs.action import Walk
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node

class WalkActionClient(Node):
    def __init__(self):
        super().__init__('walk_action_client')
        self._action_client = ActionClient(self, Walk, 'walk_to_goal')
    
    def send_goal(self, x, y, theta):
        goal_msg = Walk.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y
        goal_msg.target_theta = theta
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback received: {feedback.distance_remaining:.2f}m remaining, '
            f'status: {feedback.status}')
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}, message: {result.message}')
```

## Advanced Communication Patterns

### Multiple Publishers/Listeners

```python
class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Multiple subscribers for different sensors
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        
        # Publisher for fused data
        self.fused_pub = self.create_publisher(
            Odometry, 'fused_odom', 10)
    
    def imu_callback(self, msg):
        # Process IMU data
        self.imu_data = msg
    
    def odom_callback(self, msg):
        # Process odometry data
        self.odom_data = msg
    
    def laser_callback(self, msg):
        # Process laser data
        self.laser_data = msg
        
        # When all data is available, publish fused result
        if hasattr(self, 'imu_data') and hasattr(self, 'odom_data'):
            fused_odom = self.fuse_sensor_data()
            self.fused_pub.publish(fused_odom)
```

### Transforms (tf2)

For humanoid robotics, managing coordinate frames is crucial:

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publish transforms at 50Hz
        self.timer = self.create_timer(0.02, self.publish_transforms)
    
    def publish_transforms(self):
        # Get current joint states (from your joint state subscriber)
        # Calculate forward kinematics to determine link positions
        transforms = self.calculate_link_transforms()
        
        # Publish all transforms
        for transform in transforms:
            self.tf_broadcaster.sendTransform(transform)
```

These advanced communication patterns are essential for building complex humanoid robotics systems with ROS2. In the next chapter, we'll explore packages and workspaces in more detail.