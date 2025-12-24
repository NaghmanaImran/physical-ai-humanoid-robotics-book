---
sidebar_position: 6
---

# Manipulation in NVIDIA Isaac for Humanoid Robotics

## Overview of Manipulation in Isaac

Manipulation in NVIDIA Isaac encompasses the technologies and techniques for enabling humanoid robots to interact with objects in their environment. This includes grasp planning, dexterous manipulation, force control, and integration with perception systems to create robust manipulation capabilities.

## Isaac Sim Manipulation Tools

### Physics-Based Manipulation Simulation

Isaac Sim provides accurate physics simulation essential for manipulation tasks:

```python
# Example: Physics-based manipulation in Isaac Sim
import omni
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np

class ManipulationWorld:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.robot = None
        self.objects = []
        
    def setup_manipulation_scene(self):
        """Setup a scene for manipulation tasks"""
        # Add ground plane
        self.world.scene.add_default_ground_plane()
        
        # Add a humanoid robot (simplified as a manipulator for this example)
        assets_root_path = get_assets_root_path()
        if assets_root_path is not None:
            # Add a robot with manipulation capabilities
            robot_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
            add_reference_to_stage(usd_path=robot_path, prim_path="/World/Robot")
            
            # Add manipulable objects
            self.add_manipulable_objects()
        
        self.world.reset()
    
    def add_manipulable_objects(self):
        """Add objects that can be manipulated"""
        # Add a cube to manipulate
        cube = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",
                name="cube",
                position=np.array([0.5, 0.0, 0.5]),
                size=np.array([0.1, 0.1, 0.1]),
                color=np.array([0.8, 0.1, 0.1])
            )
        )
        self.objects.append(cube)
        
        # Add other objects
        sphere = self.world.scene.add(
            DynamicCuboid(  # Using cuboid for simplicity
                prim_path="/World/Sphere",
                name="sphere",
                position=np.array([0.7, 0.2, 0.5]),
                size=np.array([0.08, 0.08, 0.08]),
                color=np.array([0.1, 0.8, 0.1])
            )
        )
        self.objects.append(sphere)
    
    def execute_manipulation_task(self):
        """Execute a simple manipulation task"""
        # Reset the world
        self.world.reset()
        
        # Example manipulation task: move robot to object
        for i in range(1000):
            # In a real implementation, this would involve:
            # 1. Perception to locate objects
            # 2. Motion planning to reach object
            # 3. Grasp planning
            # 4. Execution of grasp
            # 5. Transport to destination
            
            self.world.step(render=True)
            
            if i % 100 == 0:
                print(f"Manipulation simulation step: {i}")
    
    def get_object_states(self):
        """Get current states of manipulable objects"""
        states = {}
        for obj in self.objects:
            pos, quat = obj.get_world_pose()
            lin_vel, ang_vel = obj.get_linear_velocity(), obj.get_angular_velocity()
            states[obj.name] = {
                'position': pos,
                'orientation': quat,
                'linear_velocity': lin_vel,
                'angular_velocity': ang_vel
            }
        return states

# Usage
manip_world = ManipulationWorld()
manip_world.setup_manipulation_scene()
manip_world.execute_manipulation_task()
```

### Grasp Planning in Simulation

```python
# Grasp planning simulation
from omni.isaac.core import World
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import Gf, Usd, UsdGeom
import numpy as np

class GraspPlannerSim:
    def __init__(self, world):
        self.world = world
        self.robot_prim = None
        self.object_prims = []
        
    def find_grasp_poses(self, object_prim_path):
        """Find potential grasp poses for an object"""
        # Get object properties
        object_prim = get_prim_at_path(object_prim_path)
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default, ["default"])
        bbox = bbox_cache.ComputeWorldBBox(object_prim)
        
        # Calculate grasp points around the object
        center = Gf.Vec3d(
            (bbox.GetRange().GetMin()[0] + bbox.GetRange().GetMax()[0]) / 2,
            (bbox.GetRange().GetMin()[1] + bbox.GetRange().GetMax()[1]) / 2,
            (bbox.GetRange().GetMin()[2] + bbox.GetRange().GetMax()[2]) / 2
        )
        
        size = bbox.GetRange().GetSize()
        
        # Generate potential grasp poses around the object
        grasp_poses = []
        for angle in np.linspace(0, 2*np.pi, 8):  # 8 grasp positions around object
            x_offset = 0.15 * np.cos(angle)  # 15cm from object
            y_offset = 0.15 * np.sin(angle)
            
            # Position grasp point near the object
            grasp_pos = [center[0] + x_offset, center[1] + y_offset, center[2] + size[2]/2 + 0.05]
            
            # Orientation to grasp from the side
            grasp_quat = [0.707, 0.0, 0.707, 0.0]  # Rotate 90 degrees around X-axis
            
            grasp_poses.append({
                'position': grasp_pos,
                'orientation': grasp_quat,
                'approach_direction': [-x_offset, -y_offset, 0]  # Approach from this direction
            })
        
        return grasp_poses
    
    def simulate_grasp(self, object_prim_path, grasp_pose):
        """Simulate a grasp action"""
        # Move robot to grasp pose
        # In a real implementation, this would control the robot's end-effector
        print(f"Simulating grasp at position: {grasp_pose['position']}")
        
        # Check if grasp would be successful based on simulation
        # This would involve checking contact forces, grasp stability, etc.
        success = self.check_grasp_stability(object_prim_path, grasp_pose)
        
        return success
    
    def check_grasp_stability(self, object_prim_path, grasp_pose):
        """Check if a grasp is stable using physics simulation"""
        # In a real implementation, this would:
        # 1. Apply forces to the object in simulation
        # 2. Check if the object remains stable
        # 3. Evaluate grasp quality metrics
        
        # For this example, return a random result
        return np.random.random() > 0.3  # 70% success rate
```

## Isaac ROS Manipulation Stack

### Isaac ROS Manipulation Packages

```python
# Example of Isaac ROS manipulation node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import MoveItErrorCodes
import numpy as np

class IsaacROSManipulator(Node):
    def __init__(self):
        super().__init__('isaac_ros_manipulator')
        
        # Publishers for manipulation commands
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        self.grasp_command_pub = self.create_publisher(
            String,
            '/grasp_command',
            10
        )
        
        # Subscribers for manipulation feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.object_position_sub = self.create_subscription(
            PointStamped,
            '/closest_object',
            self.object_position_callback,
            10
        )
        
        # Manipulation state
        self.current_joint_positions = {}
        self.target_object_position = None
        
        # Timer for manipulation planning
        self.manip_timer = self.create_timer(0.1, self.manipulation_callback)
        
        self.get_logger().info('Isaac ROS Manipulator initialized')
    
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
    
    def object_position_callback(self, msg):
        """Update target object position"""
        self.target_object_position = msg.point
    
    def manipulation_callback(self):
        """Main manipulation planning callback"""
        if self.target_object_position is not None:
            # Plan manipulation to reach the target object
            self.plan_manipulation_to_object()
    
    def plan_manipulation_to_object(self):
        """Plan and execute manipulation to reach target object"""
        # In a real implementation, this would:
        # 1. Use MoveIt to plan a trajectory to the object
        # 2. Consider collision avoidance
        # 3. Generate grasp pose
        # 4. Execute the plan
        
        # For this example, send a simple joint trajectory
        self.send_simple_trajectory()
    
    def send_simple_trajectory(self):
        """Send a simple joint trajectory for demonstration"""
        msg = JointTrajectory()
        msg.joint_names = [
            'joint1', 'joint2', 'joint3',  # Replace with actual joint names
            'joint4', 'joint5', 'joint6'
        ]
        
        point = JointTrajectoryPoint()
        # Set desired joint positions (example values)
        point.positions = [0.5, 0.3, -0.2, 0.1, 0.4, -0.3]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        msg.points = [point]
        
        self.joint_trajectory_pub.publish(msg)
        self.get_logger().info('Sent joint trajectory command')

def main(args=None):
    rclpy.init(args=args)
    node = IsaacROSManipulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Force Control and Compliance

```python
# Force control for compliant manipulation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class ForceControlNode(Node):
    def __init__(self):
        super().__init__('force_control')
        
        # Subscribers
        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            '/ft_sensor/wrench',
            self.wrench_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )
        
        self.ee_command_pub = self.create_publisher(
            Twist,
            '/ee_force_control',
            10
        )
        
        # Force control parameters
        self.stiffness = np.diag([1000, 1000, 1000, 100, 100, 100])  # Cartesian stiffness
        self.damping_ratio = 1.0  # Critical damping
        self.target_force = np.array([0.0, 0.0, -5.0, 0.0, 0.0, 0.0])  # 5N downward force
        
        # Current state
        self.current_wrench = np.zeros(6)
        self.current_joint_positions = {}
        
        self.get_logger().info('Force control node initialized')
    
    def wrench_callback(self, msg):
        """Update current wrench measurement"""
        self.current_wrench = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y, 
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])
    
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
    
    def compute_impedance_control(self, target_pose, target_wrench=None):
        """Compute impedance control commands"""
        if target_wrench is None:
            target_wrench = self.target_force
        
        # Compute force error
        force_error = target_wrench - self.current_wrench
        
        # Simple impedance control: F = K * (x_desired - x_current) + D * (v_desired - v_current)
        # For force control, we adjust position based on force error
        position_correction = np.linalg.solve(self.stiffness, force_error)
        
        # Publish position correction commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = position_correction.tolist()
        self.joint_command_pub.publish(cmd_msg)
        
        self.get_logger().info(f'Force error: {force_error}, Position correction: {position_correction}')

def main(args=None):
    rclpy.init(args=args)
    node = ForceControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Advanced Manipulation Techniques

### Visual Servoing

```python
# Visual servoing for precise manipulation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2D
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing')
        
        self.cv_bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.detection_sub = self.create_subscription(
            Detection2D,
            '/target_detection',
            self.detection_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/visual_servo_cmd',
            10
        )
        
        # Visual servoing parameters
        self.gain = 1.0
        self.pixel_threshold = 10  # pixels
        self.target_center = None
        self.camera_matrix = None
        
        self.get_logger().info('Visual servoing node initialized')
    
    def camera_info_callback(self, msg):
        """Get camera intrinsic parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
    
    def detection_callback(self, msg):
        """Process detection to get target position"""
        self.target_center = np.array([
            msg.bbox.center.x,
            msg.bbox.center.y
        ])
    
    def image_callback(self, msg):
        """Process image for visual servoing"""
        if self.target_center is not None and self.camera_matrix is not None:
            # Get image center
            image_center = np.array([msg.width / 2, msg.height / 2])
            
            # Compute error in pixel space
            pixel_error = self.target_center - image_center
            
            # Check if target is centered enough
            if np.linalg.norm(pixel_error) > self.pixel_threshold:
                # Convert pixel error to camera frame velocity
                camera_vel = self.pixel_error_to_camera_velocity(pixel_error)
                
                # Publish velocity command
                cmd_msg = Twist()
                cmd_msg.linear.x = camera_vel[0] * self.gain
                cmd_msg.linear.y = camera_vel[1] * self.gain
                cmd_msg.linear.z = camera_vel[2] * self.gain
                cmd_msg.angular.z = camera_vel[5] * self.gain  # Simplified rotation
                
                self.cmd_vel_pub.publish(cmd_msg)
            else:
                # Target is centered, stop movement
                cmd_msg = Twist()
                self.cmd_vel_pub.publish(cmd_msg)
    
    def pixel_error_to_camera_velocity(self, pixel_error):
        """Convert pixel error to camera frame velocity"""
        # Simple approximation: velocity proportional to pixel error
        # In practice, this would use more sophisticated visual servoing laws
        camera_vel = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]
        
        # Map pixel errors to camera motions
        camera_vel[0] = -pixel_error[1] * 0.001  # Vertical pixel error -> forward/back motion
        camera_vel[1] = -pixel_error[0] * 0.001  # Horizontal pixel error -> lateral motion
        camera_vel[2] = 0.0  # No vertical motion for this example
        
        return camera_vel

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Bimanual Manipulation

```python
# Bimanual manipulation coordination
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

class BimanualManipulator(Node):
    def __init__(self):
        super().__init__('bimanual_manipulator')
        
        # Publishers for both arms
        self.left_arm_pub = self.create_publisher(
            JointTrajectory,
            '/left_arm_controller/joint_trajectory',
            10
        )
        
        self.right_arm_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            10
        )
        
        # Subscribers for both arms
        self.left_joint_sub = self.create_subscription(
            JointState,
            '/left_arm/joint_states',
            self.left_joint_callback,
            10
        )
        
        self.right_joint_sub = self.create_subscription(
            JointState,
            '/right_arm/joint_states',
            self.right_joint_callback,
            10
        )
        
        # Coordination parameters
        self.coordination_mode = "independent"  # "independent", "coordinated", "symmetric"
        
        # Current joint positions
        self.left_positions = {}
        self.right_positions = {}
        
        # Timer for coordination
        self.coordination_timer = self.create_timer(0.1, self.coordination_callback)
        
        self.get_logger().info('Bimanual manipulator initialized')
    
    def left_joint_callback(self, msg):
        """Update left arm joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.left_positions[name] = msg.position[i]
    
    def right_joint_callback(self, msg):
        """Update right arm joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.right_positions[name] = msg.position[i]
    
    def coordination_callback(self):
        """Handle bimanual coordination"""
        if self.coordination_mode == "symmetric":
            self.execute_symmetric_motion()
        elif self.coordination_mode == "coordinated":
            self.execute_coordinated_task()
    
    def execute_symmetric_motion(self):
        """Execute symmetric motions with both arms"""
        # Example: Both arms mirror each other's movements
        # In a real implementation, this would track a reference trajectory
        # and generate symmetric commands for the other arm
        
        # For demonstration, send a simple symmetric trajectory
        left_traj = self.create_joint_trajectory(
            joint_names=['left_joint1', 'left_joint2', 'left_joint3'],
            positions=[0.5, 0.3, -0.2],
            duration=2.0
        )
        
        right_traj = self.create_joint_trajectory(
            joint_names=['right_joint1', 'right_joint2', 'right_joint3'],
            positions=[0.5, -0.3, 0.2],  # Symmetric positions
            duration=2.0
        )
        
        self.left_arm_pub.publish(left_traj)
        self.right_arm_pub.publish(right_traj)
    
    def execute_coordinated_task(self):
        """Execute a coordinated manipulation task"""
        # Example: One arm holds an object while the other manipulates it
        # This would require more sophisticated planning in practice
        
        # For demonstration, send coordinated trajectories
        left_traj = self.create_joint_trajectory(
            joint_names=['left_joint1', 'left_joint2', 'left_joint3'],
            positions=[0.0, 0.0, 0.0],  # Hold position
            duration=2.0
        )
        
        right_traj = self.create_joint_trajectory(
            joint_names=['right_joint1', 'right_joint2', 'right_joint3'],
            positions=[0.5, 0.3, -0.2],  # Move to manipulate
            duration=2.0
        )
        
        self.left_arm_pub.publish(left_traj)
        self.right_arm_pub.publish(right_traj)
    
    def create_joint_trajectory(self, joint_names, positions, duration):
        """Create a simple joint trajectory message"""
        msg = JointTrajectory()
        msg.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        msg.points = [point]
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = BimanualManipulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Integration with Perception for Manipulation

### Perception-Guided Manipulation

```python
# Integration of perception and manipulation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import numpy as np

class PerceptionGuidedManipulation(Node):
    def __init__(self):
        super().__init__('perception_guided_manipulation')
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/isaac_ros/detections',
            self.detection_callback,
            10
        )
        
        self.object_3d_sub = self.create_subscription(
            PointStamped,
            '/object_3d_position',
            self.object_3d_callback,
            10
        )
        
        # Publishers
        self.manipulation_goal_pub = self.create_publisher(
            PoseStamped,
            '/manipulation_goal',
            10
        )
        
        self.manipulation_command_pub = self.create_publisher(
            String,
            '/manipulation_command',
            10
        )
        
        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Manipulation state
        self.detected_objects = {}
        self.selected_object = None
        
        self.get_logger().info('Perception-guided manipulation initialized')
    
    def detection_callback(self, msg):
        """Process object detections"""
        for detection in msg.detections:
            # For this example, assume we're looking for a specific object type
            if detection.results[0].id == 1:  # Example: looking for object ID 1
                object_info = {
                    'bbox': detection.bbox,
                    'confidence': detection.results[0].score,
                    'timestamp': self.get_clock().now()
                }
                
                # Store detected object
                obj_id = f"object_{len(self.detected_objects)}"
                self.detected_objects[obj_id] = object_info
                
                # Select the most confident detection as the target
                if (self.selected_object is None or 
                    object_info['confidence'] > self.detected_objects[self.selected_object]['confidence']):
                    self.selected_object = obj_id
    
    def object_3d_callback(self, msg):
        """Process 3D object position"""
        if self.selected_object is not None:
            # Update 3D position of selected object
            self.detected_objects[self.selected_object]['position_3d'] = [
                msg.point.x, msg.point.y, msg.point.z
            ]
            
            # Plan manipulation to reach this object
            self.plan_manipulation_to_object(self.selected_object)
    
    def plan_manipulation_to_object(self, obj_id):
        """Plan manipulation to reach the specified object"""
        obj_info = self.detected_objects[obj_id]
        
        if 'position_3d' in obj_info:
            # Create manipulation goal at object position
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'base_link'  # Adjust as needed
            goal_msg.pose.position.x = obj_info['position_3d'][0]
            goal_msg.pose.position.y = obj_info['position_3d'][1]
            goal_msg.pose.position.z = obj_info['position_3d'][2]
            
            # Set orientation for approach
            goal_msg.pose.orientation.w = 1.0  # Default orientation
            
            # Publish manipulation goal
            self.manipulation_goal_pub.publish(goal_msg)
            
            # Send manipulation command
            cmd_msg = String()
            cmd_msg.data = f"approach_object_{obj_id}"
            self.manipulation_command_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionGuidedManipulation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Manipulation Best Practices

### Safety and Robustness

When implementing manipulation in Isaac, consider these best practices:

1. **Force Limiting**: Always implement force limits to prevent damage
2. **Collision Checking**: Verify trajectories don't cause self-collisions
3. **Grasp Verification**: Check grasp stability before lifting objects
4. **Recovery Behaviors**: Implement recovery when manipulation fails

### Performance Optimization

1. **Simulation Speed**: Use appropriate physics parameters for real-time performance
2. **Model Simplification**: Use simplified collision models for fast simulation
3. **Parallel Processing**: Leverage Isaac's multi-GPU capabilities
4. **Caching**: Cache computed grasp poses and motion plans when possible

The manipulation capabilities in NVIDIA Isaac provide a comprehensive framework for developing sophisticated manipulation behaviors in humanoid robots. Combined with Isaac's perception and simulation capabilities, this creates a powerful platform for advanced robotics applications.