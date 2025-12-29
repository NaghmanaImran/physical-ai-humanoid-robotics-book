# Robot Modeling in Gazebo

## Overview

Robot modeling in Gazebo involves creating accurate 3D representations of physical robots that can be simulated in virtual environments. This chapter covers the fundamentals of robot modeling, including the Unified Robot Description Format (URDF), joint configurations, physical properties, and sensor integration.

## Understanding URDF (Unified Robot Description Format)

URDF (Unified Robot Description Format) is an XML-based format used to describe robots in ROS and Gazebo. It defines the robot's physical structure, including links, joints, and other properties.

### URDF Structure

A basic URDF file consists of:
- **Links**: Rigid parts of the robot (e.g., chassis, arms, wheels)
- **Joints**: Connections between links (e.g., revolute, prismatic, fixed)
- **Visual**: How the robot appears in simulation
- **Collision**: How the robot interacts physically with the environment
- **Inertial**: Mass, center of mass, and inertia properties

### Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Wheel links -->
  <link name="wheel_front_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joint connecting wheel to base -->
  <joint name="wheel_front_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_front_left"/>
    <origin xyz="0.2 0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Creating 3D Robot Models

### Link Definition

Links represent rigid parts of the robot. Each link must have:

1. **Visual Properties**: How the link appears in the simulation
2. **Collision Properties**: How the link interacts physically
3. **Inertial Properties**: Mass and inertia characteristics

#### Visual Properties
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Options: box, cylinder, sphere, mesh -->
    <box size="1.0 0.5 0.3"/>
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

#### Collision Properties
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1.0 0.5 0.3"/>
  </geometry>
</collision>
```

#### Inertial Properties
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

### Joint Types

Joints connect links and define how they can move relative to each other:

1. **Fixed**: No movement between links
2. **Revolute**: Rotational movement around a single axis
3. **Continuous**: Like revolute but unlimited rotation
4. **Prismatic**: Linear sliding movement
5. **Floating**: 6 degrees of freedom
6. **Planar**: Movement in a plane

#### Joint Definition Example
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>
```

## Adding Joints and Kinematic Chains

### Kinematic Chains

A kinematic chain is a series of rigid bodies (links) connected by joints. For example, a robot arm consists of multiple links connected by joints.

### Example: Simple Robot Arm
```xml
<!-- Shoulder joint -->
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
</joint>

<!-- Elbow joint -->
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0.3 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
</joint>

<!-- Wrist joint -->
<joint name="wrist_joint" type="revolute">
  <parent link="forearm"/>
  <child link="hand"/>
  <origin xyz="0.25 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
</joint>
```

## Configuring Physical Properties

### Mass and Inertia

Accurate mass and inertia properties are crucial for realistic simulation:

- **Mass**: The weight of each link
- **Inertia**: Resistance to rotational motion
- **Center of Mass**: Point where mass is concentrated

For common shapes, you can calculate inertia using these formulas:
- Box: `Ixx = m/12 * (h² + d²)`, `Iyy = m/12 * (w² + d²)`, `Izz = m/12 * (w² + h²)`
- Cylinder: `Ixx = Iyy = m/12 * (3r² + h²)`, `Izz = m/2 * r²`
- Sphere: `Ixx = Iyy = Izz = 2m/5 * r²`

Where:
- m = mass
- w = width, h = height, d = depth
- r = radius, h = height

### Friction and Damping

Add friction and damping properties for more realistic simulation:

```xml
<gazebo reference="link_name">
  <mu1>0.3</mu1>  <!-- Friction coefficient -->
  <mu2>0.3</mu2>  <!-- Secondary friction coefficient -->
  <kp>1000000.0</kp>  <!-- Spring stiffness -->
  <kd>1.0</kd>     <!-- Damping coefficient -->
  <self_collide>false</self_collide>
</gazebo>
```

## Integrating Sensors

### Camera Sensor
```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### LIDAR Sensor
```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>
          <max_angle>1.570796</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
      <topic_name>laser_scan</topic_name>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensor
```xml
<gazebo reference="imu_link">
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

## XACRO for Complex Models

XACRO (XML Macros) is an extension to URDF that allows for more complex robot descriptions using macros, properties, and mathematical expressions.

### Basic XACRO Example
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">
  
  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_width" value="0.5" />
  <xacro:property name="base_length" value="0.8" />
  <xacro:property name="base_height" value="0.2" />
  
  <!-- Macro for wheels -->
  <xacro:macro name="wheel" params="prefix *origin">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>
    
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <xacro:insert_block name="origin"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>
  
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Wheels using macro -->
  <xacro:wheel prefix="front_left">
    <origin xyz="0.2 0.2 0" rpy="0 0 0"/>
  </xacro:wheel>
  
  <xacro:wheel prefix="front_right">
    <origin xyz="0.2 -0.2 0" rpy="0 0 0"/>
  </xacro:wheel>
  
</robot>
```

## Best Practices for Robot Modeling

### 1. Model Simplification
- Use simplified geometries for collision models
- Complex visual models don't need to match collision models exactly
- Balance accuracy with performance

### 2. Proper Scaling
- Ensure all dimensions are in meters
- Verify that the model is properly scaled
- Check that mass values are realistic

### 3. Coordinate Systems
- Use consistent coordinate conventions (typically X-forward, Y-left, Z-up)
- Ensure joint axes are properly oriented
- Verify that transformations are correct

### 4. Testing Your Model
- Load the model in Gazebo to check for errors
- Verify that joints move as expected
- Test collision detection
- Check that sensors are properly positioned

## Loading Models in Gazebo

### Method 1: Direct Loading
```bash
gz sim -r simple_robot.urdf
```

### Method 2: Through ROS2
```bash
# Launch Gazebo with ROS2 bridge
ros2 launch gazebo_ros empty_world.launch.py

# Spawn the robot
ros2 run gazebo_ros spawn_entity.py -file /path/to/robot.urdf -entity robot_name
```

### Method 3: Using Launch Files
Create a launch file to automate the process:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ])
        ),
        
        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot', '-file', '/path/to/robot.urdf'],
            output='screen'
        )
    ])
```

## Common Modeling Issues and Solutions

### Issue 1: Robot Falls Through Ground
**Cause**: Incorrect inertial properties or missing collision models
**Solution**: Verify mass, inertia, and collision geometries are properly defined

### Issue 2: Joints Behaving Unexpectedly
**Cause**: Incorrect joint axes or limits
**Solution**: Check joint axis orientation and limit values

### Issue 3: Performance Issues
**Cause**: Too complex collision or visual geometries
**Solution**: Simplify geometries or use simpler shapes for collision models

### Issue 4: Robot Parts Overlapping
**Cause**: Incorrect origins or transformations
**Solution**: Verify all xyz and rpy values in origins

## Advanced Topics

### Transmission Elements
For controlling joints with ROS2, add transmission elements:

```xml
<transmission name="wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="wheel_front_left_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo-Specific Properties
Add Gazebo-specific properties for enhanced simulation:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <turnGravityOff>false</turnGravityOff>
  <self_collide>false</self_collide>
  <enable_wind>false</enable_wind>
  <gravity>1</gravity>
</gazebo>
```

## Chapter Summary

In this chapter, you learned how to create accurate 3D models of robots for Gazebo simulation. You covered:

1. The fundamentals of URDF and robot modeling
2. How to define links, joints, and kinematic chains
3. How to configure physical properties for realistic simulation
4. How to integrate various sensors into your robot model
5. Advanced techniques using XACRO for complex models
6. Best practices for robot modeling

## Next Steps

Now that you understand how to create robot models, continue to the next chapter to learn about basic simulation concepts in Gazebo.

## Diagram Placeholders

[Image: URDF Structure diagram showing the structure of a URDF file]

[Image: Robot Modeling Process diagram showing the robot modeling workflow]