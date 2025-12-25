---
sidebar_position: 4
---

# Robot Modeling in Gazebo for Humanoid Robotics

## Understanding Robot Modeling

Robot modeling in Gazebo involves creating accurate 3D representations of robots that behave realistically in simulation. For humanoid robots, this is particularly challenging due to the complex kinematics, dynamics, and the need for stable locomotion.

## URDF vs. SDF for Humanoid Modeling

### URDF (Unified Robot Description Format)

URDF is the standard format for describing robots in ROS and works well with Gazebo through the `gazebo_ros` plugins:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.5 0.3 1.0"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.5 0.3 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo-specific extensions -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1000000.0</kd>
  </gazebo>
</robot>
```

### SDF (Simulation Description Format)

SDF is Gazebo's native format and provides more direct access to Gazebo-specific features:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="humanoid_model">
    <!-- Links -->
    <link name="base_link">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
      
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 1.0</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient>
          <diffuse>0 0 1 1</diffuse>
          <specular>0 0 1 1</specular>
        </material>
      </visual>
      
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 1.0</size>
          </box>
        </geometry>
        
        <surface>
          <friction>
            <ode>
              <mu>0.2</mu>
              <mu2>0.2</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>1000000.0</kd>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
```

## Kinematic Chain Design for Humanoids

### Humanoid Joint Structure

A typical humanoid robot has multiple kinematic chains:

```xml
<!-- Head chain -->
<joint name="neck_joint" type="revolute">
  <parent link="base_link"/>
  <child link="head_link"/>
  <origin xyz="0 0 1.0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="10" velocity="3"/>
</joint>

<!-- Left arm chain -->
<joint name="left_shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="left_upper_arm"/>
  <origin xyz="0 0.15 0.8"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="3"/>
</joint>

<joint name="left_elbow_joint" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_lower_arm"/>
  <origin xyz="0 0 -0.3"/>
  <axis xyz="0 0 1"/>
  <limit lower="-2.0" upper="0.5" effort="30" velocity="3"/>
</joint>

<!-- Left leg chain -->
<joint name="left_hip_joint" type="revolute">
  <parent link="base_link"/>
  <child link="left_thigh"/>
  <origin xyz="0 0.1 -0.1"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3"/>
</joint>

<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.4"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.0" effort="100" velocity="3"/>
</joint>

<joint name="left_ankle_joint" type="revolute">
  <parent link="left_shin"/>
  <child link="left_foot"/>
  <origin xyz="0 0 -0.4"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="50" velocity="3"/>
</joint>
```

## Inertial Properties for Humanoid Stability

### Calculating Inertial Parameters

Accurate inertial properties are crucial for stable humanoid simulation:

```xml
<!-- Example: Properly configured link with realistic inertial properties -->
<link name="thigh_link">
  <inertial>
    <!-- Mass based on approximate volume and density -->
    <mass value="5.0"/>
    <!-- Origin at the center of mass -->
    <origin xyz="0 0 -0.2"/>
    <!-- Inertia tensor for a cylinder-like shape -->
    <inertia 
      ixx="0.05" 
      ixy="0.0" 
      ixz="0.0" 
      iyy="0.05" 
      iyz="0.0" 
      izz="0.01"/>
  </inertial>
  
  <visual>
    <origin xyz="0 0 -0.2"/>
    <geometry>
      <cylinder length="0.4" radius="0.08"/>
    </geometry>
  </visual>
  
  <collision>
    <origin xyz="0 0 -0.2"/>
    <geometry>
      <cylinder length="0.4" radius="0.08"/>
    </geometry>
  </collision>
</link>
```

## Gazebo-Specific Extensions

### Adding Gazebo Plugins

For humanoid robots, several plugins are essential:

```xml
<!-- Controller plugin for ROS control -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find humanoid_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>

<!-- IMU sensor plugin -->
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
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

<!-- Force/Torque sensor plugin -->
<gazebo>
  <plugin name="left_foot_ft_sensor" filename="libgazebo_ros_ft_sensor.so">
    <ros>
      <namespace>left_foot</namespace>
      <remapping>~/out:=left_foot/ft_sensor</remapping>
    </ros>
    <update_rate>100</update_rate>
    <topic>left_foot/ft_sensor</topic>
  </plugin>
</gazebo>
```

## Creating Complex Humanoid Models with Xacro

Xacro allows for parameterization and macro definitions, making complex humanoid models manageable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="advanced_humanoid">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_mass" value="10.0" />
  <xacro:property name="link_density" value="1000" /> <!-- kg/m^3 -->

  <!-- Macro for creating links with consistent properties -->
  <xacro:macro name="humanoid_link" params="name length radius mass *origin *geometry">
    <link name="${name}">
      <inertial>
        <mass value="${mass}"/>
        <xacro:insert_block name="origin"/>
        <!-- Calculate inertia for cylinder -->
        <inertia 
          ixx="${0.0833333 * mass * (3*radius*radius + length*length)}" 
          ixy="0.0" 
          ixz="0.0" 
          iyy="${0.0833333 * mass * (3*radius*radius + length*length)}" 
          iyz="0.0" 
          izz="${0.25 * mass * radius * radius}"/>
      </inertial>
      
      <visual>
        <xacro:insert_block name="origin"/>
        <xacro:insert_block name="geometry"/>
      </visual>
      
      <collision>
        <xacro:insert_block name="origin"/>
        <xacro:insert_block name="geometry"/>
      </collision>
    </link>
  </xacro:macro>

  <!-- Macro for creating joints with consistent properties -->
  <xacro:macro name="humanoid_joint" params="name type parent child xyz lower upper effort velocity">
    <joint name="${name}" type="${type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
      <dynamics damping="1.0" friction="0.1"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <xacro:humanoid_link name="base_link" length="0.6" radius="0.15" mass="${base_mass}">
    <origin xyz="0 0 0.3"/>
    <geometry>
      <cylinder length="0.6" radius="0.15"/>
    </geometry>
  </xacro:humanoid_link>

  <!-- Left leg using macros -->
  <xacro:humanoid_joint name="left_hip_joint" type="revolute" 
                        parent="base_link" child="left_thigh" 
                        xyz="0 0.15 -0.1" 
                        lower="${-M_PI/2}" upper="${M_PI/2}" 
                        effort="100" velocity="3"/>

  <xacro:humanoid_link name="left_thigh" length="0.4" radius="0.08" mass="5.0">
    <origin xyz="0 0 -0.2"/>
    <geometry>
      <cylinder length="0.4" radius="0.08"/>
    </geometry>
  </xacro:humanoid_link>

  <xacro:humanoid_joint name="left_knee_joint" type="revolute" 
                        parent="left_thigh" child="left_shin" 
                        xyz="0 0 -0.4" 
                        lower="0" upper="${M_PI}" 
                        effort="100" velocity="3"/>

  <xacro:humanoid_link name="left_shin" length="0.4" radius="0.08" mass="4.0">
    <origin xyz="0 0 -0.2"/>
    <geometry>
      <cylinder length="0.4" radius="0.08"/>
    </geometry>
  </xacro:humanoid_link>

  <!-- Gazebo-specific properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_thigh">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="left_shin">
    <material>Gazebo/Red</material>
  </gazebo>
</robot>
```

## Best Practices for Humanoid Modeling

### 1. Collision Mesh Optimization

For complex humanoid models, optimize collision meshes:

```xml
<!-- Use simplified collision geometry for performance -->
<link name="head_link">
  <!-- Visual geometry can be complex -->
  <visual>
    <geometry>
      <mesh filename="package://humanoid_description/meshes/head.dae"/>
    </geometry>
  </visual>
  
  <!-- Collision geometry should be simpler -->
  <collision>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
</link>
```

### 2. Proper Joint Limits and Dynamics

Set realistic joint limits and dynamics for humanoid joints:

```xml
<!-- Hip joint with appropriate limits -->
<joint name="left_hip_joint" type="revolute">
  <parent link="base_link"/>
  <child link="left_thigh"/>
  <origin xyz="0 0.1 -0.1"/>
  <axis xyz="0 1 0"/>
  <!-- Based on human hip range of motion -->
  <limit lower="${-M_PI/3}" upper="${M_PI/2}" effort="200" velocity="5"/>
  <!-- Include dynamics for more realistic simulation -->
  <dynamics damping="5.0" friction="1.0"/>
</joint>
```

### 3. Center of Mass Considerations

Position the base link's origin at the approximate center of mass for better simulation stability:

```xml
<!-- Position base link origin near the center of mass -->
<link name="base_link">
  <inertial>
    <!-- Mass of entire upper body -->
    <mass value="20.0"/>
    <!-- Center of mass at approximately 50cm height -->
    <origin xyz="0 0 0.5"/>
    <!-- Inertia tensor based on approximate shape -->
    <inertia ixx="1.5" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="0.8"/>
  </inertial>
  <!-- Visual origin relative to inertial origin -->
  <visual>
    <origin xyz="0 0 0.5"/>
    <geometry>
      <box size="0.4 0.3 0.6"/>
    </geometry>
  </visual>
</link>
```

## Validating Your Humanoid Model

Before using your model in complex simulations, validate it:

1. Check for URDF errors:
```bash
check_urdf /path/to/your/humanoid.urdf
```

2. Visualize the robot in RViz:
```bash
ros2 run rviz2 rviz2
```

3. Test the model in Gazebo:
```bash
gz sim -r -v 4 empty.sdf
# Then insert your model
```

Creating accurate and stable humanoid models in Gazebo requires careful attention to kinematics, dynamics, and Gazebo-specific extensions. In the next chapter, we'll explore physics engines and their impact on humanoid simulation.