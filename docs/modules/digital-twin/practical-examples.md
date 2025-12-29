# Practical Examples in Digital Twin Robotics

## Overview

This chapter provides practical examples that demonstrate the integration of all concepts covered in the Digital Twin module. These examples will help you apply Gazebo simulation, Unity integration, and physics engines in real-world scenarios.

## Example 1: Mobile Robot Navigation in Gazebo

### Scenario Description
Create a mobile robot that navigates through a complex environment using ROS2 navigation stack in Gazebo simulation.

### Step-by-Step Implementation

#### 1. Create the Robot Model (URDF)

First, create a differential drive robot model:

```xml
<?xml version="1.0"?>
<robot name="diff_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Properties -->
  <xacro:property name="base_width" value="0.6"/>
  <xacro:property name="base_length" value="0.8"/>
  <xacro:property name="base_height" value="0.2"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="wheel_x_offset" value="0.1"/>
  <xacro:property name="wheel_y_offset" value="0.3"/>

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
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.8" iyz="0.0" izz="1.2"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Base to left wheel joint -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="${wheel_x_offset} ${wheel_y_offset} 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Base to right wheel joint -->
  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="${wheel_x_offset} ${-wheel_y_offset} 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Add a caster wheel for stability -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="${-0.3} 0 ${-0.05}"/>
  </joint>

  <!-- Add a camera for perception -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.1 0.03"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.1 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${base_length/2 - 0.02} 0 ${base_height/2}"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="wheel_left">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <fdir1>1 0 0</fdir1>
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="wheel_right">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <fdir1>1 0 0</fdir1>
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="caster_wheel">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Grey</material>
  </gazebo>

  <!-- Camera sensor -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
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

  <!-- Differential drive plugin -->
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <update_rate>30</update_rate>
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>${2 * wheel_y_offset}</wheel_separation>
      <wheel_diameter>${2 * wheel_radius}</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_wheel_tf>false</publish_wheel_tf>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>
</robot>
```

#### 2. Create the World File

Create a world file with obstacles:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="navigation_world">
    <!-- Physics -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Walls -->
    <model name="wall_1">
      <pose>0 5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>100</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100</iyy>
            <iyz>0</iyz>
            <izz>100</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_2">
      <pose>0 -5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>100</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100</iyy>
            <iyz>0</iyz>
            <izz>100</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_3">
      <pose>5 0 1 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>100</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100</iyy>
            <iyz>0</iyz>
            <izz>100</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_4">
      <pose>-5 0 1 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>100</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100</iyy>
            <iyz>0</iyz>
            <izz>100</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Obstacles -->
    <model name="obstacle_1">
      <pose>2 2 0.5 0 0 0</pose>
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
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="obstacle_2">
      <pose>-2 -2 0.5 0 0 0</pose>
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
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="obstacle_3">
      <pose>0 3 0.5 0 0 0.785</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>0.5</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.5</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

#### 3. Launch the Simulation

Create a launch file to start the simulation:

```xml
<launch>
  <!-- Start Gazebo with the world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find your_robot_description)/worlds/navigation_world.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Spawn the robot -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" 
        args="-file $(find your_robot_description)/urdf/diff_drive_robot.urdf 
              -urdf -model diff_drive_robot -x 0 -y 0 -z 0.1" />

  <!-- Robot state publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" 
        type="robot_state_publisher" />
</launch>
```

#### 4. Test Navigation

Once the simulation is running, you can test navigation with:

```bash
# Send a simple velocity command
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2" -r 10
```

## Example 2: Unity Visualization of Gazebo Simulation

### Scenario Description
Create a Unity visualization that mirrors the Gazebo simulation in real-time, providing a high-fidelity visual representation.

### Implementation Steps

#### 1. Unity Scene Setup

Create a Unity scene with the same environment as the Gazebo world:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROS_TCPConnector.MessageTypes.Nav;
using Unity.Robotics.ROS_TCPConnector.MessageTypes.Std;

public class GazeboUnityBridge : MonoBehaviour
{
    [Header("Environment Setup")]
    public GameObject groundPlane;
    public GameObject[] walls;
    public GameObject[] obstacles;
    
    [Header("Robot Visualization")]
    public GameObject robotModel;
    public GameObject cameraModel;
    
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        // Subscribe to robot odometry
        ros.Subscribe<OdometryMsg>("odom", UpdateRobotPosition);
        
        // Subscribe to laser scan for visualization
        ros.Subscribe<LaserScanMsg>("scan", UpdateLaserScan);
        
        // Initialize environment
        SetupEnvironment();
    }
    
    void SetupEnvironment()
    {
        // Create ground plane
        groundPlane = GameObject.CreatePrimitive(PrimitiveType.Plane);
        groundPlane.transform.localScale = new Vector3(10, 1, 10);
        groundPlane.GetComponent<Renderer>().material.color = Color.gray;
        
        // Create walls (simplified representation)
        CreateWalls();
        
        // Create obstacles
        CreateObstacles();
    }
    
    void CreateWalls()
    {
        // Create 4 walls around the environment
        float wallHeight = 2f;
        float wallThickness = 0.2f;
        float envSize = 10f;
        
        // North wall
        GameObject northWall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        northWall.transform.position = new Vector3(0, wallHeight/2, envSize/2);
        northWall.transform.localScale = new Vector3(envSize, wallHeight, wallThickness);
        northWall.GetComponent<Renderer>().material.color = Color.gray;
        
        // South wall
        GameObject southWall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        southWall.transform.position = new Vector3(0, wallHeight/2, -envSize/2);
        southWall.transform.localScale = new Vector3(envSize, wallHeight, wallThickness);
        southWall.GetComponent<Renderer>().material.color = Color.gray;
        
        // East wall
        GameObject eastWall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        eastWall.transform.position = new Vector3(envSize/2, wallHeight/2, 0);
        eastWall.transform.localScale = new Vector3(wallThickness, wallHeight, envSize);
        eastWall.GetComponent<Renderer>().material.color = Color.gray;
        
        // West wall
        GameObject westWall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        westWall.transform.position = new Vector3(-envSize/2, wallHeight/2, 0);
        westWall.transform.localScale = new Vector3(wallThickness, wallHeight, envSize);
        westWall.GetComponent<Renderer>().material.color = Color.gray;
    }
    
    void CreateObstacles()
    {
        // Create obstacle 1
        GameObject obstacle1 = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle1.transform.position = new Vector3(2, 0.5f, 2);
        obstacle1.transform.localScale = new Vector3(1, 1, 1);
        obstacle1.GetComponent<Renderer>().material.color = Color.red;
        
        // Create obstacle 2
        GameObject obstacle2 = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle2.transform.position = new Vector3(-2, 0.5f, -2);
        obstacle2.transform.localScale = new Vector3(1, 1, 1);
        obstacle2.GetComponent<Renderer>().material.color = Color.green;
        
        // Create obstacle 3 (cylinder)
        GameObject obstacle3 = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        obstacle3.transform.position = new Vector3(0, 0.5f, 3);
        obstacle3.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f); // Scale Y affects height
        obstacle3.transform.rotation = Quaternion.Euler(90, 0, 0); // Rotate to stand upright
        obstacle3.GetComponent<Renderer>().material.color = Color.blue;
    }
    
    void UpdateRobotPosition(OdometryMsg odom)
    {
        // Update robot position in Unity
        robotModel.transform.position = new Vector3(
            (float)odom.pose.pose.position.x,
            (float)odom.pose.pose.position.z + 0.1f, // Add small offset to prevent sinking
            (float)odom.pose.pose.position.y
        );
        
        // Update robot rotation
        robotModel.transform.rotation = new Quaternion(
            (float)odom.pose.pose.orientation.x,
            (float)odom.pose.pose.orientation.z,
            (float)odom.pose.pose.orientation.y,
            (float)odom.pose.pose.orientation.w
        );
    }
    
    void UpdateLaserScan(LaserScanMsg scan)
    {
        // Visualize laser scan points
        for (int i = 0; i < scan.ranges.Length; i += 10) // Sample every 10th point for performance
        {
            float distance = (float)scan.ranges[i];
            if (distance < scan.range_max && distance > scan.range_min)
            {
                float angle = (float)(scan.angle_min + i * scan.angle_increment);
                
                Vector3 position = robotModel.transform.position + 
                    new Vector3(
                        distance * Mathf.Cos(angle),
                        0.1f, // Height above ground
                        distance * Mathf.Sin(angle)
                    );
                
                // Create a small sphere to represent the laser point
                GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                point.transform.position = position;
                point.transform.localScale = Vector3.one * 0.05f;
                point.GetComponent<Renderer>().material.color = Color.yellow;
                
                // Destroy after a few seconds to prevent clutter
                Destroy(point, 2.0f);
            }
        }
    }
}
```

#### 2. Enhanced Visualization

Add more sophisticated visualization elements:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROS_TCPConnector.MessageTypes.Sensor;

public class EnhancedVisualization : MonoBehaviour
{
    [Header("Visualization Settings")]
    public GameObject pathVisualization;
    public GameObject goalMarker;
    public GameObject laserScanVisualization;
    
    [Header("UI Elements")]
    public UnityEngine.UI.Text robotStatusText;
    public UnityEngine.UI.Text batteryLevelText;
    public UnityEngine.UI.Text positionText;
    
    private ROSConnection ros;
    private LineRenderer pathRenderer;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        // Setup path visualization
        SetupPathVisualization();
        
        // Subscribe to relevant topics
        ros.Subscribe<NavSatFixMsg>("/gps/fix", UpdateGPSPosition);
        ros.Subscribe<BatteryStateMsg>("/battery_state", UpdateBatteryStatus);
        ros.Subscribe<PathMsg>("/move_base/NavfnROS/plan", UpdatePathVisualization);
    }
    
    void SetupPathVisualization()
    {
        pathVisualization = new GameObject("PathVisualization");
        pathRenderer = pathVisualization.AddComponent<LineRenderer>();
        pathRenderer.material = new Material(Shader.Find("Sprites/Default"));
        pathRenderer.color = Color.green;
        pathRenderer.startWidth = 0.1f;
        pathRenderer.endWidth = 0.1f;
    }
    
    void UpdatePathVisualization(PathMsg path)
    {
        if (path.poses.Count > 0)
        {
            Vector3[] points = new Vector3[path.poses.Count];
            
            for (int i = 0; i < path.poses.Count; i++)
            {
                points[i] = new Vector3(
                    (float)path.poses[i].pose.position.x,
                    (float)path.poses[i].pose.position.z + 0.1f,
                    (float)path.poses[i].pose.position.y
                );
            }
            
            pathRenderer.positionCount = points.Length;
            pathRenderer.SetPositions(points);
        }
    }
    
    void UpdateGPSPosition(NavSatFixMsg gps)
    {
        positionText.text = $"Position: {gps.latitude:F6}, {gps.longitude:F6}";
    }
    
    void UpdateBatteryStatus(BatteryStateMsg battery)
    {
        batteryLevelText.text = $"Battery: {(int)(battery.percentage * 100)}%";
        
        // Change color based on battery level
        if (battery.percentage < 0.2f)
        {
            batteryLevelText.color = Color.red;
        }
        else if (battery.percentage < 0.5f)
        {
            batteryLevelText.color = Color.yellow;
        }
        else
        {
            batteryLevelText.color = Color.green;
        }
    }
}
```

## Example 3: Multi-Robot Coordination

### Scenario Description
Simulate multiple robots working together to accomplish a task, demonstrating coordination and communication.

### Implementation

#### 1. Multi-Robot World

Create a world file for multiple robots:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="multi_robot_world">
    <!-- Physics -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Area boundaries -->
    <model name="boundary_north">
      <pose>0 7.5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>15 0.1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>15 0.1 1</size></box>
          </geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
        <inertial><mass>100</mass><inertia><ixx>100</ixx><iyy>100</iyy><izz>100</izz></inertia></inertial>
      </link>
    </model>

    <model name="boundary_south">
      <pose>0 -7.5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>15 0.1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>15 0.1 1</size></box>
          </geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
        <inertial><mass>100</mass><inertia><ixx>100</ixx><iyy>100</iyy><izz>100</izz></inertia></inertial>
      </link>
    </model>

    <model name="boundary_east">
      <pose>7.5 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>15 0.1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>15 0.1 1</size></box>
          </geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
        <inertial><mass>100</mass><inertia><ixx>100</ixx><iyy>100</iyy><izz>100</izz></inertia></inertial>
      </link>
    </model>

    <model name="boundary_west">
      <pose>-7.5 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>15 0.1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>15 0.1 1</size></box>
          </geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
        <inertial><mass>100</mass><inertia><ixx>100</ixx><iyy>100</iyy><izz>100</izz></inertia></inertial>
      </link>
    </model>

    <!-- Robot 1 -->
    <include>
      <uri>model://diff_drive_robot</uri>
      <name>robot1</name>
      <pose>-3 0 0 0 0 0</pose>
    </include>

    <!-- Robot 2 -->
    <include>
      <uri>model://diff_drive_robot</uri>
      <name>robot2</name>
      <pose>3 0 0 0 0 3.14159</pose>
    </include>

    <!-- Robot 3 -->
    <include>
      <uri>model://diff_drive_robot</uri>
      <name>robot3</name>
      <pose>0 3 0 0 0 1.5708</pose>
    </include>

    <!-- Targets -->
    <model name="target_1">
      <pose>-5 -5 0.1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.3</radius><length>0.2</length></cylinder></geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.3</radius><length>0.2</length></cylinder></geometry>
        </collision>
        <inertial><mass>0.1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      </link>
    </model>

    <model name="target_2">
      <pose>5 5 0.1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.3</radius><length>0.2</length></cylinder></geometry>
          <material><ambient>0 1 0 1</ambient></material>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.3</radius><length>0.2</length></cylinder></geometry>
        </collision>
        <inertial><mass>0.1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      </link>
    </model>
  </world>
</sdf>
```

#### 2. Coordination Algorithm

Implement a simple coordination algorithm:

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf
import math

class MultiRobotCoordinator:
    def __init__(self):
        rospy.init_node('multi_robot_coordinator')
        
        # Robot names
        self.robots = ['robot1', 'robot2', 'robot3']
        
        # Publishers for each robot
        self.cmd_vel_pubs = {}
        for robot in self.robots:
            self.cmd_vel_pubs[robot] = rospy.Publisher(f'/{robot}/cmd_vel', Twist, queue_size=10)
        
        # Subscribers for robot positions
        self.robot_positions = {}
        for robot in self.robots:
            rospy.Subscriber(f'/{robot}/odom', Odometry, self.odom_callback, robot)
        
        # Target positions
        self.targets = [
            (-5, -5),  # Target 1
            (5, 5),    # Target 2
        ]
        
        # Assign targets to robots
        self.robot_targets = {}
        
        # Timer for coordination updates
        self.timer = rospy.Timer(rospy.Duration(0.1), self.coordination_callback)
        
    def odom_callback(self, msg, robot_name):
        # Store robot position
        self.robot_positions[robot_name] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
    
    def coordination_callback(self, event):
        # Simple assignment: assign closest robot to each target
        if len(self.robot_positions) == len(self.robots):
            # Calculate distances from each robot to each target
            distances = {}
            for robot, pos in self.robot_positions.items():
                distances[robot] = []
                for target in self.targets:
                    dist = math.sqrt((pos[0] - target[0])**2 + (pos[1] - target[1])**2)
                    distances[robot].append(dist)
            
            # Assign targets (simple greedy assignment)
            assigned_robots = set()
            assigned_targets = set()
            
            for target_idx, target in enumerate(self.targets):
                min_dist = float('inf')
                closest_robot = None
                
                for robot, pos in self.robot_positions.items():
                    if robot not in assigned_robots:
                        dist = distances[robot][target_idx]
                        if dist < min_dist:
                            min_dist = dist
                            closest_robot = robot
                
                if closest_robot:
                    self.robot_targets[closest_robot] = target
                    assigned_robots.add(closest_robot)
                    assigned_targets.add(target_idx)
            
            # Move each robot toward its assigned target
            for robot, target in self.robot_targets.items():
                if robot in self.robot_positions:
                    self.move_robot_to_target(robot, target)
    
    def move_robot_to_target(self, robot, target):
        # Get current robot position
        current_pos = self.robot_positions[robot]
        
        # Calculate direction to target
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]
        
        # Create velocity command
        cmd = Twist()
        cmd.linear.x = min(0.5, math.sqrt(dx**2 + dy**2))  # Scale speed based on distance
        cmd.angular.z = math.atan2(dy, dx)  # Turn toward target
        
        # Publish command
        self.cmd_vel_pubs[robot].publish(cmd)

if __name__ == '__main__':
    coordinator = MultiRobotCoordinator()
    rospy.spin()
```

## Example 4: Physics Simulation Comparison

### Scenario Description
Compare different physics engines (ODE, Bullet) to understand their impact on simulation accuracy and performance.

### Implementation

#### 1. Physics Comparison World

Create a world file that allows comparison:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="physics_comparison">
    <!-- ODE Physics -->
    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
      </ode>
    </physics>

    <!-- Environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Objects for comparison -->
    <model name="ode_ball">
      <pose>-2 0 2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><sphere><radius>0.1</radius></sphere></geometry>
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>0.1</radius></sphere></geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia>
        </inertial>
      </link>
    </model>

    <model name="bullet_ball">
      <pose>2 0 2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><sphere><radius>0.1</radius></sphere></geometry>
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>0.1</radius></sphere></geometry>
          <material><ambient>0 0 1 1</ambient></material>
        </visual>
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia>
        </inertial>
      </link>
    </model>

    <!-- Ramps for testing -->
    <model name="ode_ramp">
      <pose>-2 0 0.5 0 0.3 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>2 0.1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>2 0.1 1</size></box></geometry>
          <material><ambient>0.7 0 0 1</ambient></material>
        </visual>
        <inertial><mass>10</mass><inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz></inertia></inertial>
      </link>
    </model>

    <model name="bullet_ramp">
      <pose>2 0 0.5 0 0.3 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>2 0.1 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>2 0.1 1</size></box></geometry>
          <material><ambient>0 0 0.7 1</ambient></material>
        </visual>
        <inertial><mass>10</mass><inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz></inertia></inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Best Practices for Digital Twin Implementation

### 1. Model Fidelity vs Performance
- Balance model accuracy with simulation performance
- Use simplified models for real-time applications
- Implement level-of-detail (LOD) systems

### 2. Data Synchronization
- Ensure consistent time synchronization between systems
- Implement data buffering for network delays
- Use appropriate data compression techniques

### 3. Validation and Verification
- Compare simulation results with real-world data
- Implement systematic testing procedures
- Document model limitations and assumptions

### 4. Scalability Considerations
- Design systems that can handle multiple robots
- Optimize for the target hardware platform
- Consider cloud-based simulation for complex scenarios

## Chapter Summary

In this chapter, you learned how to implement practical examples that combine all aspects of digital twin technology:

1. How to create complete mobile robot navigation scenarios in Gazebo
2. How to integrate Unity for high-fidelity visualization
3. How to coordinate multiple robots in simulation
4. How to compare different physics engines
5. Best practices for implementing digital twin systems

These examples demonstrate the practical application of digital twin concepts in robotics, showing how simulation environments can be used to test algorithms, validate designs, and develop control strategies before deploying on physical hardware.

## Next Steps

With these practical examples, you now have a comprehensive understanding of digital twin technology in robotics. You can apply these concepts to create your own simulation environments for testing and validating robotic systems.

## Diagram Placeholders

[Image: Multi-Robot Coordination diagram showing multiple robots coordinating in a shared environment]

[Image: Physics Engine Comparison diagram comparing physics simulation results between different engines]