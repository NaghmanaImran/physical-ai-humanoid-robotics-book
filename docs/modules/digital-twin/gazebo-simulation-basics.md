# Simulation Basics in Gazebo

## Overview

This chapter covers the fundamental concepts of running simulations in Gazebo. You'll learn how to create and configure simulation environments, run basic simulations, and understand the core components that make up a Gazebo simulation.

## Understanding Gazebo Simulation Components

### World Files

World files define the complete simulation environment, including:
- Models (robots, objects)
- Physics properties
- Lighting and environment settings
- Plugins and controllers

A basic world file structure:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_world">
    <!-- Physics engine configuration -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Environment lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Robot model -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

    <!-- Other objects -->
    <model name="box">
      <pose>2 0 0.5 0 0 0</pose>
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
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.1</iyy>
            <iyz>0.0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Creating World Environments

### Basic World File

Create a simple world file named `simple_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_world">
    <!-- Physics engine -->
    <physics name="default_physics" type="ode">
      <gravity>0 0 -9.8</gravity>
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

    <!-- Simple robot -->
    <model name="simple_robot">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="chassis">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.5 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.5 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.1 1</ambient>
            <diffuse>0.8 0.2 0.1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.05</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.15</iyy>
            <iyz>0.0</iyz>
            <izz>0.15</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Running a World File

To run the world file:

```bash
gazebo simple_world.world
```

Or with verbose output:

```bash
gazebo --verbose simple_world.world
```

## Physics Simulation and Material Properties

### Physics Engine Configuration

Gazebo supports multiple physics engines. The most common is ODE (Open Dynamics Engine):

```xml
<physics name="default_physics" type="ode">
  <!-- Time step settings -->
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  
  <!-- Solver settings -->
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
```

### Material Properties

Materials define how objects look and interact physically:

```xml
<material name="blue">
  <ambient>0 0 0.8 1</ambient>
  <diffuse>0 0 0.8 1</diffuse>
  <specular>0.1 0.1 0.1 1</specular>
  <emissive>0 0 0 1</emissive>
</material>
```

## Sensor Simulation

### Camera Simulation

Add a camera sensor to your model:

```xml
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <camera name="cam">
    <horizontal_fov>1.047</horizontal_fov>
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
    <frame_name>camera_frame</frame_name>
    <topic_name>camera/image_raw</topic_name>
  </plugin>
</sensor>
```

### LIDAR Simulation

Add a LIDAR sensor:

```xml
<sensor name="laser" type="ray">
  <pose>0.2 0 0.1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
    <topic_name>laser_scan</topic_name>
    <frame_name>laser_frame</frame_name>
  </plugin>
</sensor>
```

## Running Simulations

### Basic Simulation Commands

1. **Start Gazebo with an empty world**:
   ```bash
   gazebo
   ```

2. **Start Gazebo with a specific world**:
   ```bash
   gazebo my_world.world
   ```

3. **Start with verbose output**:
   ```bash
   gazebo --verbose my_world.world
   ```

4. **Start paused**:
   ```bash
   gazebo --pause my_world.world
   ```

### Simulation Control

Once Gazebo is running, you can control the simulation:

- **Play/Pause**: Use the play/pause buttons in the GUI
- **Step**: Step through simulation one time step at a time
- **Reset**: Reset the simulation to its initial state

### Command Line Control

You can also control the simulation from the command line:

```bash
# Pause the simulation
gz service -s /world/simple_world/control --req-type gz.msgs.WorldControl --req 'pause: true'

# Resume the simulation
gz service -s /world/simple_world/control --req-type gz.msgs.WorldControl --req 'pause: false'

# Reset the simulation
gz service -s /world/simple_world/control --req-type gz.msgs.WorldControl --req 'reset: true'
```

## Controlling Models in Simulation

### Moving Models Programmatically

You can move models in the simulation using Gazebo services:

```bash
# Move a model to a specific pose
gz service -s /world/simple_world/set_pose --req-type gz.msgs.Pose --req 'name: "simple_robot", pose: {position: {x: 1, y: 1, z: 0.5}, orientation: {w: 1, x: 0, y: 0, z: 0}}'
```

### Applying Forces

Apply forces to models:

```bash
# Apply a force to a link
gz service -s /world/simple_world/apply_force --req-type gz.msgs.EntityWrench --req 'entity: {name: "simple_robot::chassis"}, wrench: {force: {x: 10, y: 0, z: 0}}'
```

## Simulation Parameters

### Time Settings

Understanding time settings is crucial for simulation performance:

- **Max Step Size**: The largest time step the physics engine will take (typically 0.001s)
- **Real Time Factor**: How fast the simulation runs compared to real time (1.0 = real-time)
- **Real Time Update Rate**: How many simulation steps per second (1000 = 1000 Hz)

### Performance Considerations

1. **Step Size**: Smaller steps are more accurate but slower
2. **Real Time Factor**: Values > 1 run faster than real-time
3. **Update Rate**: Higher rates are more accurate but more computationally expensive

## Creating Custom Environments

### Building a Room Environment

Create a world file for an indoor environment:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="indoor_room">
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Floor -->
    <model name="floor">
      <pose>0 0 0 0 0 0</pose>
      <link name="floor_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1000</mass>
          <inertia>
            <ixx>1000</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1000</iyy>
            <iyz>0</iyz>
            <izz>1000</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Walls -->
    <model name="wall_north">
      <pose>0 5 1.5 0 0 0</pose>
      <link name="wall_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 3</size>
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

    <!-- Add more walls as needed -->
  </world>
</sdf>
```

## Simulation Best Practices

### 1. Model Simplification
- Use simpler geometries for collision models than visual models
- Reduce the number of polygons in visual models when possible
- Use bounding boxes instead of complex shapes for collision detection

### 2. Physics Tuning
- Start with default physics parameters and adjust as needed
- Use smaller step sizes for more accurate simulation
- Balance accuracy with performance requirements

### 3. Environment Design
- Create environments that match your testing requirements
- Include appropriate lighting and visual elements
- Add reference objects for scale and orientation

### 4. Testing and Validation
- Test models in simple environments before complex ones
- Verify that physical properties are realistic
- Check that sensors provide expected data

## Debugging Simulations

### Common Issues

1. **Models Falling Through Ground**:
   - Check collision geometries
   - Verify inertial properties
   - Ensure proper mass values

2. **Unstable Simulation**:
   - Reduce step size
   - Adjust solver parameters
   - Check joint limits and constraints

3. **Performance Issues**:
   - Simplify collision geometries
   - Reduce the number of objects
   - Adjust physics parameters

### Debugging Tools

1. **Visualize Collision Shapes**:
   - In Gazebo GUI, enable "View" → "Transparent" to see collision shapes
   - Use "View" → "Wireframe" to see model structure

2. **Check Physics Properties**:
   - Use Gazebo's model inspector to verify properties
   - Check that mass and inertia values are reasonable

3. **Monitor Simulation Performance**:
   - Watch the real-time factor in the GUI
   - Use system monitoring tools to check resource usage

## Advanced Simulation Concepts

### Plugins

Gazebo plugins extend simulation capabilities:

```xml
<plugin name="model_plugin" filename="libMyModelPlugin.so">
  <param1>value1</param1>
  <param2>value2</param2>
</plugin>
```

### Services and Topics

Gazebo provides ROS2 interfaces for controlling simulation:

```bash
# List available topics
ros2 topic list

# Echo a topic to see data
ros2 topic echo /camera/image_raw sensor_msgs/msg/Image

# Publish to a topic
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

## Chapter Summary

In this chapter, you learned the basics of running simulations in Gazebo:

1. How to create and configure world files
2. How to set up physics simulation parameters
3. How to add and configure sensors
4. How to control simulations programmatically
5. Best practices for simulation design
6. How to debug common simulation issues

## Next Steps

Now that you understand basic simulation concepts, continue to the next chapter to learn about physics engines in more detail.

## Diagram Placeholders

[Image: Gazebo Simulation Components diagram showing the components of a Gazebo simulation]

[Image: World File Structure diagram showing the structure of a Gazebo world file]