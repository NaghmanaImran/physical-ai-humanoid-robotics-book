---
sidebar_position: 3
---

# Gazebo Simulation Basics for Humanoid Robotics

## Understanding the Gazebo Interface

Gazebo provides both a graphical user interface (GUI) and command-line tools for simulation. Understanding these interfaces is crucial for effective humanoid robotics development.

### Gazebo GUI Components

When you launch Gazebo, you'll see several key components:

1. **3D Viewport**: The main window showing the simulated environment
2. **Scene Tree**: Shows all models and objects in the simulation
3. **Tools Panel**: Contains tools for interaction and modification
4. **Layer Tabs**: Different views (Physics, Rendering, etc.)
5. **Status Bar**: Shows simulation time and performance metrics

### Command Line Interface

Gazebo can also be controlled entirely through command line:

```bash
# Launch Gazebo with an empty world
gz sim -r -v 4 empty.sdf

# Launch with a specific world file
gz sim -r -v 4 my_world.sdf

# Launch without GUI (headless mode)
gz sim -s -r my_world.sdf

# Launch with specific parameters
gz sim -r -s -v 4 --iterations 1000 my_world.sdf
```

## Basic Simulation Concepts

### World Files (.sdf)

World files define the entire simulation environment:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
    
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include a light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Define a simple humanoid robot -->
    <model name="simple_humanoid">
      <pose>0 0 1 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>0.4</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.4</iyy>
            <iyz>0.0</iyz>
            <izz>0.4</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.3 0.6</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.3 0.6</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### Model Files (.sdf/.urdf)

Models represent individual objects in the simulation. For humanoid robots, URDF (Unified Robot Description Format) is commonly used with the help of xacro for complex models:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0.3" />
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.4" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.3"/>
      <geometry>
        <box size="0.3 0.3 0.6" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.3"/>
      <geometry>
        <box size="0.3 0.3 0.6" />
      </geometry>
    </collision>
  </link>

  <!-- Left leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0 0.1 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  </joint>

  <link name="left_hip">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
  </link>
</robot>
```

## Essential Simulation Parameters

### Physics Configuration

The physics engine parameters are critical for humanoid simulation:

```xml
<physics type="ode">
  <!-- Time step for simulation (smaller = more accurate but slower) -->
  <max_step_size>0.001</max_step_size>
  
  <!-- Real-time factor (1.0 = real-time, >1.0 = faster than real-time) -->
  <real_time_factor>1.0</real_time_factor>
  
  <!-- Update rate for physics calculations -->
  <real_time_update_rate>1000.0</real_time_update_rate>
  
  <!-- Gravity vector (x, y, z) -->
  <gravity>0 0 -9.8</gravity>
  
  <!-- ODE-specific parameters -->
  <ode>
    <solver>
      <type>quick</type>  <!-- or "pgs" for more stability -->
      <iters>10</iters>   <!-- Solver iterations -->
      <sor>1.3</sor>     <!-- Successive over-relaxation parameter -->
    </solver>
    <constraints>
      <cfm>0.0</cfm>      <!-- Constraint force mixing -->
      <erp>0.2</erp>      <!-- Error reduction parameter -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Performance vs. Accuracy Trade-offs

For humanoid robotics, you need to balance simulation accuracy with performance:

- **Time Step**: Smaller steps (0.001s) provide better accuracy for fast dynamics but require more computation
- **Real-time Factor**: Setting this to 1.0 ensures simulation runs at real-time speed for ROS integration
- **Solver Parameters**: Adjust for stability vs. performance (more iterations = more stable but slower)

## Basic Simulation Operations

### Controlling Simulation

```bash
# Pause simulation
gz service -s /world/default/control --req-type gz.msgs.WorldControl --req 'pause: true'

# Resume simulation
gz service -s /world/default/control --req-type gz.msgs.WorldControl --req 'pause: false'

# Reset simulation
gz service -s /world/default/control --req-type gz.msgs.WorldControl --req 'reset: true'

# Step simulation by a specific number of iterations
gz service -s /world/default/control --req-type gz.msgs.WorldControl --req 'step: true, multi_step: 100'
```

### Model Manipulation

```bash
# Spawn a model at a specific pose
gz service -s /world/default/create --req-type gz.msgs.EntityFactory --req 'sdf: "<sdf version=\"1.6\"><model name=\"test_model\"><link name=\"link\"><visual name=\"visual\"><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></sdf>", pose: {position: {x: 1, y: 2, z: 3}}'

# Remove a model
gz service -s /world/default/remove --req-type gz.msgs.Entity --req 'name: "test_model", type: MODEL'
```

## Humanoid-Specific Simulation Considerations

### Center of Mass and Stability

For humanoid robots, the center of mass (CoM) is critical for stability:

```xml
<!-- Example of properly configuring CoM for a humanoid base -->
<link name="base_link">
  <inertial>
    <mass>10.0</mass>
    <!-- Position the CoM appropriately for stability -->
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <inertia>
      <!-- Diagonal inertia matrix -->
      <ixx>0.4</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.4</iyy>
      <iyz>0.0</iyz>
      <izz>0.4</izz>
    </inertia>
  </inertial>
</link>
```

### Joint Limits and Dynamics

Humanoid joints require careful configuration of limits and dynamics:

```xml
<joint name="hip_joint" type="revolute">
  <parent link="base_link"/>
  <child link="thigh_link"/>
  <origin xyz="0 0.1 -0.3"/>
  <axis xyz="0 1 0"/>
  <!-- Joint limits based on human anatomy -->
  <limit lower="-1.57" upper="1.57" effort="200" velocity="5.0"/>
  <!-- Joint dynamics -->
  <dynamics damping="1.0" friction="0.1"/>
</joint>
```

### Contact and Friction Parameters

For stable humanoid locomotion, contact parameters are crucial:

```xml
<collision name="foot_collision">
  <geometry>
    <box size="0.2 0.1 0.05"/>
  </geometry>
  <!-- Surface parameters for stable contact -->
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>  <!-- Coefficient of friction -->
        <mu2>1.0</mu2>
        <fdir1>0 0 0</fdir1>
        <slip1>0.0</slip1>
        <slip2>0.0</slip2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.01</restitution_coefficient>
      <threshold>100000</threshold>
    </bounce>
    <contact>
      <ode>
        <soft_cfm>0.000001</soft_cfm>
        <soft_erp>0.2</soft_erp>
        <kp>1000000000000.0</kp>
        <kd>1000000000000.0</kd>
        <max_vel>100.0</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

## Running Your First Humanoid Simulation

Create a simple world file for humanoid testing:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_test">
    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Simple humanoid model -->
    <model name="simple_humanoid">
      <pose>0 0 1 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>10.0</mass>
          <origin xyz="0 0 0.3"/>
          <inertia>
            <ixx>0.4</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.4</iyy>
            <iyz>0.0</iyz>
            <izz>0.4</izz>
          </inertia>
        </inertial>
        <visual name="base_visual">
          <geometry>
            <box>
              <size>0.3 0.3 0.6</size>
            </box>
          </geometry>
        </visual>
        <collision name="base_collision">
          <geometry>
            <box>
              <size>0.3 0.3 0.6</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

Save this as `humanoid_test.sdf` and run:

```bash
gz sim -r -v 4 humanoid_test.sdf
```

Understanding these simulation basics is essential for creating effective humanoid robotics simulations in Gazebo. In the next chapter, we'll explore more advanced modeling techniques for humanoid robots.