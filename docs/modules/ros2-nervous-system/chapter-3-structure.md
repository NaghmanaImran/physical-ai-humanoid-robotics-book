---
title: Robot Structure and Control Interfaces
sidebar_label: Chapter 3
description: Introduction to URDF, robot structure modeling, and connecting AI agents to ROS 2
---

# Robot Structure and Control Interfaces

## Introduction

In the previous chapters, we explored ROS 2 as the nervous system of humanoid robots and examined its communication model. This chapter focuses on how robot structure is represented in ROS 2, specifically through the Unified Robot Description Format (URDF), and how Python AI agents interface with ROS 2 systems.

## Introduction to URDF (Unified Robot Description Format)

URDF is an XML-based format that describes robot models in ROS. It defines the physical and visual properties of a robot, including:

- **Links**: Rigid parts of the robot (like arms, legs, torso)
- **Joints**: Connections between links that allow motion
- **Visual**: How the robot appears visually
- **Collision**: How the robot interacts with its environment in collision detection

URDF serves as the digital blueprint for robots in ROS, analogous to how DNA contains the blueprint for biological organisms.

### Links

Links represent the rigid parts of a robot. Each link has properties such as:

- **Mass**: Physical mass of the link
- **Inertia**: How the link resists rotational motion
- **Visual**: How the link appears visually (shape, material, etc.)
- **Collision**: How the link interacts with other objects in collision detection

### Joints

Joints define the connection between two links and specify how they can move relative to each other. Joint types include:

- **Fixed**: No movement between links
- **Revolute**: Single axis rotation (like a hinge)
- **Continuous**: Unlimited single axis rotation
- **Prismatic**: Single axis translation
- **Floating**: 6-DOF freedom
- **Planar**: Motion on a plane

### Frames

URDF establishes a kinematic tree with a base frame and transforms between all parts. This allows ROS to understand the spatial relationships between different parts of the robot.

## How Humanoid Bodies are Modeled

### Links, Joints, and Frames

Humanoid robots are modeled by creating a kinematic chain that mimics human anatomy:

- **Torso**: The main body of the robot, typically serving as the root of the kinematic tree
- **Head**: Connected to the torso with joints that allow for looking around
- **Arms**: Each with shoulder, elbow, and wrist joints to enable manipulation
- **Legs**: Each with hip, knee, and ankle joints for locomotion
- **Hands**: With multiple joints for fine manipulation tasks

### Example Structure

A simplified humanoid robot might have the following structure:

```
base_link (torso)
├── head
├── upper_arm_left
│   ├── lower_arm_left
│   └── gripper_left
├── upper_arm_right
│   ├── lower_arm_right
│   └── gripper_right
├── upper_leg_left
│   └── lower_leg_left
└── upper_leg_right
    └── lower_leg_right
```

Each connection between these parts is defined by joints that specify the type of movement allowed.

## Relationship between URDF, ROS 2 Nodes, and Controllers

### URDF and ROS 2 Integration

URDF files are loaded into ROS 2 using the robot_state_publisher node, which:

- Reads the URDF file
- Publishes static transforms for the robot's structure
- Enables other nodes to understand the robot's kinematic structure

### Robot State Publisher

The robot_state_publisher node takes the URDF and joint states to publish the complete state of the robot's kinematic tree. This enables:

- Forward kinematics: calculating end-effector position from joint angles
- Inverse kinematics: calculating joint angles to achieve a desired end-effector position
- Collision detection and avoidance

### Controllers

Controllers in ROS 2 manage the actual movement of robot joints. They:

- Subscribe to command topics (e.g., joint positions, velocities, or efforts)
- Interface with the physical hardware
- Publish feedback about actual joint states

Controllers work in conjunction with the URDF to ensure movements are physically possible and safe.

## Bridging Python AI Agents to ROS 2 using rclpy

### Conceptual Flow

Python AI agents connect to ROS 2 systems through the rclpy library, which provides the Python client interface for ROS 2. The connection flow involves:

1. **Initialization**: The AI agent initializes as a ROS 2 node
2. **Subscriptions**: The AI agent subscribes to relevant topics (sensor data, robot state)
3. **Processing**: The AI agent processes information using AI algorithms
4. **Commands**: The AI agent publishes commands to control topics
5. **Feedback**: The AI agent receives feedback from the system to adjust its behavior

### rclpy Integration

rclpy enables Python AI agents to:

- Create ROS 2 nodes that can communicate with other system components
- Subscribe to sensor data streams for perception
- Publish commands to control the robot
- Use services for request-response interactions
- Implement actions for complex, long-running behaviors

### Example Integration Concept

A Python AI agent might follow this conceptual flow:

1. Initialize as a ROS 2 node using rclpy
2. Subscribe to sensor topics (camera images, IMU data, joint states)
3. Process sensor data using AI algorithms to make decisions
4. Publish commands to control topics (joint positions, velocities)
5. Monitor feedback and adjust behavior accordingly

## Preparing for Simulation and Navigation

### Digital Twin Connection

The URDF model serves as the foundation for creating digital twins of physical robots in simulation environments like Gazebo or Isaac. The same URDF that describes the physical robot is used to create an accurate virtual representation.

### Simulation Integration

The combination of URDF, ROS 2 nodes, and AI agents enables:

- **Testing**: AI algorithms can be tested in safe virtual environments
- **Validation**: Robot behaviors can be validated before deployment on physical hardware
- **Training**: AI agents can be trained in simulation before being transferred to physical robots

### Navigation Preparation

The structured approach of URDF, ROS 2 communication, and AI agent integration prepares learners for:

- **Path planning**: Using the robot's structural model to plan safe paths
- **Motion planning**: Generating joint trajectories that respect the robot's kinematic constraints
- **Control**: Implementing AI-driven control strategies that work with the robot's physical capabilities

## Conclusion

The integration of URDF, ROS 2 nodes, and controllers provides the foundation for representing and controlling humanoid robots. This structure enables Python AI agents to interface with physical robots through rclpy, bridging the gap between AI logic and physical robot movement.

Understanding this relationship is crucial for developing AI systems that can effectively control humanoid robots, preparing learners for advanced topics in simulation and navigation in subsequent modules.