---
title: ROS 2 Communication Model for Humanoid Control
sidebar_label: Chapter 2
description: Deep dive into nodes, publishers, subscribers, and services for humanoid robot control
---

# ROS 2 Communication Model for Humanoid Control

## Introduction

In the previous chapter, we established ROS 2 as the "nervous system" of humanoid robots. This chapter delves deeper into the communication model that enables data flow between sensors, controllers, and actuators. Understanding this communication model is crucial for bridging AI agents with physical robot movement.

## Deep Dive into Communication Components

### Nodes, Publishers, and Subscribers

The publish-subscribe pattern is fundamental to ROS 2's communication model. Let's explore how this works in the context of humanoid robot control:

- **Publishers**: Nodes that send data to topics (like a sensory organ sending signals to the brain)
- **Subscribers**: Nodes that receive data from topics (like a motor center receiving commands from the brain)
- **Topics**: Named channels through which data flows between publishers and subscribers

This pattern enables decoupled communication, where publishers don't need to know which subscribers exist, and subscribers don't need to know which publishers are sending data.

### Services and Actions

In addition to the publish-subscribe pattern, ROS 2 provides service-based communication for request-response interactions. Services are useful for operations requiring immediate responses or acknowledgments.

For longer-running tasks with feedback, ROS 2 provides Actions, which extend the service concept with additional features like progress feedback and goal preemption.

## Data Flow in Humanoid Robots

### Sensor Data Flow

Sensors in humanoid robots (cameras, IMUs, force/torque sensors) publish their data to specific topics. For example:

- Camera sensors publish image data to `/camera/image_raw`
- IMU sensors publish orientation and acceleration data to `/imu/data`
- Force/torque sensors publish joint force data to `/ft_sensor/wrist`

Multiple nodes can subscribe to these sensor data streams simultaneously, enabling parallel processing of information.

### Control Command Flow

Control algorithms (potentially running AI agents) process sensor data and generate commands for the robot's actuators. These commands are published to specific topics:

- Joint position commands go to `/joint_group_position_controller/command`
- Joint velocity commands go to `/joint_group_velocity_controller/command`
- Cartesian position commands go to `/cartesian_position_controller/command`

### Example Communication Flow

Let's consider a simple scenario where an AI agent controls a humanoid robot's arm to reach for an object:

1. Camera sensors publish image data to `/camera/image_raw`
2. Perception node subscribes to image data and detects the target object
3. Perception node publishes the object's location to `/target_position`
4. Motion planning node subscribes to target position and current robot state
5. Motion planning node computes a trajectory and publishes commands to `/joint_group_position_controller/command`
6. Controller node receives commands and sends them to the physical actuators

## ROS 2 Executors and Lifecycle Nodes

### Executors

Executors manage the execution of callbacks from subscriptions, services, and timers. Different executor types provide various capabilities:

- **Single-threaded executor**: Processes all callbacks sequentially in one thread
- **Multi-threaded executor**: Processes callbacks in parallel using a thread pool
- **Static single-threaded executor**: Optimized for a fixed set of entities

### Lifecycle Nodes

Lifecycle nodes provide a structured approach to managing node states, which is particularly important in humanoid robots where reliability is crucial. The lifecycle includes:

- **Unconfigured**: Node loaded but not configured
- **Inactive**: Node configured but not active
- **Active**: Node is running and processing data
- **Finalized**: Node is shutting down

This lifecycle ensures that nodes transition through appropriate states in a controlled manner, improving system reliability.

## High-Level Examples Using Python

While we're focusing on conceptual understanding rather than detailed implementation, here's a simplified example of how a publisher and subscriber might be structured:

```python
# Publisher example (conceptual)
import rclpy
from std_msgs.msg import String

def create_sensor_publisher():
    # Initialize ROS 2
    rclpy.init()
    
    # Create node
    node = rclpy.create_node('sensor_publisher')
    
    # Create publisher
    publisher = node.create_publisher(String, 'sensor_data', 10)
    
    # Publish sensor data
    msg = String()
    msg.data = 'sensor_reading'
    publisher.publish(msg)
```

```python
# Subscriber example (conceptual)
import rclpy
from std_msgs.msg import String

def create_command_subscriber():
    # Initialize ROS 2
    rclpy.init()
    
    # Create node
    node = rclpy.create_node('command_subscriber')
    
    # Create subscription
    subscription = node.create_subscription(
        String,
        'robot_commands',
        callback_function,
        10)
```

## Conclusion

The ROS 2 communication model provides a robust framework for controlling humanoid robots by enabling flexible, decoupled communication between components. The publish-subscribe pattern allows for efficient data flow between sensors, controllers, and actuators, while services and actions provide mechanisms for request-response interactions and long-running operations.

Understanding this communication model is essential for effectively bridging AI agents with physical robot movement, which we'll explore further in the next chapter on robot structure and control interfaces.