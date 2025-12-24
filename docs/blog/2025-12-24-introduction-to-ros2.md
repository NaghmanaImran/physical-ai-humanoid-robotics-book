---
slug: introduction-to-ros2
title: "Getting Started with ROS 2: A Beginner's Guide to Robot Operating System 2"
authors: [default]
tags: [ros2, robotics, python, beginner]
---

# Getting Started with ROS 2: A Beginner's Guide to Robot Operating System 2

Robotics is an exciting field that's rapidly evolving, and ROS 2 (Robot Operating System 2) is at the heart of many modern robotic applications. Whether you're a student, hobbyist, or professional looking to break into robotics, understanding ROS 2 is an essential skill. In this guide, we'll explore what ROS 2 is, its core components, and how to get started with your first ROS 2 application.

<!-- truncate -->

## What is ROS 2 and Why is it Important?

ROS 2 (Robot Operating System 2) is not actually an operating system but rather a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Why ROS 2 Matters:

- **Standardization**: Provides common interfaces and tools that allow researchers and developers to share code and collaborate more effectively
- **Scalability**: Designed to work on everything from small embedded systems to large cloud-based robotic applications
- **Industry Adoption**: Widely used in industrial robotics, autonomous vehicles, and research institutions worldwide
- **Security**: Built-in security features make it suitable for commercial and industrial applications
- **Real-time Support**: Improved real-time capabilities compared to its predecessor

## Core Components of ROS 2

ROS 2 consists of several fundamental building blocks that work together to create robotic applications:

### 1. Nodes

Nodes are individual processes that perform computation. They are the basic building blocks of a ROS 2 program. Multiple nodes can work together to form a complete robotic application.

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Camera    │    │   Planner   │    │  Controller │
│    Node     │───▶│    Node     │───▶│    Node     │
└─────────────┘    └─────────────┘    └─────────────┘
```

### 2. Topics and Messages

Topics are named buses over which nodes exchange messages. Messages are the data packets sent between nodes. This implements a publish-subscribe communication pattern.

- **Publisher**: A node that sends messages to a topic
- **Subscriber**: A node that receives messages from a topic

### 3. Services

Services provide a request/reply communication pattern. A client sends a request to a service and waits for a response.

### 4. Actions

Actions are used for long-running tasks that provide feedback during execution. They're ideal for tasks like navigation where you want to monitor progress and potentially cancel the operation.

## Simple Examples with Python Code

Let's look at some basic examples to understand these concepts better:

### Simple Publisher Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Simple Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Getting Started Guide

### Installation

ROS 2 can be installed on Ubuntu, Windows, and macOS. Here's how to install ROS 2 Humble Hawksbill (the latest LTS version) on Ubuntu:

```bash
# Setup locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
```

### Setting up Your First Publisher/Subscriber

1. **Create a new workspace:**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. **Source ROS 2:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Create a package:**
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_robot_tutorials
   ```

4. **Add the publisher code to `my_robot_tutorials/my_robot_tutorials/publisher_member_function.py`**

5. **Add the subscriber code to `my_robot_tutorials/my_robot_tutorials/subscriber_member_function.py`**

6. **Build your package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_tutorials
   ```

7. **Source your workspace:**
   ```bash
   source install/setup.bash
   ```

8. **Run the publisher and subscriber in separate terminals:**
   ```bash
   # Terminal 1
   ros2 run my_robot_tutorials publisher_member_function
   
   # Terminal 2
   ros2 run my_robot_tutorials subscriber_member_function
   ```

You should now see the publisher sending messages and the subscriber receiving them!

## Benefits of Learning ROS 2

Learning ROS 2 offers numerous advantages for your robotics career:

### Technical Benefits:
- **Modularity**: Build complex systems by connecting simple, reusable components
- **Simulation**: Extensive tools for simulating robots before deploying on hardware
- **Hardware Abstraction**: Write code that works across different robot platforms
- **Community Support**: Large community with extensive documentation and packages

### Career Benefits:
- **Industry Standard**: Many robotics companies require ROS 2 experience
- **Research Applications**: Widely used in academic and research institutions
- **Cross-Platform**: Works on various operating systems and hardware platforms
- **Skill Transferability**: Knowledge applies to many different robotic systems

### Learning Benefits:
- **Best Practices**: Learn software engineering best practices applied to robotics
- **Problem Solving**: Develop solutions for common robotics challenges
- **Integration**: Understand how different components of a robot work together

## Conclusion

ROS 2 is a powerful framework that has become essential for robotics development. While it may seem complex at first, understanding its core concepts—nodes, topics, services, and actions—will give you a solid foundation for building sophisticated robotic applications.

Start with simple examples like the publisher/subscriber pattern, and gradually move to more complex systems. The robotics community is welcoming and supportive, so don't hesitate to seek help and contribute your own solutions.

Ready to dive deeper? Explore the official ROS 2 tutorials and consider joining the ROS Discourse forum to connect with other robotics enthusiasts and professionals.

---

*This post provides an introduction to ROS 2 for beginners. As you progress, you'll discover more advanced features like parameters, launch files, and complex message types that will expand your capabilities as a roboticist.*