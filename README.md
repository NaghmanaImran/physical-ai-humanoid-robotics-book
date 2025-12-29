# AI Robotics Textbook: Building Intelligent Physical Systems

A comprehensive, open-source textbook exploring the intersection of artificial intelligence and robotics for next-generation autonomous systems.

## Overview

This textbook provides a thorough exploration of the technologies and methodologies required to develop intelligent robotic systems that can perceive, reason, and act in real-world environments. It combines theoretical foundations with practical implementation guidance, focusing on creating embodied AI systems that demonstrate intelligent behavior through physical interaction with their environment.

## Purpose

Our mission is to bridge the gap between traditional robotics and modern AI techniques, emphasizing embodied intelligence where the physical form and environmental interaction are fundamental to the emergence of intelligent behavior. This resource prepares students and practitioners to build next-generation robotic systems capable of complex interactions with humans and environments.

## Target Audience

- Graduate students in robotics, AI, and computer science
- Researchers working in physical AI and embodied intelligence
- Engineers developing robotic systems
- AI practitioners interested in embodied intelligence
- Anyone interested in the intersection of AI and robotics

## Key Features

- **Spec-driven development**: Complete specifications for each module following modern development methodologies
- **Comprehensive technology coverage**:
  - ROS 2: The middleware framework for robotics applications
  - Digital Twin (Gazebo & Unity): Physics simulation and testing environment
  - NVIDIA Isaac: AI-powered perception and manipulation platform
  - VLA (Vision-Language-Action): Multimodal AI systems for robot cognition
- **Capstone Project**: Integrated project combining all modules to create an autonomous robotic system
- **Hands-on Labs**: Practical exercises for each module with real-world applications
- **AI-native approach**: Emphasis on AI integration throughout all robotics components
- **Interactive Learning Tools**: Search functionality and navigation aids for enhanced learning
- **Open Source**: Community-driven development and continuous improvement

## Modules

1. **Introduction**: Foundations of Physical AI and Robotics
2. **Module 1: ROS 2**: The Robotic Nervous System
3. **Module 2: Digital Twin**: Simulation with Gazebo and Unity
4. **Module 3: NVIDIA Isaac**: The AI-Robot Brain
5. **Module 4: VLA**: Vision-Language-Action Integration
6. **Capstone Project**: Autonomous Robotic System Implementation

## Prerequisites

- Basic programming experience (Python/C++)
- Understanding of linear algebra and calculus
- Familiarity with machine learning concepts
- Basic knowledge of physics and mechanics (helpful but not required)

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
