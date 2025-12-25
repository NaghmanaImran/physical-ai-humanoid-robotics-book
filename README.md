# Physical AI & Humanoid Robotics Textbook

An AI-native technical textbook created for Panaversity Hackathon using Spec-Kit Plus and Docusaurus.

## Overview

This comprehensive textbook covers the essential technologies and methodologies for developing Physical AI and humanoid robotics systems. It provides both theoretical foundations and practical implementation guidance for creating sophisticated robotic systems that integrate perception, reasoning, and action in real-world environments.

## Purpose

The textbook aims to bridge the gap between traditional robotics and modern AI techniques, focusing on embodied intelligence where physical form and environmental interaction are fundamental to the emergence of intelligent behavior. It prepares students and practitioners to build next-generation humanoid robots capable of complex interactions with humans and environments.

## Target Audience

- Graduate students in robotics, AI, and computer science
- Researchers working in physical AI and humanoid robotics
- Engineers developing robotic systems
- AI practitioners interested in embodied intelligence
- Anyone interested in the intersection of AI and robotics

## Key Features

- **Spec-driven development with multiple numbered specs**: Complete specifications for each module following Spec-Kit Plus methodology
- **Full coverage of core technologies**:
  - ROS 2: The middleware framework for robotics applications
  - Digital Twin (Gazebo & Unity): Physics simulation and testing environment
  - NVIDIA Isaac: AI-powered perception and manipulation platform
  - VLA (Vision-Language-Action): Multimodal AI systems for robot cognition
- **Capstone Project**: Integrated project combining all modules to create an autonomous humanoid robot
- **Hands-on Labs**: Practical exercises for each module with real-world applications
- **AI-native approach**: Emphasis on AI integration throughout all robotics components
- **Interactive RAG Chatbot**: Floating search button for querying the textbook content
- **Live demo**: Available at https://naghmanaimran.github.io/physical-ai-humanoid-robotics-book/

## Modules

1. **Introduction**: Foundations of Physical AI and Humanoid Robotics
2. **Module 1: ROS 2**: The Robotic Nervous System
3. **Module 2: Digital Twin**: Simulation with Gazebo and Unity
4. **Module 3: NVIDIA Isaac**: The AI-Robot Brain
5. **Module 4: VLA**: Vision-Language-Action Integration
6. **Capstone Project**: Autonomous Humanoid Implementation

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
