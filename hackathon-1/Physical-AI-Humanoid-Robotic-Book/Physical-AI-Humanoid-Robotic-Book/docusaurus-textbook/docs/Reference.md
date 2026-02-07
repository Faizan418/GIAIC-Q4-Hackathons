---
sidebar_position: 4
title: '04 - Reference'
description: 'Comprehensive technical reference for ROS 2, Gazebo, Unity, Isaac, VLA systems, and hardware/software configuration tables'
---

# Reference Guide: Physical AI Development Frameworks

## Introduction

This reference guide provides comprehensive technical documentation for the key frameworks and tools used throughout the Physical AI curriculum. It serves as a quick reference for installation, configuration, and usage patterns for ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action (VLA) systems.

## ROS 2 Framework Reference

### Installation and Setup

ROS 2 (Robot Operating System 2) provides the communication infrastructure for robotics applications. The recommended distribution for this curriculum is ROS 2 Humble Hawksbill.

**System Requirements:**
- Ubuntu 22.04 LTS
- 8GB+ RAM recommended
- Multi-core processor
- Compatible GPU for visualization

**Installation Steps:**
```bash
# Setup locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions
sudo rosdep init
rosdep update
```

### Core Concepts

**Nodes:** Independent processes that perform computation. Nodes are organized in a distributed architecture and communicate via messages.

**Topics:** Named buses over which nodes exchange messages. Topics support many-to-many communication patterns.

**Services:** Synchronous request/response communication pattern between nodes.

**Actions:** Goal-oriented communication pattern with feedback and status updates.

### Common Commands

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# List available nodes
ros2 node list

# List available topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Call a service
ros2 service call /service_name std_srvs/srv/Empty

# Run a node
ros2 run package_name executable_name
```

## Gazebo Simulation Reference

### Installation and Setup

Gazebo is a physics-based simulation environment for robotics development. The recommended version is Gazebo Garden.

**Installation:**
```bash
# Add Gazebo's package repository
wget https://packages.osrfoundation.org/gazebo.gpg -O /tmp/gazebo.gpg
if [ ! -z "$GITHUB_ACTIONS" ]; then
  sudo -E gpg --dearmor -o /usr/share/keyrings/gazebo-archive.gpg /tmp/gazebo.gpg
else
  gpg --dearmor -o /usr/share/keyrings/gazebo-archive.gpg /tmp/gazebo.gpg
fi
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-archive.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install
sudo apt update
sudo apt install gz-garden
```

### Core Components

**World Files:** XML files that define the simulation environment including models, lighting, and physics properties.

**Models:** 3D representations of objects with physical properties and visual appearance.

**Plugins:** Custom code that extends Gazebo functionality for sensors, controllers, or custom behaviors.

### Common Usage Patterns

```bash
# Launch Gazebo with a world file
gz sim -r my_world.sdf

# Launch Gazebo GUI
gz sim gui

# List running simulations
gz service -s /gazebo/worlds
```

## Unity Integration Reference

### Installation and Setup

Unity provides 3D visualization and simulation capabilities. For Physical AI applications, Unity 2022.3 LTS is recommended.

**System Requirements:**
- Windows 10/11, macOS 10.14+, or Linux Ubuntu 18.04+
- 8GB+ RAM
- DirectX 10, OpenGL 3.3, or Vulkan-compatible GPU
- 15GB+ available disk space

**Installation:**
1. Download Unity Hub from unity.com
2. Install Unity 2022.3 LTS through Unity Hub
3. Install additional modules as needed (Linux Build Support, etc.)

### Unity Robotics Package

The Unity Robotics package provides integration between Unity and ROS 2:

```bash
# In Unity Package Manager
com.unity.robotics.ros-tcp-connector
com.unity.robotics.urdf-importer
```

### Key Components

**ROS TCP Connector:** Enables communication between Unity and ROS 2 nodes via TCP/IP.

**URDF Importer:** Imports robot models from ROS URDF files into Unity.

**Robotics Simulation Tools:** Physics simulation, sensor simulation, and environment tools.

## NVIDIA Isaac Reference

### Isaac Sim Installation

NVIDIA Isaac Sim provides high-fidelity simulation for robotics with GPU acceleration.

**Prerequisites:**
- NVIDIA GPU with RTX or GTX 1080+ (8GB+ VRAM)
- NVIDIA Driver 470+
- CUDA 11.8+
- Isaac Sim compatible with your ROS 2 distribution

**Installation:**
1. Download Isaac Sim from NVIDIA Developer website
2. Extract to desired location
3. Install dependencies:
```bash
cd isaac-sim
python3 -m pip install -e .
```

### Core Features

**Omniverse Platform:** Real-time 3D simulation and collaboration platform.

**PhysX Integration:** High-fidelity physics simulation.

**Synthetic Data Generation:** Tools for generating training data with perfect annotations.

**ROS 2 Bridge:** Direct integration with ROS 2 ecosystem.

### Common Usage

```bash
# Launch Isaac Sim
./isaac-sim/python.sh -m omni.isaac.kit --exec scripts/my_script.py

# Launch with ROS 2 bridge
./isaac-sim/python.sh -m omni.isaac.robohive --scene my_scene.usd
```

## Vision-Language-Action (VLA) Systems Reference

### Architecture Overview

VLA systems integrate perception, cognition, and action capabilities:

**Vision Component:** Processes RGB-D data for object detection, scene understanding, and environment mapping.

**Language Component:** Interprets natural language commands and generates appropriate responses.

**Action Component:** Plans and executes physical actions based on perception and language understanding.

### Key Technologies

**Large Language Models (LLMs):** Foundation models for language understanding and generation.

**Vision Transformers:** Models for image understanding and object detection.

**Reinforcement Learning:** Training methods for action optimization.

**Multi-Modal Fusion:** Techniques for combining vision and language information.

## Hardware Configuration Reference

### Recommended Robot Platforms

| Platform | DOF | Payload | Applications | Notes |
|----------|-----|---------|--------------|-------|
| Tesla Optimus | 28+ | 20kg | General purpose | Research platform |
| Boston Dynamics Atlas | 28+ | 10kg | Dynamic locomotion | Advanced mobility |
| Engineered Arts Ameca | 51+ | 5kg | Humanoid interaction | Social robotics |
| Unitree H1 | 19 | 15kg | Research | Cost-effective |

### Computing Requirements

| Component | Minimum | Recommended | Notes |
|-----------|---------|-------------|-------|
| CPU | 8-core @ 2.5GHz | 16-core @ 3.0GHz | Multi-threading critical for real-time performance |
| GPU | RTX 3080 | RTX 4090 | Required for perception and LLM inference |
| RAM | 32GB | 64GB+ | Large models require significant memory |
| Storage | 1TB SSD | 2TB+ NVMe | Fast storage for model loading |

### Sensor Specifications

| Sensor Type | Purpose | Requirements | Notes |
|-------------|---------|--------------|-------|
| RGB-D Camera | Perception | 1080p@30fps, depth accuracy &lt;2cm | Intel RealSense, Azure Kinect |
| IMU | Balance/Orientation | 6-axis, 100Hz+ | Critical for humanoid balance |
| Force/Torque | Safe interaction | 6-axis, high precision | Wrist/ankle mounted |
| Joint Encoders | Position feedback | High resolution | Integrated with actuators |

## Software Configuration Tables

### ROS 2 Packages

| Package | Purpose | Installation | Version |
|---------|---------|--------------|---------|
| ros-humble-desktop | Core ROS 2 | apt | 2.7.0+ |
| ros-humble-moveit | Motion planning | apt | 2.5.0+ |
| ros-humble-navigation2 | Navigation | apt | 1.2.0+ |
| ros-humble-isaac-ros-* | Isaac integration | source | Latest |
| ros-humble-vision-* | Perception | apt/source | Latest |

### Simulation Environments

| Environment | Purpose | Setup Complexity | Performance |
|-------------|---------|------------------|-------------|
| Gazebo Garden | Physics simulation | Low | Good |
| Isaac Sim | High-fidelity simulation | High | Excellent |
| Unity | Visual simulation | Medium | Good |
| Webots | Multi-robot simulation | Low | Medium |

## Troubleshooting Common Issues

### ROS 2 Communication Issues

**Problem:** Nodes cannot communicate across machines
**Solution:** Ensure ROS_DOMAIN_ID matches and network configuration is correct
```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0  # For multi-machine setup
```

### Simulation Performance

**Problem:** Low frame rate in simulation
**Solution:** Reduce physics update rate or simplify scene complexity
```xml
<!-- In world file -->
<physics>
  <max_step_size>0.01</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

### GPU Memory Issues

**Problem:** Out of memory errors with LLMs or perception models
**Solution:** Use model quantization or reduce batch sizes
```python
# For LLM inference
model = AutoModel.from_pretrained("model", torch_dtype=torch.float16)
```

## Best Practices

### Code Organization
- Use ROS 2 packages for logical grouping
- Follow standard ROS 2 naming conventions
- Document interfaces with comments and launch files
- Use parameters for configuration

### Simulation to Real Transfer
- Use domain randomization in simulation
- Validate performance in simulation before real-world testing
- Implement safety checks for real-world deployment
- Use identical control interfaces for simulation and reality

### Performance Optimization
- Profile code to identify bottlenecks
- Use multithreading for independent tasks
- Optimize sensor data processing pipelines
- Implement efficient state management

This reference guide should serve as your primary resource for technical details about the frameworks and tools used throughout the Physical AI curriculum. For more specific implementation details, refer to the individual module documentation.
