# Quickstart Guide: Module 2 — Digital Twin: Simulation, Physics, and Virtual Worlds

**Feature**: 001-digital-twin-sim
**Date**: 2025-12-14
**Status**: Complete

## Prerequisites

Before starting with the Digital Twin simulation modules, ensure you have:

1. **ROS 2 Humble Hawksbill** installed (or latest LTS version)
2. **Gazebo Classic** (version 11.x) installed
3. **Docusaurus development environment** set up
4. **Basic understanding** of URDF and ROS 2 concepts (covered in Module 1)

## Setup Environment

### 1. Install Gazebo and ROS 2 Integration

```bash
# Install Gazebo Classic
sudo apt update
sudo apt install gazebo libgazebo-dev

# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control

# Install additional simulation tools
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim
```

### 2. Verify Installation

```bash
# Test Gazebo
gazebo --version

# Test ROS 2 integration
ros2 run gazebo_ros gazebo
```

### 3. Create Workspace Structure

```bash
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws
colcon build
source install/setup.bash
```

## First Simulation: Simple Robot

### 1. Create Basic Robot Model

Create a simple URDF robot file (`simple_robot.urdf`):

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Simple Wheel -->
  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint connecting wheel to base -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

### 2. Create Gazebo World File

Create `simple_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

### 3. Launch Simulation

```bash
# Launch Gazebo with your world
gazebo simple_world.world

# In another terminal, spawn your robot
ros2 run gazebo_ros spawn_entity.py -file simple_robot.urdf -entity simple_robot
```

## ROS 2 Integration

### 1. Create Launch File

Create `gazebo_simulation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'worlds',
                'simple_world.world'
            ])
        }.items()
    )
    ld.add_action(gazebo)

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True}]
    )
    ld.add_action(robot_state_publisher)

    return ld
```

### 2. Run Complete Simulation

```bash
# Build and source your workspace
cd ~/digital_twin_ws
colcon build
source install/setup.bash

# Launch the complete simulation
ros2 launch my_robot_description gazebo_simulation.launch.py
```

## Unity Integration (Overview)

For Unity integration, you'll need:

1. **Unity 2022.3 LTS** or later
2. **Unity Robotics Simulation package**
3. **ROS-TCP-Connector** for communication

### Basic Unity Setup

1. Create new Unity project
2. Import ROS-TCP-Connector package
3. Configure connection to ROS 2 network
4. Import URDF models using Unity URDF Importer

## Next Steps

After completing this quickstart:

1. **Chapter 06**: Deep dive into Gazebo setup, physics engines, and sensor simulation
2. **Chapter 07**: Explore URDF modeling, Unity integration, and human-robot interaction
3. **Applied Lab**: Build a complete digital twin with ROS 2 integration

Each chapter will expand on these concepts with detailed explanations, advanced examples, and practical applications following the simulation-first approach.
