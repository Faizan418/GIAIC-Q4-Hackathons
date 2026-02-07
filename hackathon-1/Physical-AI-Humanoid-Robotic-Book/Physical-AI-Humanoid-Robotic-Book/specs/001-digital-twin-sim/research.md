# Research: Module 2 — Digital Twin: Simulation, Physics, and Virtual Worlds

**Feature**: 001-digital-twin-sim
**Date**: 2025-12-14
**Status**: Complete

## Research Tasks Completed

### 1. Gazebo Simulation Architecture and Best Practices

**Decision**: Use Gazebo Classic (not Gazebo Garden/Harmonic) for better ROS 2 integration and educational stability
**Rationale**: Gazebo Classic has mature ROS 2 integration with better documentation for educational purposes
**Alternatives considered**: Gazebo Garden, Ignition Gazebo - rejected due to less stable ROS 2 integration for learning

### 2. Physics Engine Selection for Digital Twins

**Decision**: Support both ODE and Bullet physics engines with ODE as default for stability
**Rationale**: ODE is more stable for educational purposes, Bullet offers advanced features for complex scenarios
**Alternatives considered**: Simbody, DART - rejected due to complexity for beginners

### 3. Sensor Simulation Best Practices

**Decision**: Focus on camera, LiDAR, and IMU sensors as primary educational examples
**Rationale**: These sensors are fundamental to robotics and have good Gazebo support
**Alternatives considered**: GPS, magnetometer, force/torque sensors - will be mentioned but not primary focus

### 4. ROS 2 Integration Patterns

**Decision**: Use ROS 2 Humble Hawksbill (LTS) with standard gazebo_ros_pkgs for stable integration
**Rationale**: LTS version ensures stability for educational content
**Alternatives considered**: Rolling release - rejected due to potential instability

### 5. Unity Integration Approach

**Decision**: Use Unity Robotics Simulation package for ROS-TCP-Connector integration
**Rationale**: Provides official bridge between ROS 2 and Unity with good documentation
**Alternatives considered**: Custom TCP connections - rejected due to complexity for educational use

### 6. URDF Modeling Best Practices

**Decision**: Focus on proper inertial properties, collision vs visual geometry, and joint constraints
**Rationale**: These are critical for realistic simulation and often misunderstood by beginners
**Alternatives considered**: Simplified models - rejected as they don't teach proper simulation

### 7. Documentation and Educational Structure

**Decision**: Follow simulation-first approach with hands-on labs that build on each other
**Rationale**: Reduces real-world testing iterations and builds proper understanding
**Alternatives considered**: Theory-first approach - rejected as less effective for practical learning

## Technical Decisions Summary

- Use Gazebo Classic with ODE physics engine by default
- Implement standard ROS 2 sensor interfaces for simulated sensors
- Create comprehensive URDF examples with proper inertial properties
- Include Unity integration via ROS-TCP-Connector
- Structure content as progressive learning with applied labs
