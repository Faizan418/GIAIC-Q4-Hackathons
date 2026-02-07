# Data Model: Module 2 — Digital Twin: Simulation, Physics, and Virtual Worlds

**Feature**: 001-digital-twin-sim
**Date**: 2025-12-14
**Status**: Complete

## Content Structure Model

### Chapter Entity
- **name**: String (e.g., "Gazebo Setup and Simulation")
- **target_length**: Range (5000-6000 words)
- **sections**: Array of Section entities
- **code_examples**: Array of CodeExample entities
- **diagrams**: Array of Diagram entities
- **lab**: Lab entity
- **validation_rules**: Word count, content depth, practical applicability

### Section Entity
- **title**: String (e.g., "Gazebo Core Components")
- **word_count**: Integer (target 600-1000 words per section for 6-8 sections per chapter)
- **content_type**: Enum ("conceptual", "practical", "tutorial", "reference")
- **prerequisites**: Array of prerequisite Section entities
- **validation_rules**: Technical accuracy, educational value, ROS 2 integration relevance

### CodeExample Entity
- **title**: String (e.g., "ROS 2 Launch File for Gazebo")
- **language**: String ("xml", "python", "c++", "bash", "urdf")
- **purpose**: String (educational objective)
- **complexity**: Enum ("basic", "intermediate", "advanced")
- **validation_rules**: Must run successfully in simulation environment, must demonstrate real concept

### Diagram Entity
- **title**: String (e.g., "Gazebo Simulation Pipeline")
- **type**: Enum ("mermaid", "svg", "flowchart", "architecture")
- **purpose**: String (educational objective)
- **elements**: Array of related concepts/components
- **validation_rules**: Must accurately represent system architecture, must be educational

### Lab Entity
- **title**: String (e.g., "Launch and Control Simulated Robot")
- **duration**: Integer (estimated completion time in minutes)
- **prerequisites**: Array of Section entities
- **objectives**: Array of learning objectives
- **steps**: Array of procedural steps
- **validation_rules**: Must be achievable with provided content, must demonstrate key concepts

## Simulation-Specific Models

### Gazebo World Model
- **name**: String
- **description**: String
- **physics_engine**: Enum ("ode", "bullet", "simbody")
- **models**: Array of URDF Model entities
- **lighting**: Environmental lighting properties
- **validation_rules**: Must load successfully, must be appropriate for educational use

### URDF Robot Model
- **name**: String
- **links**: Array of Link entities
- **joints**: Array of Joint entities
- **inertial_properties**: Mass, center of mass, inertia tensor
- **collision_geometry**: Collision mesh definitions
- **visual_geometry**: Visual mesh definitions
- **sensors**: Array of Sensor entities
- **validation_rules**: Must be physically realistic, must simulate properly in Gazebo

### Sensor Model
- **type**: Enum ("camera", "lidar", "imu", "gps", "force_torque")
- **topic_name**: String (ROS 2 topic for data publication)
- **update_rate**: Float (Hz)
- **range**: Float (for range-based sensors)
- **resolution**: Float or Vector3 (for cameras)
- **noise_model**: Noise characteristics
- **validation_rules**: Must publish realistic data, must integrate properly with ROS 2

## Validation Rules

1. **Chapter Validation**: Each chapter must be 5000-6000 words with 2+ code examples and 2+ diagrams
2. **Code Validation**: All code examples must be tested in simulation environment
3. **Educational Validation**: Content must follow simulation-first approach
4. **Integration Validation**: All examples must work with ROS 2 and Gazebo integration
5. **Unity Validation**: Unity integration examples must demonstrate clear connection to Gazebo simulation
