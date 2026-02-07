---
title: Technical Setup & Lab Architecture Guide
description: Complete guide to setting up infrastructure for Physical AI & Humanoid Robotics course
sidebar_position: 2
---

# Technical Setup & Lab Architecture Guide

This guide defines all infrastructure requirements for the Physical AI & Humanoid Robotics course, covering software stack, hardware requirements, and architectural decisions needed for successful course execution.

## Software Stack Overview

Our recommended software stack balances performance, compatibility, and educational value:

- **OS**: Ubuntu 22.04 LTS - Optimal hardware driver support and robotics ecosystem compatibility
- **Middleware**: ROS 2 - Mature robotics framework for message passing and hardware abstraction
- **Simulation**: Gazebo - Realistic physics simulation with sensor modeling
- **AI Framework**: NVIDIA Isaac - Optimized platform for AI on NVIDIA hardware

This stack ensures compatibility with required hardware and provides proven tools for academic and industrial robotics applications.

## Digital Twin Workstation Requirements

Your digital twin workstation serves as the primary development environment requiring substantial computational resources for robotics simulation.

### Minimum Requirements
- **CPU**: Intel i7-10700K or AMD Ryzen 7 3700X (8 cores, 16 threads)
- **GPU**: NVIDIA RTX 3070 (8GB VRAM) or equivalent
- **RAM**: 32GB DDR4 (dual-channel)
- **Storage**: 1TB NVMe SSD
- **Network**: Gigabit Ethernet (1 Gbps)

### Ideal Requirements
- **CPU**: Intel i9-12900K or AMD Ryzen 9 5900X (16+ cores)
- **GPU**: NVIDIA RTX 4080/4090 (16GB+ VRAM) or RTX 6000 Ada
- **RAM**: 64GB DDR4/DDR5 (dual-channel)
- **Storage**: 2TB+ NVMe SSD
- **Network**: 2.5 Gbps or 10 Gbps Ethernet

The high computational requirements stem from real-time physics rendering, sensor simulation, and AI model training. The GPU is critical for both visual rendering and AI acceleration.

## High Compute Requirements Analysis

Physical AI applications demand high computational resources due to complex simulation, training, and real-time processing needs:

### Simulation Complexity
Robotic simulation requires modeling complex physical interactions in real-time with realistic sensor simulation and physics rendering. These computations must run at high frame rates (100Hz+) to maintain stable control loops, with accurate physics calculations to ensure behaviors learned in simulation transfer effectively to the real world.

### AI Training Requirements
Deep learning approaches require substantial computational resources for processing large datasets, running expensive neural network operations, and executing reinforcement learning algorithms that need thousands of simulation episodes.

### Real-Time Processing Demands
Physical AI systems must maintain real-time performance with low-latency sensor processing, rapid decision-making, precise motor control, and multi-modal sensor fusion.

## Architecture Integration

![Simulation to Edge to Robot Architecture](/img/sim-edge-robot-architecture.svg)

The above diagram shows the flow from simulation environment to edge AI processing to physical robot execution. This architecture enables safe development and testing in simulation before deployment to real hardware.

![Cloud Training to Local Inference Flow](/img/cloud-training-local-inference-flow.svg)

This flow shows how AI models are trained in high-compute cloud environments and then optimized for deployment to local edge devices on robots, ensuring optimal performance in real-world scenarios.

## Research-Backed Explanations

Technical requirements are based on extensive robotics research. Brooks' "Intelligence without Representation" highlights the need for tight perception-action-environment coupling, driving high computational requirements for real-time processing. Pfeifer and Bongard's work on morphological computation shows how physical form contributes to computational efficiency, supporting our emphasis on accurate simulation of physical properties.

Studies by Kober et al. on reinforcement learning for robotics highlight the substantial computational resources needed for learning in physical domains. Research in sim-to-real transfer demonstrates the importance of high-fidelity simulation environments for successful deployment to physical robots.

## Cloud vs Local Architecture

When setting up your lab environment, you'll face a decision between cloud-based and local architectures. Both approaches have distinct advantages and trade-offs.

### On-Premise Labs: Advantages
1. **Low Latency**: Direct connection eliminates network delays, critical for real-time robot control
2. **Full Control**: Complete administrative control over the environment, hardware, and software stack
3. **Data Privacy**: Sensitive data remains in your controlled environment
4. **Predictable Costs**: Upfront capital expenditure with predictable operational costs

### Cloud Labs: Advantages
1. **Scalability**: Easy to scale compute resources up or down based on needs
2. **Accessibility**: Access from anywhere with internet connection
3. **Reduced Maintenance**: No need to maintain physical infrastructure

### On-Prem vs Cloud Lab Comparison

| Factor | On-Premise | Cloud |
|--------|------------|-------|
| Initial Investment | High hardware costs | Minimal upfront investment |
| Ongoing Costs | Lower after initial setup | Recurring subscription fees |
| Performance | Consistent, high-bandwidth | Variable, network-dependent |
| Latency | Minimal (local processing) | Higher (network-dependent) |
| Maintenance | Full responsibility | Provider responsibility |
| Scalability | Limited by physical hardware | Highly scalable |

### Failure Points and Latency Risks

**On-Premise Risks:**
- Hardware failures requiring local maintenance
- Power and cooling infrastructure requirements

**Cloud Risks:**
- Network connectivity disruptions
- Bandwidth limitations affecting real-time control
- Service provider outages

## Warnings and Constraints

### System Compatibility Constraints

**Ubuntu Requirement**: The entire software stack has been validated exclusively on Ubuntu 22.04 LTS. While other operating systems may work, significant compatibility issues are likely with Windows or macOS.

**GPU Requirements**: NVIDIA RTX cards are required for full functionality. AMD or Intel graphics cards will not support the CUDA-based operations required for simulation and AI processing. Specifically, RTX 3070 or higher is required.

**Memory Minimums**: Systems with less than 32GB of RAM will experience significant performance degradation during complex simulations. For optimal performance, 64GB is recommended.

### Performance Warnings

**Network Latency**: If using any cloud-based components or remote robot control, network latency exceeding 50ms will significantly impact performance. For real-time robot control, local processing is essential.

**Simulation Accuracy**: Using hardware below the minimum specifications will result in unrealistic simulation behavior, potentially causing learned behaviors to fail when transferred to real robots.

### Critical Warnings

⚠️ **Warning**: Do not attempt to run complex Gazebo simulations with less than 32GB RAM. This will cause system instability and crashes that may result in data loss.

⚠️ **Warning**: Avoid mixing different generations of NVIDIA GPUs in the same system. This can cause driver conflicts and unpredictable behavior in CUDA applications.

⚠️ **Warning**: Never rely solely on cloud infrastructure for real-time robot control. Network disruptions can cause dangerous robot behavior and potential damage to equipment or injury.

## Jetson Edge AI Kits

The Jetson Orin series provides the ideal platform for edge AI processing in robotics applications.

### Jetson Orin Series Specifications

#### Jetson Orin NX
- **GPU**: 2048-core NVIDIA Ampere architecture GPU
- **CPU**: 8-core NVIDIA Arm Cortex-A78AE v8.2 64-bit CPU
- **DL Accelerator**: 2x NVIDIA Deep Learning Accelerator (DLA)
- **Memory**: 8GB LPDDR5
- **Power**: 10W - 25W
- **Best for**: Lightweight robots, drones, and portable AI applications

#### Jetson Orin AGX
- **GPU**: 2048-core NVIDIA Ampere architecture GPU
- **CPU**: 8-core NVIDIA Arm Cortex-A78AE v8.2 64-bit CPU
- **DL Accelerator**: 2x NVIDIA Deep Learning Accelerator (DLA)
- **Memory**: 32GB LPDDR5
- **Power**: 15W - 60W
- **Best for**: Full-sized humanoid robots, complex perception tasks

### Sensor Stack Requirements

Robots require a comprehensive sensor stack to perceive and interact with their environment effectively:

#### Primary Sensors
- **Camera System**: Stereo cameras for depth perception, RGB for color recognition
- **IMU (Inertial Measurement Unit)**: Accelerometer, gyroscope, and magnetometer for orientation and motion tracking
- **Audio System**: Microphones for sound detection and speech recognition
- **LiDAR**: Light Detection and Ranging for precise distance measurement and mapping

#### Secondary Sensors
- **Force/Torque Sensors**: For manipulation tasks and contact detection
- **Temperature Sensors**: For monitoring environmental conditions

### Robot Lab Options

Depending on your needs and budget, several robot lab configurations are available:

#### Proxy Robot Lab (Budget Option)
- **Hardware**: Simple mobile base with basic sensors
- **Purpose**: Algorithm development and testing
- **Advantages**: Low cost, easy maintenance

#### Humanoid Robot Lab (Mid-range Option)
- **Hardware**: Humanoid robot platform with full sensor suite
- **Purpose**: Advanced AI and robotics research
- **Advantages**: Closest to real-world humanoid applications

#### Premium Robot Lab (Full Capability)
- **Hardware**: Multiple advanced robot platforms with full lab setup
- **Purpose**: Comprehensive research and development
- **Advantages**: Full range of testing capabilities

### Workstation, Edge, and Robot Roles Definition

- **Digital Twin Workstation**: High-performance PC for simulation, development, and testing. Handles complex computations that don't require real-time response.
- **Jetson Edge AI Kit**: Embedded computer on the robot for real-time AI processing, sensor fusion, and immediate decision making.
- **Robot Platform**: Physical platform that executes commands, interacts with the environment, and provides real-world feedback.

This separation allows for efficient development workflows with complex algorithms developed and tested in simulation before deployment to the physical robot, while maintaining real-time responsiveness through local edge processing.

## Hardware Tiers Table

| Tier | CPU | GPU | RAM | Storage | Network | Use Case | Price Range |
|------|-----|-----|-----|---------|---------|----------|-------------|
| Student | i7-10700K | RTX 3070 | 32GB | 1TB NVMe | Gigabit | Coursework, Projects | $2,500-$3,500 |
| Research | i9-12900K | RTX 4080 | 64GB | 2TB NVMe | 2.5 Gbps | Advanced Research | $5,000-$7,000 |
| Production | i9-14900K | RTX 6000 Ada | 128GB | 4TB NVMe+ | 10 Gbps | Production Systems | $10,000+ |

## Cost Comparison Table

| Solution Type | Initial Cost | Ongoing Cost | Performance | Latency | Maintenance | Best For |
|---------------|--------------|--------------|-------------|---------|-------------|----------|
| On-Premise High-End | $5,000-$10,000 | Low | Highest | Minimal | High | Research Labs |
| On-Premise Mid-Range | $2,500-$5,000 | Low | Good | Minimal | Medium | Education |
| Cloud-Based | $0-$500 setup | $200-$500/month | Variable | Network-dependent | Low | Prototyping |
| Hybrid | $2,000-$5,000 | $100-$300/month | Flexible | Mixed | Medium | Development |

## Architecture Responsibility Matrix

| Component | Development | Testing | Deployment | Monitoring | Maintenance |
|-----------|-------------|---------|------------|------------|-------------|
| Simulation Environment | Workstation | Workstation | N/A | Workstation | Workstation |
| Edge AI Processing | Workstation | Workstation+Edge | Robot | Robot | Edge Kit |
| Physical Robot | N/A | N/A | Robot | Robot | Robot |
| Cloud Services | N/A | N/A | Cloud | Cloud | Cloud |

## References

[1] Brooks, R. A. (1991). Intelligence without representation. Artificial Intelligence, 47(1-3), 139-159.

[2] Pfeifer, R., & Bongard, J. (2006). How the body shapes the way we think: A new view of intelligence. MIT Press.

[3] Clark, A. (2008). Supersizing the mind: Embodiment, action, and cognitive extension. Oxford University Press.

[4] Kober, J., Bagnell, J. A., & Peters, J. (2013). Reinforcement learning in robotics: A survey. The International Journal of Robotics Research, 32(11), 1238-1274.

[5] Koos, S., Mouret, J. B., & Doncieux, S. (2013). Crossing the reality gap in evolutionary robotics by promoting transferable controllers. Artificial Life, 19(1), 109-128.

[6] NVIDIA Corporation. (2023). Jetson Orin Series Product Guide. Retrieved from https://developer.nvidia.com/embedded/jetson-orin

[7] Open Robotics. (2023). ROS 2 Documentation and Best Practices. Retrieved from https://docs.ros.org/

[8] Mozilla Foundation. (2023). Docusaurus Documentation Framework. Retrieved from https://docusaurus.io/

[9] Ubuntu Community. (2023). Ubuntu for Robotics Applications. Retrieved from https://ubuntu.com/robotics

[10] Gazebo Sim Team. (2023). Gazebo Simulation Environment Best Practices. Retrieved from https://gazebosim.org/
