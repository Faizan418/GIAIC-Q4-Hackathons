---
title: Curriculum Overview - Modules and Chapter Structure
sidebar_position: 3
description: Comprehensive overview of the four-module robotics curriculum structure, progression, and capstone integration
---

# Curriculum, Modules, and Chapter Overview

This document provides a comprehensive overview of the four-module robotics curriculum structure, explaining how modules progress from foundational concepts to advanced integration, and how they connect to real-world robotic capabilities. The curriculum is designed with a capstone-first approach, where students understand the complete system they'll build from the beginning. This approach ensures that students appreciate the relevance of each foundational concept as they progress through the modules, understanding how each piece contributes to the larger robotic system they will ultimately develop.

The curriculum has been carefully structured to balance theoretical understanding with practical application, ensuring students not only learn the concepts but also gain hands-on experience implementing them. Each module builds systematically on previous knowledge while introducing new challenges and capabilities, creating a learning journey that progresses from basic robotic communication to advanced cognitive systems.

## Curriculum Philosophy

The robotics curriculum follows a systems thinking approach, where students learn to understand robots as integrated systems rather than collections of independent components. This approach emphasizes how different technologies work together to create capable robotic systems.

The curriculum is designed with a "capstone-first" philosophy, where students understand from the beginning what they're working toward. This approach provides motivation and context for each module, helping students see how foundational concepts build toward advanced capabilities. By starting with an understanding of the complete system, students can better appreciate why each foundational concept is important and how it fits into the larger picture.

Progressive skill building is central to the curriculum design. Each module builds upon previous knowledge while introducing new concepts, creating a logical sequence from basic to advanced robotics capabilities. This approach ensures that students develop both theoretical understanding and practical skills in a structured manner. The progression is carefully calibrated to introduce complexity gradually, allowing students to master fundamental concepts before moving to more advanced topics.

## Module 1: ROS 2 Foundations

Module 1 introduces students to ROS 2 (Robot Operating System 2), which serves as the foundational communication framework for all subsequent modules. Students learn the core concepts of ROS 2 including nodes, topics, services, and actions that enable different components of a robotic system to communicate effectively.

The module covers essential ROS 2 concepts such as the client library architecture, message passing mechanisms, parameter management, and launch systems. Students will understand how ROS 2 provides the middleware that connects all components of a robotic system, from sensors to actuators to high-level planning systems.

This module is critical as it provides the essential communication infrastructure that all other modules depend upon. Without a solid understanding of ROS 2, students would struggle to integrate the perception, navigation, and action components introduced in later modules.

## Module 2: Digital Twins

Module 2 focuses on digital twins and physics-based simulation, providing students with virtual environments to test and validate robotic systems safely and cost-effectively. Students learn to create and work with simulation environments that accurately model real-world physics, sensor behavior, and environmental conditions.

The module covers simulation frameworks like Gazebo and Isaac Sim, physics engines, sensor simulation, and environment creation. Students will understand how to develop, test, and validate robotic algorithms in virtual environments before deploying them on real hardware.

Digital twins are essential for robotics development as they allow for rapid iteration, testing of edge cases, and validation of complex behaviors without the risks and costs associated with real-world testing. This module prepares students for the sim-to-real transfer challenges they'll encounter in later modules.

## Module 3: Perception & Navigation

Module 3 introduces students to perception systems and navigation capabilities, teaching them how robots can understand their environment and move through it autonomously. Students learn computer vision techniques, sensor fusion, path planning algorithms, and obstacle avoidance strategies.

The module covers perception algorithms including object detection, recognition, and scene understanding. Students will also learn navigation stacks, SLAM (Simultaneous Localization and Mapping), path planning, and obstacle avoidance techniques that enable robots to operate in dynamic environments.

This module builds upon the ROS 2 foundation from Module 1 and leverages the simulation environment from Module 2 for testing and validation. The perception and navigation capabilities form the core of many robotic applications and provide essential components for the capstone system.

## Module 4: Vision-Language-Action

Module 4 represents the culmination of the curriculum, integrating vision, language, and action systems to create cognitive robotics capabilities. Students learn to implement voice interfaces, LLM-based task planning, and multimodal AI systems that enable natural human-robot interaction.

The module covers vision-language models, natural language processing, task decomposition, and cognitive architectures. Students will understand how to create systems that can interpret natural language commands and translate them into executable robotic actions.

This module integrates capabilities from all previous modules: the communication infrastructure from Module 1, simulation tools from Module 2, and perception/navigation from Module 3. It represents the advanced integration that demonstrates the full potential of the robotics curriculum.

## Capstone System Overview

The capstone system integrates components from all four modules into a unified voice-controlled robotic system. The system features a voice interface layer that processes natural language commands, an LLM-based planning engine that decomposes high-level tasks into executable sequences, and integrated perception and navigation systems.

Core components of the capstone include the voice interface for natural command input, the planning engine for task decomposition and sequence generation, navigation systems for mobility, perception systems for environmental awareness, action execution for physical interaction, simulation capabilities for testing, and the ROS 2 middleware connecting all components.

![Capstone Architecture Flow](/img/sense-think-act-loop.svg)

The capstone architecture flow diagram shows how components from each module contribute to the integrated system. The voice interface from Module 4 connects to the LLM-based planning engine, which orchestrates the navigation and perception systems from Module 3, while leveraging the simulation capabilities from Module 2 and the ROS 2 communication infrastructure from Module 1.

The capstone system demonstrates sim-to-real capabilities, allowing students to develop and test their systems in simulation before deploying them on real hardware. This approach ensures safety while providing a realistic development experience.

## Weekly Mapping

### Module 1: Weeks 1-3
- Week 1: ROS 2 basics, workspace setup, nodes and topics
- Week 2: Services, actions, launch files, parameters
- Week 3: ROS 2 tools, debugging, and system architecture

### Module 2: Weeks 4-6
- Week 4: Simulation environments, Gazebo/Isaac Sim basics
- Week 5: Physics modeling, sensor simulation, environment creation
- Week 6: Integration with ROS 2, testing frameworks

### Module 3: Weeks 7-10
- Week 7: Perception fundamentals, computer vision with ROS 2
- Week 8: Navigation stack, path planning, obstacle avoidance
- Week 9: Sensor fusion, SLAM concepts
- Week 10: Learning algorithms, adaptive behaviors

### Module 4: Weeks 11-13
- Week 11: Vision-language models, multimodal AI
- Week 12: Task planning, cognitive architectures
- Week 13: Human-robot interaction, voice interfaces

### Capstone Integration: Weeks 14-16
- Week 14: System integration, component testing
- Week 15: End-to-end testing, sim-to-real validation
- Week 16: Final project, presentation, and assessment

## Curriculum Progression Map

![Curriculum Progression Map](/img/digital-ai-to-physical-ai-pipeline.svg)

The curriculum progression map illustrates how each module builds upon the previous one, creating a systematic learning path from foundational ROS 2 concepts to advanced cognitive robotics capabilities. The diagram shows the prerequisite relationships and skill building progression that enables students to develop competency systematically.

## Module Prerequisites

The curriculum follows a clear prerequisite structure to ensure students build knowledge systematically:

- Module 1 (ROS 2) serves as the essential foundation for all subsequent modules, providing the communication infrastructure that all other components depend upon.
- Module 2 (Digital Twins) requires Module 1 knowledge for ROS 2 integration.
- Module 3 (Perception & Navigation) depends on Module 1 for communication and Module 2 for testing.
- Module 4 (Vision-Language-Action) requires all previous modules for full system integration.

This progression ensures students develop competency systematically, with each module building upon previous knowledge while preparing for advanced integration.

## Real-World Applications

Each module connects to specific real-world robotic applications:

**Module 1 (ROS 2)**: Industrial automation, multi-robot systems, distributed robotics, middleware architecture for complex robotic systems.

**Module 2 (Digital Twins)**: Autonomous vehicle testing, surgical robot validation, warehouse automation simulation, safety-critical system validation.

**Module 3 (Perception & Navigation)**: Autonomous navigation, object recognition, warehouse robotics, search and rescue robotics, agricultural robotics.

**Module 4 (Vision-Language-Action)**: Assistive robotics, customer service robots, collaborative robots (cobots), educational robotics, social robotics.

## Chapter Intent Descriptions

Each chapter within the curriculum modules has specific learning objectives designed to build toward the comprehensive capstone system:

**Module 1 Chapters:**
- **ROS 2 Basics**: Students will understand the fundamental concepts of ROS 2 architecture, including nodes, topics, and services
- **Advanced ROS 2**: Students will master complex communication patterns and system integration techniques
- **ROS 2 Tools**: Students will gain proficiency in debugging and monitoring tools for robotic systems

**Module 2 Chapters:**
- **Simulation Environments**: Students will learn to create and configure realistic simulation environments
- **Physics Modeling**: Students will understand how to model real-world physics accurately in simulation
- **Validation Techniques**: Students will learn to validate simulation results against real-world data

**Module 3 Chapters:**
- **Perception Fundamentals**: Students will master computer vision and sensor fusion techniques
- **Navigation Systems**: Students will implement path planning and obstacle avoidance algorithms
- **Environmental Interaction**: Students will create systems that can interact with dynamic environments

**Module 4 Chapters:**
- **Multimodal Interfaces**: Students will develop systems that integrate vision, language, and action
- **Cognitive Architectures**: Students will implement LLM-based planning and reasoning systems
- **Human-Robot Interaction**: Students will create intuitive interfaces for natural human-robot collaboration

## Sim-to-Real Readiness

The curriculum emphasizes sim-to-real transfer capabilities, ensuring students understand how to develop systems that can transition from simulation to real-world deployment. This approach includes learning about domain randomization, reality gap mitigation, and validation techniques.

Students learn to develop and test their algorithms in simulation environments before deploying on real hardware, reducing development time and safety risks. The curriculum includes techniques for validating sim-to-real transfer and ensuring system reliability in both environments.
