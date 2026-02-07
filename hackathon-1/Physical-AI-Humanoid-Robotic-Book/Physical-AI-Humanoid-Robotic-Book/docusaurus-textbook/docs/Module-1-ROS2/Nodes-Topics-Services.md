---
title: Nodes, Topics, Services, and Actions
sidebar_position: 4
description: Mastering ROS 2 communication patterns - topics, services, and actions for effective robotic system design
---

# Nodes, Topics, Services, and Actions

## Introduction to ROS 2 Communication Patterns

ROS 2 provides three primary communication patterns that form the foundation of all robotic system interactions: topics for asynchronous data streaming, services for synchronous request/response communication, and actions for long-running tasks with feedback. Understanding when and how to use each pattern is crucial for designing effective robotic systems that can operate reliably in complex environments.

The communication patterns in ROS 2 are built on top of the DDS middleware, which provides the underlying infrastructure for message passing between nodes. Each pattern serves different purposes and has different characteristics in terms of reliability, timing, and complexity. The choice of communication pattern significantly impacts the design and performance of robotic systems.

Topics are ideal for continuous data streams such as sensor readings, status updates, and control commands. They provide a decoupled, asynchronous communication model where publishers and subscribers don't need to be synchronized in time. This makes topics perfect for real-time sensor data and status information that needs to be continuously updated.

Services provide synchronous request/response communication that is appropriate for discrete operations where a client needs to wait for a response from a server. This pattern is ideal for operations like requesting a map, saving parameters, or triggering a specific action that returns a result.

Actions extend the service pattern to handle long-running operations that may take seconds or minutes to complete. Actions provide feedback during execution and allow for cancellation, making them suitable for complex tasks like navigation, manipulation, or calibration procedures.

## Topics vs Services vs Actions Comparison

Understanding the differences between these three communication patterns is essential for effective system design. Each pattern has specific use cases, advantages, and limitations that make it appropriate for different scenarios.

### Topics: Asynchronous Data Streaming

Topics implement a publish-subscribe communication pattern that is fundamentally asynchronous. Publishers send messages without waiting for acknowledgment, and subscribers receive messages when they are available. This pattern provides several advantages:

**Decoupling**: Publishers and subscribers don't need to be synchronized in time. A publisher can send messages even when no subscribers are active, and subscribers can join the system at any time to receive new messages.

**Scalability**: Multiple publishers can publish to the same topic, and multiple subscribers can subscribe to the same topic, enabling flexible system architectures.

**Real-time performance**: The asynchronous nature of topics makes them well-suited for real-time applications where timing is critical.

**Broadcast capability**: A single message can be received by multiple subscribers simultaneously.

However, topics also have limitations:
- No guaranteed delivery: Messages may be lost if the communication system is overloaded
- No response mechanism: Publishers don't know if their messages are received or processed
- No acknowledgment: There's no way to confirm that subscribers have processed messages

### Services: Synchronous Request/Response

Services implement a synchronous request/response communication pattern where a client sends a request and waits for a response from a server. This pattern provides:

**Reliability**: Service calls provide guaranteed delivery and response, making them suitable for critical operations.

**Synchronization**: The client waits for the server to complete the operation before continuing, ensuring proper sequencing.

**Error handling**: Services can return specific error codes and messages, enabling robust error handling.

**Simple interface**: The request/response model is intuitive and easy to understand.

However, services have limitations:
- Blocking nature: The client is blocked while waiting for the response, which can impact system performance
- Not suitable for long-running operations: Clients may timeout if the server takes too long to respond
- Limited scalability: Each service call is a separate interaction, which can create bottlenecks

### Actions: Asynchronous Long-Running Operations

Actions combine elements of both topics and services to handle long-running operations that require feedback and cancellation. Actions provide:

**Feedback during execution**: Clients receive periodic updates on the progress of the operation.

**Cancellation capability**: Operations can be cancelled before completion if circumstances change.

**Goal tracking**: Clients can monitor the status of their requests and receive results when available.

**Asynchronous operation**: Unlike services, clients are not blocked while waiting for the operation to complete.

However, actions add complexity:
- More complex implementation: Both client and server sides require more code to handle the action interface
- Additional message types: Actions require separate message types for goals, feedback, and results
- Increased resource usage: Maintaining action state requires additional memory and processing

## Asynchronous Communication in ROS 2

Asynchronous communication is a fundamental aspect of ROS 2 that enables responsive, non-blocking robotic systems. Understanding how to effectively use asynchronous patterns is crucial for building systems that can handle multiple concurrent operations without blocking or deadlocking.

### Asynchronous Programming Models

ROS 2 provides several models for handling asynchronous operations:

**Callback-based**: The traditional ROS approach where callbacks are executed when messages arrive or services are called. This model is simple but can lead to complex callback hierarchies.

**Async/await**: Modern Python and C++ provide async/await patterns that can be used with ROS 2 for cleaner asynchronous code.

**Thread-based**: Multiple threads can be used to handle different aspects of communication, though this requires careful attention to thread safety.

**State machines**: Complex asynchronous operations can be modeled as state machines that transition between different states based on events.

### Callback Handling and Threading

ROS 2 provides flexible callback handling mechanisms that allow developers to control how callbacks are executed:

**Single-threaded execution**: All callbacks are executed in a single thread, simplifying synchronization but potentially limiting performance.

**Multi-threaded execution**: Callbacks can be executed in multiple threads to improve performance, but this requires careful attention to thread safety.

**Callback groups**: Nodes can organize their callbacks into groups that can have different execution characteristics, allowing fine-grained control over concurrency.

### Message Processing Pipelines

Complex robotic systems often process messages through multiple stages, each adding more semantic meaning or reducing uncertainty. Asynchronous processing allows these stages to operate independently while maintaining overall system responsiveness.

**Filtering**: Initial processing may filter out invalid or irrelevant messages before more expensive processing.

**Transformation**: Messages may need to be transformed between different coordinate frames or data representations.

**Integration**: Multiple messages may need to be integrated to create more complete information.

**Decision making**: Processed information is used to make decisions about robot behavior.

## Command and Control Patterns

Effective command and control in robotic systems requires careful consideration of communication patterns, timing, and reliability. Different types of commands require different communication approaches to ensure safe and reliable operation.

### Control Command Patterns

**Velocity Commands**: Continuous control commands like velocity commands are typically sent via topics using messages like geometry_msgs/Twist. These commands are sent at high frequency (typically 10-100 Hz) and represent the desired instantaneous motion of the robot.

**Position Commands**: Higher-level commands like desired positions or waypoints are often sent via services or actions, as they represent discrete goals rather than continuous control signals.

**Emergency Commands**: Safety-critical commands like emergency stops should use reliable communication with appropriate QoS settings to ensure they are delivered even under system stress.

### Feedback and Monitoring

Robotic systems need continuous feedback to monitor their state and adjust their behavior accordingly. Different types of feedback serve different purposes:

**State Feedback**: Current state information like position, velocity, and joint angles are typically published via topics for continuous monitoring.

**Status Feedback**: Higher-level status information like operational mode, error conditions, or task progress may be published via topics or services depending on the update frequency and reliability requirements.

**Diagnostic Information**: Detailed diagnostic information for debugging and maintenance may be available via services or actions when needed.

### Coordination and Synchronization

Complex robotic tasks often require coordination between multiple nodes or systems. This coordination can be achieved through various patterns:

**Leader-Follower**: One node coordinates the activities of multiple follower nodes, managing resource allocation and task scheduling.

**Decentralized Coordination**: Nodes coordinate directly with each other without a central coordinator, suitable for systems where no single point of failure is acceptable.

**Hierarchical Coordination**: Coordination occurs at multiple levels, with local coordinators managing subsets of nodes and higher-level coordinators managing the local coordinators.

## Publisher/Subscriber Implementation

The publisher/subscriber pattern is the most common communication pattern in ROS 2, used for continuous data streams like sensor readings, status updates, and control commands. Proper implementation of this pattern is crucial for system performance and reliability.

### Publisher Design

A well-designed publisher should consider several factors:

**Message Frequency**: The frequency at which messages are published should match the requirements of the application. Too frequent can overload the system, while too infrequent can miss important information.

**Message Content**: Messages should contain all necessary information without being unnecessarily large. Large messages can impact performance and network utilization.

**QoS Configuration**: Appropriate QoS policies should be selected based on the requirements for reliability, durability, and timing.

**Resource Management**: Publishers should properly manage resources and handle errors gracefully.

### Subscriber Design

Effective subscriber design involves:

**Message Processing**: Subscribers should process messages efficiently and handle errors without crashing the node.

**Buffer Management**: Subscribers should manage their message buffers appropriately to avoid memory issues or message loss.

**Timing Considerations**: Subscribers should handle timing issues like message delays or out-of-order delivery.

**Error Recovery**: Subscribers should be able to recover from errors and continue operation.

### Implementation Example

Here's a comprehensive example that demonstrates publisher/subscriber implementation with service calls for configuration:

```python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from example_interfaces.srv import SetBool, Trigger
from example_interfaces.msg import Float64MultiArray
import threading
import time
from collections import deque
import numpy as np


class RobotCommunicationNode(Node):
    """
    A comprehensive example demonstrating publisher/subscriber implementation
    with service calls for configuration and control.
    """

    def __init__(self):
        super().__init__('robot_communication_node')

        # Create callback groups for different types of callbacks
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self.control_callback_group = MutuallyExclusiveCallbackGroup()
        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        # Internal state
        self.robot_enabled = False
        self.safety_mode = False
        self.last_scan = None
        self.scan_buffer = deque(maxlen=10)  # Keep last 10 scans for analysis
        self.velocity_history = deque(maxlen=50)  # Keep velocity history
        self.obstacle_threshold = 1.0  # meters

        # Publishers
        self.cmd_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10,
            callback_group=self.control_callback_group
        )

        self.status_publisher = self.create_publisher(
            String,
            'robot_status',
            10
        )

        self.safety_publisher = self.create_publisher(
            Twist,
            'safety_cmd',
            10
        )

        self.sensor_processed_publisher = self.create_publisher(
            LaserScan,
            'processed_scan',
            10,
            callback_group=self.sensor_callback_group
        )

        self.velocity_history_publisher = self.create_publisher(
            Float64MultiArray,
            'velocity_history',
            10
        )

        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10,
            callback_group=self.sensor_callback_group
        )

        self.velocity_subscriber = self.create_subscription(
            Twist,
            'cmd_vel_input',
            self.velocity_callback,
            10,
            callback_group=self.control_callback_group
        )

        # Services
        self.enable_service = self.create_service(
            SetBool,
            'enable_robot',
            self.enable_robot_callback,
            callback_group=self.service_callback_group
        )

        self.safety_service = self.create_service(
            SetBool,
            'set_safety_mode',
            self.safety_mode_callback,
            callback_group=self.service_callback_group
        )

        self.get_status_service = self.create_service(
            Trigger,
            'get_robot_status',
            self.get_status_callback,
            callback_group=self.service_callback_group
        )

        # Timer for periodic status updates
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self.publish_status,
            callback_group=self.control_callback_group
        )

        # Timer for velocity history publishing
        self.history_timer = self.create_timer(
            0.5,  # 2 Hz
            self.publish_velocity_history,
            callback_group=self.control_callback_group
        )

        # Declare parameters
        self.declare_parameter('obstacle_threshold', 1.0)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)

        # Get parameter values
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value

        self.get_logger().info('Robot Communication Node initialized')

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        try:
            # Store the latest scan
            self.last_scan = msg
            self.scan_buffer.append(msg)

            # Process the scan for obstacle detection
            min_distance = self.find_min_distance_in_scan(msg)

            # Create processed scan with obstacle information
            processed_scan = LaserScan()
            processed_scan.header = msg.header
            processed_scan.angle_min = msg.angle_min
            processed_scan.angle_max = msg.angle_max
            processed_scan.angle_increment = msg.angle_increment
            processed_scan.time_increment = msg.time_increment
            processed_scan.scan_time = msg.scan_time
            processed_scan.range_min = msg.range_min
            processed_scan.range_max = msg.range_max

            # Apply simple filtering to the ranges
            filtered_ranges = []
            for range_val in msg.ranges:
                if np.isnan(range_val) or np.isinf(range_val):
                    filtered_ranges.append(float('inf'))  # Use inf for invalid readings
                else:
                    filtered_ranges.append(range_val)

            processed_scan.ranges = filtered_ranges

            # Publish processed scan
            self.sensor_processed_publisher.publish(processed_scan)

            # Check for safety conditions
            if min_distance < self.obstacle_threshold and self.robot_enabled:
                self.trigger_safety_stop()

        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {str(e)}')

    def velocity_callback(self, msg):
        """Process incoming velocity commands"""
        try:
            # Store velocity in history
            self.velocity_history.append({
                'linear': msg.linear.x,
                'angular': msg.angular.z,
                'timestamp': time.time()
            })

            # Apply safety checks before forwarding command
            if self.safety_mode or not self.robot_enabled:
                # If in safety mode or disabled, only allow zero commands
                if msg.linear.x != 0.0 or msg.angular.z != 0.0:
                    self.get_logger().warn('Command rejected due to safety mode or disabled state')
                    return

            # Apply velocity limits
            limited_msg = Twist()
            limited_msg.linear = Vector3(
                x=min(max(msg.linear.x, -self.max_linear_vel), self.max_linear_vel),
                y=0.0,
                z=0.0
            )
            limited_msg.angular = Vector3(
                x=0.0,
                y=0.0,
                z=min(max(msg.angular.z, -self.max_angular_vel), self.max_angular_vel)
            )

            # Publish the (potentially modified) command
            self.cmd_publisher.publish(limited_msg)

        except Exception as e:
            self.get_logger().error(f'Error in velocity callback: {str(e)}')

    def find_min_distance_in_scan(self, scan_msg):
        """Find minimum distance in laser scan, ignoring invalid readings"""
        valid_distances = [
            r for r in scan_msg.ranges
            if not (np.isnan(r) or np.isinf(r) or r <= scan_msg.range_min or r >= scan_msg.range_max)
        ]

        if valid_distances:
            return min(valid_distances)
        else:
            return float('inf')  # No valid readings

    def trigger_safety_stop(self):
        """Trigger safety stop when obstacles are detected"""
        if self.robot_enabled and not self.safety_mode:
            self.get_logger().warn(f'Obstacle detected at {self.find_min_distance_in_scan(self.last_scan):.2f}m, triggering safety stop!')

            # Publish emergency stop command
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.safety_publisher.publish(stop_cmd)

            # Optionally disable robot until manually re-enabled
            # self.robot_enabled = False
            # status_msg = String()
            # status_msg.data = 'SAFETY_STOP: Obstacle detected'
            # self.status_publisher.publish(status_msg)

    def enable_robot_callback(self, request, response):
        """Service callback to enable/disable the robot"""
        try:
            old_state = self.robot_enabled
            self.robot_enabled = request.data

            if request.data:
                self.get_logger().info('Robot enabled')
                response.message = 'Robot enabled successfully'
            else:
                self.get_logger().info('Robot disabled')
                response.message = 'Robot disabled successfully'

            response.success = True

            # Publish status update
            status_msg = String()
            status_msg.data = f'Robot {"enabled" if request.data else "disabled"}'
            self.status_publisher.publish(status_msg)

            # If disabling, also send stop command
            if not request.data:
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_publisher.publish(stop_cmd)

            return response

        except Exception as e:
            self.get_logger().error(f'Error in enable_robot service: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            return response

    def safety_mode_callback(self, request, response):
        """Service callback to set safety mode"""
        try:
            old_mode = self.safety_mode
            self.safety_mode = request.data

            if request.data:
                self.get_logger().info('Safety mode enabled')
                response.message = 'Safety mode enabled - robot will only accept zero velocity commands'
            else:
                self.get_logger().info('Safety mode disabled')
                response.message = 'Safety mode disabled'

            response.success = True

            # Publish safety status
            status_msg = String()
            status_msg.data = f'Safety mode {"enabled" if request.data else "disabled"}'
            self.status_publisher.publish(status_msg)

            return response

        except Exception as e:
            self.get_logger().error(f'Error in safety_mode service: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            return response

    def get_status_callback(self, request, response):
        """Service callback to get current robot status"""
        try:
            status_info = f'Robot: {"ENABLED" if self.robot_enabled else "DISABLED"}, ' \
                         f'Safety: {"ACTIVE" if self.safety_mode else "INACTIVE"}, ' \
                         f'Last scan: {"available" if self.last_scan else "none"}, ' \
                         f'Obstacle dist: {"{:.2f}m".format(self.find_min_distance_in_scan(self.last_scan)) if self.last_scan else "unknown"}'

            response.success = True
            response.message = status_info

            return response

        except Exception as e:
            self.get_logger().error(f'Error in get_status service: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
            return response

    def publish_status(self):
        """Publish periodic status updates"""
        try:
            status_msg = String()
            status_msg.data = f'Robot: {"RUNNING" if self.robot_enabled else "STOPPED"}, ' \
                             f'Safety: {"ACTIVE" if self.safety_mode else "INACTIVE"}, ' \
                             f'Scans processed: {len(self.scan_buffer)}'
            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error in status timer: {str(e)}')

    def publish_velocity_history(self):
        """Publish velocity history for analysis"""
        try:
            if self.velocity_history:
                history_msg = Float64MultiArray()

                # Flatten velocity history into a single array [lin1, ang1, lin2, ang2, ...]
                flat_history = []
                for vel_entry in list(self.velocity_history)[-10:]:  # Last 10 entries
                    flat_history.extend([vel_entry['linear'], vel_entry['angular']])

                history_msg.data = flat_history
                self.velocity_history_publisher.publish(history_msg)

        except Exception as e:
            self.get_logger().error(f'Error in history timer: {str(e)}')


def main(args=None):
    """Main function to run the Robot Communication Node"""
    rclpy.init(args=args)

    node = RobotCommunicationNode()

    # Use multi-threaded executor to handle callbacks in different threads
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        node.get_logger().info('Starting robot communication node...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Communication Pattern Applications

Different communication patterns are appropriate for different types of robotic applications. Understanding these applications helps in designing systems that use the most appropriate patterns for their requirements.

### Sensor Data Distribution

Sensors typically publish data via topics because they provide continuous streams of information that multiple nodes may need to process. For example:

**Camera Data**: Published at high frequency (typically 10-30 Hz) to multiple processing nodes like object detection, SLAM, and visualization.

**LiDAR Data**: Published at moderate frequency (5-20 Hz) to perception, mapping, and navigation nodes.

**IMU Data**: Published at high frequency (100-1000 Hz) to state estimation and control nodes.

**Joint States**: Published at moderate frequency (10-50 Hz) to monitoring, control, and simulation nodes.

### Control and Coordination

Control commands and coordination typically use a combination of patterns:

**Low-level Control**: Velocity commands are sent via topics for real-time control with high frequency updates.

**High-level Commands**: Navigation goals or manipulation tasks are often implemented as actions due to their long-running nature and need for feedback.

**Configuration**: System parameters and settings are typically managed via services or the parameter system.

**Status Queries**: On-demand status information is retrieved via services when needed.

### Real-time vs Non-real-time Considerations

The choice of communication pattern also depends on real-time requirements:

**Real-time Critical**: Use topics with appropriate QoS settings for time-critical data like control commands and safety-related information.

**Non-real-time**: Use services for configuration, diagnostics, and other operations where timing is less critical.

**Long-running Operations**: Use actions for tasks that may take seconds or minutes to complete.

## Advanced Communication Patterns

Beyond the basic patterns, ROS 2 supports several advanced communication patterns that can be useful for complex robotic applications.

### Action Clients and Servers

Actions provide a sophisticated communication pattern for long-running operations that require feedback and cancellation. An action-based navigation system might work as follows:

1. A navigation client sends a goal to a navigation server
2. The server begins processing the goal and sends periodic feedback about progress
3. The client can monitor progress and cancel the goal if needed
4. When the goal is completed (or fails), the server sends the final result

This pattern is essential for operations like navigation, manipulation, and calibration that take significant time to complete.

### Service Composition

Complex operations can be built by composing multiple services. For example, a "go to location" operation might involve:
1. A service call to get the current map
2. A service call to plan a path
3. An action call to execute the navigation

This composition allows for modular design where each service provides a specific capability.

### Topic Bridges

In distributed systems, topic bridges can connect different ROS 2 domains or even different ROS versions. This enables communication between systems that might otherwise be isolated.

### Parameter Services

ROS 2 provides built-in services for parameter management that allow nodes to be configured at runtime without recompilation. This is crucial for adapting to different environments and operational conditions.

## ROS 2 Data-Flow Model Diagram

The following diagram illustrates the data flow patterns in ROS 2 and how different communication patterns interact:

```mermaid
graph TB
    subgraph "Sensor Layer"
        Camera[Camera<br/>sensor_msgs/Image]
        LiDAR[LiDAR<br/>sensor_msgs/LaserScan]
        IMU[IMU<br/>sensor_msgs/Imu]
        Joint[Joints<br/>sensor_msgs/JointState]
    end

    subgraph "Processing Layer"
        Perception[Perception Node<br/>Object Detection]
        Localization[Localization Node<br/>Pose Estimation]
        Planning[Planning Node<br/>Path Planning]
        Control[Control Node<br/>Motion Control]
    end

    subgraph "Communication Patterns"
        Topics[Topics<br/>Publish/Subscribe<br/>Async]
        Services[Services<br/>Request/Response<br/>Sync]
        Actions[Actions<br/>Goal/Feedback/Result<br/>Async Long-Running]
    end

    subgraph "Control Layer"
        NavAction[Navigation Action<br/>move_to_pose]
        ManipService[Manipulation Service<br/>pick_object]
        ConfigService[Configuration Service<br/>set_parameters]
    end

    subgraph "Output Layer"
        CmdVel[cmd_vel<br/>geometry_msgs/Twist]
        Status[status<br/>std_msgs/String]
        Results[results<br/>custom messages]
    end

    subgraph "User Interface"
        RViz[rviz2<br/>Visualization]
        Command[Command Interface<br/>Manual Control]
        Monitor[Monitoring Tools<br/>Status Display]
    end

    %% Sensor to Processing
    Camera --> Perception
    LiDAR --> Perception
    IMU --> Localization
    Joint --> Control

    %% Processing to Communication
    Perception --> Topics
    Localization --> Topics
    Planning --> Services
    Control --> Topics

    %% Communication to Control
    Topics --> Control
    Services --> Planning
    Actions --> NavAction

    %% Control to Output
    Control --> CmdVel
    NavAction --> CmdVel
    Planning --> CmdVel

    %% Services and Actions
    Command --> ConfigService
    ConfigService --> Control
    Command --> ManipService
    ManipService --> Control

    %% Feedback paths
    CmdVel --> RViz
    Status --> Monitor
    Results --> RViz

    %% Bidirectional flows
    RViz <--> Topics
    Command <--> Services

    style "Sensor Layer" fill:#e1f5fe
    style "Processing Layer" fill:#f3e5f5
    style "Communication Patterns" fill:#e8f5e8
    style "Control Layer" fill:#fff3e0
    style "Output Layer" fill:#fce4ec
    style "User Interface" fill:#f1f8e9
```

This diagram shows how different communication patterns work together in a typical robotic system, with sensors providing data through topics, services handling configuration and discrete operations, and actions managing long-running tasks.

## Hands-on Lab: Communication Patterns Challenge

In this hands-on lab, you'll implement a complete robotic communication system that demonstrates the proper use of topics, services, and actions. This lab will give you practical experience with the communication patterns covered in this chapter.

### Lab Objectives

By completing this lab, you will:
- Implement publisher/subscriber patterns for continuous data streams
- Create service servers and clients for discrete operations
- Design action servers and clients for long-running tasks
- Understand the trade-offs between different communication approaches
- Build a system that properly coordinates multiple communication patterns

### Lab Setup

For this lab, you'll need:
- A ROS 2 installation (Humble Hawksbill or later)
- Basic Python programming skills
- Understanding of ROS 2 concepts from previous chapters

### Implementation Steps

1. **Create a new ROS 2 package** for the lab:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python communication_patterns_lab
```

2. **Implement a sensor fusion node** that:
   - Subscribes to multiple sensor topics (camera, LiDAR, IMU)
   - Publishes fused sensor data via topics
   - Provides configuration via services
   - Implements appropriate QoS policies

3. **Create a navigation action server** that:
   - Accepts navigation goals via actions
   - Provides feedback on progress
   - Handles cancellation requests
   - Integrates with the sensor fusion node

4. **Develop a command and control client** that:
   - Uses services for configuration
   - Uses actions for navigation tasks
   - Subscribes to status topics
   - Implements proper error handling

5. **Test the system** with different scenarios:
   - Normal operation with all communication patterns
   - Service call failures and recovery
   - Action cancellation
   - Network disruptions affecting different patterns

### Lab Deliverables

Submit the following for evaluation:
1. Your complete node implementations for sensor fusion, navigation, and control
2. A configuration file showing different QoS policies used
3. A brief report describing your communication pattern choices and rationale
4. Test results showing the system behavior under different conditions

### Evaluation Criteria

Your implementation will be evaluated on:
- Correctness: Do the communication patterns work as expected?
- Robustness: How well does the system handle failures and errors?
- Pattern Appropriateness: Are the right communication patterns used for each task?
- Resource Management: Are resources properly managed and released?
- Documentation: Is the code well-documented and easy to understand?

## Chapter Summary

This chapter has provided a comprehensive exploration of ROS 2 communication patterns - topics, services, and actions - which form the foundation of all robotic system interactions. We've examined the differences between these patterns, their appropriate use cases, and how to implement them effectively in real robotic systems.

Topics provide the asynchronous, publish-subscribe communication model that is ideal for continuous data streams like sensor readings and status updates. Their decoupled nature makes them perfect for real-time applications where timing is critical and multiple subscribers may need the same information.

Services offer synchronous request/response communication that is appropriate for discrete operations where a client needs to wait for a specific result. This pattern is essential for configuration, diagnostics, and other operations where guaranteed delivery and response are important.

Actions extend the service pattern to handle long-running operations that require feedback and cancellation capabilities. This pattern is crucial for complex tasks like navigation, manipulation, and calibration that take significant time to complete.

The practical example demonstrated how these communication patterns can be combined in a comprehensive robotic system, showing proper implementation techniques, error handling, and resource management. The example illustrated how different patterns serve different purposes within a single system, with topics handling continuous data, services managing configuration, and proper state management throughout.

The advanced communication patterns section explored more sophisticated uses of these basic patterns, including service composition, action coordination, and parameter management. These advanced techniques enable the development of complex, production-quality robotic systems.

Understanding these communication patterns is essential for designing effective robotic systems that can operate reliably in complex environments. The next chapter will build on these concepts by exploring ROS 2 packages and launch files, which provide the system composition and deployment mechanisms that bring these communication patterns together into complete robotic applications.

As we continue through the curriculum, you'll see how these communication patterns integrate with system architecture, package management, and deployment strategies to create comprehensive Physical AI systems that can operate effectively in real-world environments.
