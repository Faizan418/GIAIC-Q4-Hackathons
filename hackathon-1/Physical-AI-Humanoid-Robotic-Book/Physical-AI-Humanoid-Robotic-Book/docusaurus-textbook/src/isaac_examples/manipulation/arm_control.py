#!/usr/bin/env python3

"""
Arm Control for Robotic Manipulation

This module implements arm control algorithms for executing manipulation tasks.
It includes trajectory planning, inverse kinematics, and control interfaces
for robotic arms.
"""

import math
import threading
import time
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Point, Pose, Quaternion

# Import common utilities
from isaac_examples.common.isaac_ros_utils import (
    create_point,
    create_pose,
    create_vector3,
)
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmState(Enum):
    """Enumeration of arm states"""

    IDLE = "idle"
    MOVING_TO_PRE_GRASP = "moving_to_pre_grasp"
    MOVING_TO_GRASP = "moving_to_grasp"
    GRASPING = "grasping"
    MOVING_TO_PLACE = "moving_to_place"
    PLACING = "placing"
    MOVING_TO_HOME = "moving_to_home"
    ERROR = "error"


class ArmController(Node):
    """
    Arm Control Node for robotic manipulation
    """

    def __init__(self):
        super().__init__("arm_controller")

        # Declare parameters
        self.declare_parameter("robot_description_param", "robot_description")
        self.declare_parameter("move_group_name", "manipulator")
        self.declare_parameter("end_effector_link", "gripper_link")
        self.declare_parameter("max_velocity_scaling_factor", 0.5)
        self.declare_parameter("max_acceleration_scaling_factor", 0.5)
        self.declare_parameter("gripper_open_position", 0.05)
        self.declare_parameter("gripper_close_position", 0.0)
        self.declare_parameter("approach_distance", 0.1)
        self.declare_parameter("retract_distance", 0.1)

        # Get parameters
        self.robot_description_param = self.get_parameter(
            "robot_description_param"
        ).value
        self.move_group_name = self.get_parameter("move_group_name").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.max_velocity_scaling = self.get_parameter(
            "max_velocity_scaling_factor"
        ).value
        self.max_acceleration_scaling = self.get_parameter(
            "max_acceleration_scaling_factor"
        ).value
        self.gripper_open_pos = self.get_parameter("gripper_open_position").value
        self.gripper_close_pos = self.get_parameter("gripper_close_position").value
        self.approach_distance = self.get_parameter("approach_distance").value
        self.retract_distance = self.get_parameter("retract_distance").value

        # Publishers
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, "joint_trajectory", 10
        )
        self.gripper_command_pub = self.create_publisher(
            JointTrajectory, "gripper_command", 10
        )
        self.arm_state_pub = self.create_publisher(Header, "arm_state", 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 10
        )
        self.grasp_pose_sub = self.create_subscription(
            Pose, "best_grasp", self.grasp_pose_callback, 10
        )

        # Services
        self.ik_service = self.create_client(GetPositionIK, "compute_ik")
        self.fk_service = self.create_client(GetPositionFK, "compute_fk")

        # Internal state
        self.current_joint_positions = {}
        self.current_pose = None
        self.arm_state = ArmState.IDLE
        self.target_grasp_pose = None
        self.object_grasped = False
        self.manipulation_thread = None
        self.manipulation_lock = threading.Lock()

        # Joint names for the arm (example - would be loaded from URDF)
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.get_logger().info("Arm Controller initialized")

    def joint_state_callback(self, joint_state_msg: JointState):
        """
        Callback for joint state updates

        Args:
            joint_state_msg: JointState message with current joint positions
        """
        for i, name in enumerate(joint_state_msg.name):
            if i < len(joint_state_msg.position):
                self.current_joint_positions[name] = joint_state_msg.position[i]

    def grasp_pose_callback(self, grasp_pose_msg: Pose):
        """
        Callback for receiving grasp poses

        Args:
            grasp_pose_msg: Pose message for the grasp position
        """
        self.get_logger().info("Received grasp pose, initiating manipulation sequence")

        # Store the target grasp pose
        self.target_grasp_pose = grasp_pose_msg

        # Start manipulation sequence in a separate thread to avoid blocking
        with self.manipulation_lock:
            if (
                self.manipulation_thread is None
                or not self.manipulation_thread.is_alive()
            ):
                self.manipulation_thread = threading.Thread(
                    target=self.execute_manipulation_sequence
                )
                self.manipulation_thread.start()

    def execute_manipulation_sequence(self):
        """
        Execute the complete manipulation sequence: approach -> grasp -> retract -> move to place
        """
        try:
            self.set_arm_state(ArmState.MOVING_TO_PRE_GRASP)
            self.get_logger().info("Starting manipulation sequence")

            # 1. Move to pre-grasp position (approach from safe distance)
            if self.target_grasp_pose:
                pre_grasp_pose = self.calculate_pre_grasp_pose(self.target_grasp_pose)
                success = self.move_to_pose(pre_grasp_pose)
                if not success:
                    self.get_logger().error("Failed to move to pre-grasp position")
                    self.set_arm_state(ArmState.ERROR)
                    return

                # 2. Move to grasp position
                self.set_arm_state(ArmState.MOVING_TO_GRASP)
                success = self.move_to_pose(self.target_grasp_pose)
                if not success:
                    self.get_logger().error("Failed to move to grasp position")
                    self.set_arm_state(ArmState.ERROR)
                    return

                # 3. Close gripper to grasp object
                self.set_arm_state(ArmState.GRASPING)
                self.close_gripper()
                time.sleep(1.0)  # Wait for grasp to complete
                self.object_grasped = True

                # 4. Retract from object
                self.set_arm_state(ArmState.MOVING_TO_PLACE)
                post_grasp_pose = self.calculate_post_grasp_pose(self.target_grasp_pose)
                success = self.move_to_pose(post_grasp_pose)
                if not success:
                    self.get_logger().error("Failed to retract after grasp")
                    self.set_arm_state(ArmState.ERROR)
                    return

                # 5. Move to a default place position (for demonstration)
                place_pose = self.calculate_place_pose()
                success = self.move_to_pose(place_pose)
                if not success:
                    self.get_logger().error("Failed to move to place position")
                    self.set_arm_state(ArmState.ERROR)
                    return

                # 6. Open gripper to place object
                self.open_gripper()
                time.sleep(1.0)  # Wait for placement
                self.object_grasped = False

                # 7. Return to home position
                self.set_arm_state(ArmState.MOVING_TO_HOME)
                home_pose = self.calculate_home_pose()
                success = self.move_to_pose(home_pose)
                if success:
                    self.get_logger().info(
                        "Manipulation sequence completed successfully"
                    )
                    self.set_arm_state(ArmState.IDLE)
                else:
                    self.get_logger().error("Failed to return to home position")
                    self.set_arm_state(ArmState.ERROR)

        except Exception as e:
            self.get_logger().error(f"Error in manipulation sequence: {e}")
            self.set_arm_state(ArmState.ERROR)

    def calculate_pre_grasp_pose(self, grasp_pose: Pose) -> Pose:
        """
        Calculate pre-grasp pose by moving approach_distance away from grasp pose
        along the approach direction (typically the negative z-axis of the gripper)

        Args:
            grasp_pose: The target grasp pose

        Returns:
            Pre-grasp pose
        """
        pre_grasp = Pose()
        pre_grasp.orientation = grasp_pose.orientation

        # For a top-down grasp, approach from above
        # The orientation should have the gripper pointing downward
        # Move along the negative z-axis of the gripper frame
        import tf_transformations as tf

        orientation = [
            grasp_pose.orientation.x,
            grasp_pose.orientation.y,
            grasp_pose.orientation.z,
            grasp_pose.orientation.w,
        ]
        rotation_matrix = tf.quaternion_matrix(orientation)

        # The z-axis of the gripper frame in world coordinates
        approach_direction = np.array(
            [rotation_matrix[0, 2], rotation_matrix[1, 2], rotation_matrix[2, 2]]
        )

        # Move away from the grasp position
        offset = approach_direction * self.approach_distance
        pre_grasp.position.x = grasp_pose.position.x - offset[0]
        pre_grasp.position.y = grasp_pose.position.y - offset[1]
        pre_grasp.position.z = grasp_pose.position.z - offset[2]

        return pre_grasp

    def calculate_post_grasp_pose(self, grasp_pose: Pose) -> Pose:
        """
        Calculate post-grasp pose by moving retract_distance away from grasp pose

        Args:
            grasp_pose: The grasp pose

        Returns:
            Post-grasp pose
        """
        post_grasp = Pose()
        post_grasp.orientation = grasp_pose.orientation

        # For a top-down grasp, retract upward
        import tf_transformations as tf

        orientation = [
            grasp_pose.orientation.x,
            grasp_pose.orientation.y,
            grasp_pose.orientation.z,
            grasp_pose.orientation.w,
        ]
        rotation_matrix = tf.quaternion_matrix(orientation)

        # The z-axis of the gripper frame in world coordinates
        retract_direction = np.array(
            [rotation_matrix[0, 2], rotation_matrix[1, 2], rotation_matrix[2, 2]]
        )

        # Move away from the grasp position
        offset = retract_direction * self.retract_distance
        post_grasp.position.x = grasp_pose.position.x + offset[0]
        post_grasp.position.y = grasp_pose.position.y + offset[1]
        post_grasp.position.z = grasp_pose.position.z + offset[2]

        return post_grasp

    def calculate_place_pose(self) -> Pose:
        """
        Calculate a default place pose (for demonstration purposes)

        Returns:
            Place pose
        """
        place_pose = Pose()
        place_pose.position.x = 0.5  # Place at x=0.5m
        place_pose.position.y = 0.0  # Place at y=0.0m
        place_pose.position.z = 0.2  # Place at z=0.2m height

        # Default orientation for placement
        place_pose.orientation.w = 1.0
        place_pose.orientation.x = 0.0
        place_pose.orientation.y = 0.0
        place_pose.orientation.z = 0.0

        return place_pose

    def calculate_home_pose(self) -> Pose:
        """
        Calculate the home position for the arm

        Returns:
            Home pose
        """
        home_pose = Pose()
        home_pose.position.x = 0.0
        home_pose.position.y = 0.0
        home_pose.position.z = 0.5  # Home position at 0.5m height

        # Default orientation for home position
        home_pose.orientation.w = 1.0
        home_pose.orientation.x = 0.0
        home_pose.orientation.y = 0.0
        home_pose.orientation.z = 0.0

        return home_pose

    def move_to_pose(self, target_pose: Pose) -> bool:
        """
        Move the arm to a target pose using inverse kinematics

        Args:
            target_pose: Target pose for the end effector

        Returns:
            True if movement successful, False otherwise
        """
        self.get_logger().info(
            f"Moving to pose: ({target_pose.position.x:.3f}, "
            f"{target_pose.position.y:.3f}, {target_pose.position.z:.3f})"
        )

        # In a real implementation, this would call MoveIt! or similar planning service
        # For this simulation, we'll generate a simple trajectory

        # Get current joint positions as start state
        start_positions = []
        for joint_name in self.joint_names:
            if joint_name in self.current_joint_positions:
                start_positions.append(self.current_joint_positions[joint_name])
            else:
                start_positions.append(0.0)  # Default position

        # Plan a simple trajectory (in reality, this would be done with MoveIt!)
        trajectory = self.generate_simple_trajectory(start_positions, target_pose)

        if trajectory:
            # Publish the trajectory
            self.joint_trajectory_pub.publish(trajectory)

            # Wait for movement to complete (in a real system, you'd monitor feedback)
            time.sleep(3.0)  # Simulate movement time

            return True
        else:
            return False

    def generate_simple_trajectory(
        self, start_positions: List[float], target_pose: Pose
    ) -> Optional[JointTrajectory]:
        """
        Generate a simple joint trajectory to reach the target pose

        Args:
            start_positions: Current joint positions
            target_pose: Target end effector pose

        Returns:
            JointTrajectory message or None if planning fails
        """
        try:
            # In a real implementation, this would call inverse kinematics
            # For this simulation, we'll create a simple linear joint trajectory

            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names

            # Create trajectory points
            num_points = 50  # Number of points in the trajectory
            time_step = 0.1  # Time between points (seconds)

            for i in range(num_points + 1):
                point = JointTrajectoryPoint()

                # Interpolate joint positions linearly
                t = i / num_points  # Interpolation factor (0 to 1)

                # For this simulation, we'll create a simple movement
                # In reality, you'd calculate the actual joint angles needed
                for j, joint_name in enumerate(self.joint_names):
                    if j < len(start_positions):
                        # Simple linear interpolation between start and a target configuration
                        # For demonstration, we'll just create a simple motion pattern
                        start_pos = start_positions[j]
                        # Target position based on the target pose (simplified)
                        target_pos = start_pos + (0.5 if j % 2 == 0 else -0.5) * t
                        point.positions.append(target_pos)
                    else:
                        point.positions.append(0.0)

                # Set velocities to 0 (in reality, you'd calculate proper velocities)
                point.velocities = [0.0] * len(self.joint_names)
                point.accelerations = [0.0] * len(self.joint_names)

                # Set time from start
                point.time_from_start.sec = int(t * num_points * time_step)
                point.time_from_start.nanosec = int(
                    (t * num_points * time_step - point.time_from_start.sec) * 1e9
                )

                trajectory.points.append(point)

            return trajectory

        except Exception as e:
            self.get_logger().error(f"Error generating trajectory: {e}")
            return None

    def close_gripper(self):
        """
        Close the gripper to grasp an object
        """
        self.get_logger().info("Closing gripper")
        self._send_gripper_command(self.gripper_close_pos)

    def open_gripper(self):
        """
        Open the gripper to release an object
        """
        self.get_logger().info("Opening gripper")
        self._send_gripper_command(self.gripper_open_pos)

    def _send_gripper_command(self, position: float):
        """
        Send a command to the gripper

        Args:
            position: Target gripper position
        """
        try:
            trajectory = JointTrajectory()
            trajectory.joint_names = ["gripper_joint"]  # Example gripper joint name
            point = JointTrajectoryPoint()
            point.positions = [position]
            point.velocities = [0.0]
            point.time_from_start.sec = 1
            point.time_from_start.nanosec = 0
            trajectory.points = [point]
            self.gripper_command_pub.publish(trajectory)
        except Exception as e:
            self.get_logger().error(f"Error sending gripper command: {e}")

    def set_arm_state(self, state: ArmState):
        """
        Set and publish the current arm state

        Args:
            state: New arm state
        """
        self.arm_state = state
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = f"arm_state_{state.value}"
        self.arm_state_pub.publish(header)
        self.get_logger().info(f"Arm state set to: {state.value}")


class SimpleArmController:
    """
    Simplified arm controller for demonstration purposes
    """

    def __init__(self):
        self.current_joint_positions = [0.0] * 6  # Example 6-DOF arm
        self.object_grasped = False

    def move_to_pose(self, target_pose: Pose) -> bool:
        """
        Simulate moving the arm to a target pose

        Args:
            target_pose: Target pose for the end effector

        Returns:
            True if movement successful, False otherwise
        """
        print(
            f"Moving arm to pose: ({target_pose.position.x:.3f}, "
            f"{target_pose.position.y:.3f}, {target_pose.position.z:.3f})"
        )

        # Simulate movement time
        time.sleep(2.0)

        # Update joint positions based on target pose (simplified)
        # In a real system, this would involve inverse kinematics
        for i in range(len(self.current_joint_positions)):
            self.current_joint_positions[i] += 0.1  # Simple update for simulation

        return True

    def grasp_object(self) -> bool:
        """
        Simulate grasping an object

        Returns:
            True if grasp successful, False otherwise
        """
        print("Grasping object...")
        time.sleep(1.0)
        self.object_grasped = True
        return True

    def release_object(self) -> bool:
        """
        Simulate releasing an object

        Returns:
            True if release successful, False otherwise
        """
        print("Releasing object...")
        time.sleep(1.0)
        self.object_grasped = False
        return True

    def execute_pick_and_place(self, grasp_pose: Pose, place_pose: Pose) -> bool:
        """
        Execute a complete pick and place operation

        Args:
            grasp_pose: Pose for grasping the object
            place_pose: Pose for placing the object

        Returns:
            True if operation successful, False otherwise
        """
        print("Starting pick and place operation...")

        # Move to pre-grasp position
        pre_grasp = Pose()
        pre_grasp.position.x = grasp_pose.position.x
        pre_grasp.position.y = grasp_pose.position.y
        pre_grasp.position.z = grasp_pose.position.z + 0.1  # 10cm above object
        pre_grasp.orientation = grasp_pose.orientation

        if not self.move_to_pose(pre_grasp):
            print("Failed to move to pre-grasp position")
            return False

        # Move to grasp position
        if not self.move_to_pose(grasp_pose):
            print("Failed to move to grasp position")
            return False

        # Grasp the object
        if not self.grasp_object():
            print("Failed to grasp object")
            return False

        # Move to post-grasp position
        post_grasp = Pose()
        post_grasp.position.x = grasp_pose.position.x
        post_grasp.position.y = grasp_pose.position.y
        post_grasp.position.z = grasp_pose.position.z + 0.1  # 10cm above object
        post_grasp.orientation = grasp_pose.orientation

        if not self.move_to_pose(post_grasp):
            print("Failed to move to post-grasp position")
            return False

        # Move to place position
        if not self.move_to_pose(place_pose):
            print("Failed to move to place position")
            return False

        # Release the object
        if not self.release_object():
            print("Failed to release object")
            return False

        print("Pick and place operation completed successfully!")
        return True


def main(args=None):
    """
    Main function to run the Arm Controller
    """
    rclpy.init(args=args)

    arm_controller = ArmController()

    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
