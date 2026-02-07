#!/usr/bin/env python3

"""
Grasp Planning for Robotic Manipulation

This module implements grasp planning algorithms for robotic manipulation tasks.
It takes object poses from perception systems and generates feasible grasp poses
for robotic arms.
"""

import math
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion

# Import common utilities
from isaac_examples.common.isaac_ros_utils import (
    create_point,
    create_pose,
    create_vector3,
)
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene, RobotState
from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


class GraspType(Enum):
    """Enumeration of different grasp types"""

    PINCH = "pinch"
    PALM = "palm"
    SUCTION = "suction"
    PINCER = "pincer"


class GraspPose:
    """Class to represent a grasp pose with additional properties"""

    def __init__(
        self,
        pose: Pose,
        grasp_type: GraspType,
        score: float = 0.0,
        approach_direction: Point = None,
    ):
        self.pose = pose
        self.grasp_type = grasp_type
        self.score = score  # Higher score means better grasp
        self.approach_direction = approach_direction or Point(x=0.0, y=0.0, z=-1.0)


class GraspPlanner(Node):
    """
    Grasp Planning Node for robotic manipulation
    """

    def __init__(self):
        super().__init__("grasp_planner")

        # Declare parameters
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("end_effector_frame", "gripper_link")
        self.declare_parameter("grasp_approach_distance", 0.1)  # 10cm approach
        self.declare_parameter("grasp_offset_distance", 0.05)  # 5cm above object
        self.declare_parameter("min_grasp_score", 0.5)  # Minimum score for valid grasp
        self.declare_parameter("enable_visualization", True)

        # Get parameters
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.end_effector_frame = self.get_parameter("end_effector_frame").value
        self.grasp_approach_distance = self.get_parameter(
            "grasp_approach_distance"
        ).value
        self.grasp_offset_distance = self.get_parameter("grasp_offset_distance").value
        self.min_grasp_score = self.get_parameter("min_grasp_score").value
        self.enable_visualization = self.get_parameter("enable_visualization").value

        # Publishers
        self.grasp_poses_pub = self.create_publisher(PoseArray, "candidate_grasps", 10)
        self.best_grasp_pub = self.create_publisher(Pose, "best_grasp", 10)
        self.visualization_pub = (
            self.create_publisher(MarkerArray, "grasp_visualization", 10)
            if self.enable_visualization
            else None
        )

        # Subscribers
        self.object_poses_sub = self.create_subscription(
            PoseArray, "object_poses", self.object_poses_callback, 10
        )

        # Internal state
        self.object_poses = []
        self.candidate_grasps = []
        self.best_grasp = None

        self.get_logger().info("Grasp Planner initialized")

    def object_poses_callback(self, pose_array_msg):
        """
        Callback for receiving object poses from perception system

        Args:
            pose_array_msg: PoseArray message containing object poses
        """
        self.get_logger().info(
            f"Received {len(pose_array_msg.poses)} object poses for grasp planning"
        )

        # Store object poses
        self.object_poses = pose_array_msg.poses

        # Plan grasps for all objects
        all_grasps = []
        for obj_pose in self.object_poses:
            object_grasps = self.plan_grasps_for_object(obj_pose)
            all_grasps.extend(object_grasps)

        # Store all candidate grasps
        self.candidate_grasps = all_grasps

        # Select the best grasp
        self.select_best_grasp()

        # Publish results
        self.publish_grasp_results()

    def plan_grasps_for_object(self, obj_pose: Pose) -> List[GraspPose]:
        """
        Plan multiple grasp poses for a single object

        Args:
            obj_pose: Pose of the object to grasp

        Returns:
            List of candidate grasp poses
        """
        candidate_grasps = []

        # Generate multiple grasp approaches around the object
        grasp_configs = [
            # Top-down grasp
            {
                "approach": Point(x=0.0, y=0.0, z=-1.0),
                "orientation": Quaternion(
                    w=1.0, x=0.0, y=0.0, z=0.0
                ),  # Default orientation
                "grasp_type": GraspType.PINCH,
            },
            # Side grasp 1 (from positive X)
            {
                "approach": Point(x=-1.0, y=0.0, z=0.0),
                "orientation": Quaternion(
                    w=0.707, x=0.0, y=0.0, z=0.707
                ),  # Rotate 90° around Z
                "grasp_type": GraspType.PALM,
            },
            # Side grasp 2 (from negative X)
            {
                "approach": Point(x=1.0, y=0.0, z=0.0),
                "orientation": Quaternion(
                    w=0.0, x=0.0, y=0.707, z=0.707
                ),  # Rotate 90° around Y
                "grasp_type": GraspType.PALM,
            },
            # Side grasp 3 (from positive Y)
            {
                "approach": Point(x=0.0, y=-1.0, z=0.0),
                "orientation": Quaternion(w=0.5, x=0.5, y=0.5, z=0.5),  # 90° rotation
                "grasp_type": GraspType.PINCH,
            },
            # Side grasp 4 (from negative Y)
            {
                "approach": Point(x=0.0, y=1.0, z=0.0),
                "orientation": Quaternion(w=0.5, x=-0.5, y=-0.5, z=0.5),  # 90° rotation
                "grasp_type": GraspType.PINCH,
            },
        ]

        for config in grasp_configs:
            # Calculate grasp pose by offsetting from object position
            grasp_pose = Pose()

            # Position: offset from object by grasp offset distance in the approach direction
            offset_distance = self.grasp_offset_distance
            grasp_pose.position.x = (
                obj_pose.position.x + config["approach"].x * offset_distance
            )
            grasp_pose.position.y = (
                obj_pose.position.y + config["approach"].y * offset_distance
            )
            grasp_pose.position.z = (
                obj_pose.position.z + config["approach"].z * offset_distance
            )

            # Orientation: use the specified orientation
            grasp_pose.orientation = config["orientation"]

            # Calculate grasp score based on multiple factors
            score = self.calculate_grasp_score(
                grasp_pose, obj_pose, config["grasp_type"]
            )

            # Create GraspPose object
            grasp = GraspPose(
                pose=grasp_pose,
                grasp_type=config["grasp_type"],
                score=score,
                approach_direction=config["approach"],
            )

            candidate_grasps.append(grasp)

        return candidate_grasps

    def calculate_grasp_score(
        self, grasp_pose: Pose, obj_pose: Pose, grasp_type: GraspType
    ) -> float:
        """
        Calculate a score for how good a particular grasp is

        Args:
            grasp_pose: The candidate grasp pose
            obj_pose: The object pose
            grasp_type: The type of grasp

        Returns:
            Score between 0.0 and 1.0 (higher is better)
        """
        score = 0.0

        # 1. Distance factor: closer grasps are generally better
        dist_to_obj = self.calculate_distance_3d(grasp_pose.position, obj_pose.position)
        # Score decreases as distance increases (max at 0 distance, min at 0.5m distance)
        dist_score = max(0.0, 1.0 - (dist_to_obj / 0.5))
        score += dist_score * 0.3  # 30% weight

        # 2. Grasp type appropriateness
        if grasp_type == GraspType.PINCH:
            # Good for small objects
            type_score = 0.8
        elif grasp_type == GraspType.PALM:
            # Good for medium objects
            type_score = 0.7
        elif grasp_type == GraspType.SUCTION:
            # Good for flat objects
            type_score = 0.6
        else:  # PINCER
            type_score = 0.75

        score += type_score * 0.3  # 30% weight

        # 3. Approach direction factor
        # Prefer top-down approaches for stability
        approach_z = abs(grasp_pose.position.z - obj_pose.position.z)
        if approach_z > 0.05:  # If we're not very close vertically
            approach_score = 0.5
        else:
            approach_score = 0.8  # Top-down is preferred
        score += approach_score * 0.2  # 20% weight

        # 4. Orientation factor
        # Prefer orientations that align with object orientation
        # For this simulation, we'll give a moderate score
        orientation_score = 0.7
        score += orientation_score * 0.2  # 20% weight

        # Ensure score is within bounds
        return min(1.0, max(0.0, score))

    def calculate_distance_3d(self, point1: Point, point2: Point) -> float:
        """
        Calculate 3D Euclidean distance between two points

        Args:
            point1: First point
            point2: Second point

        Returns:
            Distance between the points
        """
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        dz = point1.z - point2.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def select_best_grasp(self):
        """
        Select the best grasp from all candidates based on score
        """
        if not self.candidate_grasps:
            self.best_grasp = None
            return

        # Filter grasps by minimum score
        valid_grasps = [
            g for g in self.candidate_grasps if g.score >= self.min_grasp_score
        ]

        if not valid_grasps:
            # If no grasp meets minimum score, pick the highest scoring one
            best = max(self.candidate_grasps, key=lambda g: g.score)
        else:
            # Otherwise, pick the highest scoring valid grasp
            best = max(valid_grasps, key=lambda g: g.score)

        self.best_grasp = best

        self.get_logger().info(
            f"Selected best grasp with score {best.score:.3f}, "
            f"type {best.grasp_type.value} at position "
            f"({best.pose.position.x:.3f}, {best.pose.position.y:.3f}, {best.pose.position.z:.3f})"
        )

    def publish_grasp_results(self):
        """
        Publish grasp planning results
        """
        current_time = self.get_clock().now()
        header = Header()
        header.stamp = current_time.to_msg()
        header.frame_id = self.robot_base_frame

        # Publish all candidate grasps
        candidate_poses = PoseArray()
        candidate_poses.header = header
        for grasp in self.candidate_grasps:
            candidate_poses.poses.append(grasp.pose)
        self.grasp_poses_pub.publish(candidate_poses)

        # Publish best grasp
        if self.best_grasp is not None:
            best_grasp_msg = self.best_grasp.pose
            self.best_grasp_pub.publish(best_grasp_msg)

        # Publish visualization markers
        if self.enable_visualization and self.visualization_pub:
            self.publish_grasp_visualization(header)

    def publish_grasp_visualization(self, header: Header):
        """
        Publish visualization markers for grasps

        Args:
            header: Header for the published markers
        """
        marker_array = MarkerArray()

        # Visualize all candidate grasps
        for i, grasp in enumerate(self.candidate_grasps):
            # Create arrow marker for approach direction
            arrow_marker = Marker()
            arrow_marker.header = header
            arrow_marker.ns = "grasp_approaches"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD

            # Start at object position, end at grasp position
            start_point = create_point(
                grasp.pose.position.x - grasp.approach_direction.x * 0.05,
                grasp.pose.position.y - grasp.approach_direction.y * 0.05,
                grasp.pose.position.z - grasp.approach_direction.z * 0.05,
            )
            end_point = grasp.pose.position

            arrow_marker.points = [start_point, end_point]

            # Scale
            arrow_marker.scale.x = 0.01  # Shaft diameter
            arrow_marker.scale.y = 0.02  # Head diameter
            arrow_marker.scale.z = 0.0  # Not used for arrows

            # Color based on grasp type
            if grasp.grasp_type == GraspType.PINCH:
                arrow_marker.color.r = 1.0
                arrow_marker.color.g = 0.0
                arrow_marker.color.b = 0.0
            elif grasp.grasp_type == GraspType.PALM:
                arrow_marker.color.r = 0.0
                arrow_marker.color.g = 1.0
                arrow_marker.color.b = 0.0
            elif grasp.grasp_type == GraspType.SUCTION:
                arrow_marker.color.r = 0.0
                arrow_marker.color.g = 0.0
                arrow_marker.color.b = 1.0
            else:  # PINCER
                arrow_marker.color.r = 1.0
                arrow_marker.color.g = 1.0
                arrow_marker.color.b = 0.0

            arrow_marker.color.a = 0.8

            marker_array.markers.append(arrow_marker)

            # Create sphere marker for grasp pose
            sphere_marker = Marker()
            sphere_marker.header = header
            sphere_marker.ns = "grasp_poses"
            sphere_marker.id = i + 1000  # Different ID range
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD

            sphere_marker.pose = grasp.pose
            sphere_marker.scale.x = 0.03
            sphere_marker.scale.y = 0.03
            sphere_marker.scale.z = 0.03

            # Color based on grasp score (green=good, red=bad)
            if grasp.score > 0.7:
                sphere_marker.color.g = 1.0  # Green for good grasps
            elif grasp.score > 0.5:
                sphere_marker.color.r = 1.0
                sphere_marker.color.g = 1.0  # Yellow for medium grasps
            else:
                sphere_marker.color.r = 1.0  # Red for poor grasps

            sphere_marker.color.a = 0.8

            marker_array.markers.append(sphere_marker)

        # Highlight the best grasp
        if self.best_grasp is not None:
            best_idx = len(self.candidate_grasps)
            best_marker = Marker()
            best_marker.header = header
            best_marker.ns = "best_grasp"
            best_marker.id = best_idx
            best_marker.type = Marker.CUBE
            best_marker.action = Marker.ADD

            best_marker.pose = self.best_grasp.pose
            best_marker.scale.x = 0.05
            best_marker.scale.y = 0.05
            best_marker.scale.z = 0.05

            # Bright yellow for best grasp
            best_marker.color.r = 1.0
            best_marker.color.g = 1.0
            best_marker.color.b = 0.0
            best_marker.color.a = 1.0

            marker_array.markers.append(best_marker)

        self.visualization_pub.publish(marker_array)

    def validate_grasp(self, grasp_pose: Pose) -> bool:
        """
        Validate if a grasp pose is kinematically feasible

        Args:
            grasp_pose: Grasp pose to validate

        Returns:
            True if grasp is valid, False otherwise
        """
        # In a real implementation, this would check:
        # - Robot kinematic constraints
        # - Collision avoidance
        # - Joint limits
        # - Reachability

        # For this simulation, we'll do a basic check
        # Ensure the position is within a reasonable workspace
        if (
            abs(grasp_pose.position.x) > 2.0
            or abs(grasp_pose.position.y) > 2.0
            or grasp_pose.position.z < 0.1
            or grasp_pose.position.z > 2.0
        ):
            return False

        return True


class SimpleGraspPlanner:
    """
    Simplified grasp planner for demonstration purposes
    """

    def __init__(self):
        self.min_grasp_score = 0.5

    def plan_grasp_for_object(self, object_pose: Pose) -> Optional[GraspPose]:
        """
        Plan a simple grasp for a single object

        Args:
            object_pose: Pose of the object to grasp

        Returns:
            Best grasp pose or None if no valid grasp found
        """
        # Simple strategy: top-down grasp directly above the object
        grasp_pose = Pose()
        grasp_pose.position.x = object_pose.position.x
        grasp_pose.position.y = object_pose.position.y
        grasp_pose.position.z = object_pose.position.z + 0.1  # 10cm above object

        # Default orientation for top-down grasp
        grasp_pose.orientation.w = 1.0
        grasp_pose.orientation.x = 0.0
        grasp_pose.orientation.y = 0.0
        grasp_pose.orientation.z = 0.0

        # Calculate a basic score
        score = 0.8  # High score for top-down grasp

        grasp = GraspPose(pose=grasp_pose, grasp_type=GraspType.PINCH, score=score)

        return grasp if score >= self.min_grasp_score else None

    def plan_grasps_batch(self, object_poses: List[Pose]) -> List[GraspPose]:
        """
        Plan grasps for a batch of objects

        Args:
            object_poses: List of object poses

        Returns:
            List of grasp poses for all objects
        """
        grasps = []
        for obj_pose in object_poses:
            grasp = self.plan_grasp_for_object(obj_pose)
            if grasp is not None:
                grasps.append(grasp)

        return grasps


def main(args=None):
    """
    Main function to run the Grasp Planner
    """
    rclpy.init(args=args)

    grasp_planner = GraspPlanner()

    try:
        rclpy.spin(grasp_planner)
    except KeyboardInterrupt:
        pass
    finally:
        grasp_planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
