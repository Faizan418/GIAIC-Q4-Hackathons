#!/usr/bin/env python3

"""
Object Detection and Manipulation Lab

This lab demonstrates the complete pipeline from object detection in simulation
to robotic manipulation execution. It integrates Isaac Sim for scene generation,
Isaac ROS for perception processing, and manipulation control for object handling.
"""

import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import message_filters
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray

# Import common utilities
from isaac_examples.common.isaac_ros_utils import (
    create_point,
    create_pose,
    cv2_to_image_msg,
    get_transform,
    image_msg_to_cv2,
)

# Import perception and manipulation components
from isaac_examples.isaac_ros.vision_pipeline import IsaacVisionPipeline
from isaac_examples.manipulation.arm_control import ArmController
from isaac_examples.manipulation.grasp_planning import GraspPlanner
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


class ObjectManipulationLab(Node):
    """
    Object Detection and Manipulation Lab demonstrating complete perception-action loop
    """

    def __init__(self):
        super().__init__("object_manipulation_lab")

        # Declare parameters
        self.declare_parameter("camera_namespace", "/camera")
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("manipulation_enabled", True)
        self.declare_parameter("detection_confidence_threshold", 0.6)
        self.declare_parameter("max_objects_to_manipulate", 3)

        # Get parameters
        self.camera_namespace = self.get_parameter("camera_namespace").value
        self.enable_visualization = self.get_parameter("enable_visualization").value
        self.manipulation_enabled = self.get_parameter("manipulation_enabled").value
        self.confidence_threshold = self.get_parameter(
            "detection_confidence_threshold"
        ).value
        self.max_objects = self.get_parameter("max_objects_to_manipulate").value

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Publishers
        self.manipulation_status_pub = self.create_publisher(
            Header, "manipulation_status", 10
        )
        self.lab_status_pub = self.create_publisher(Header, "lab_status", 10)
        self.visualization_pub = (
            self.create_publisher(MarkerArray, "lab_visualization", 10)
            if self.enable_visualization
            else None
        )

        # Subscribers for object poses from perception system
        self.object_poses_sub = self.create_subscription(
            PoseArray, "object_poses", self.object_poses_callback, 10
        )

        # Internal state
        self.detected_objects = []
        self.objects_to_manipulate = []
        self.manipulation_queue = []
        self.manipulation_completed = 0
        self.manipulation_lock = threading.Lock()

        # Initialize perception and manipulation components
        self.vision_pipeline = IsaacVisionPipeline()
        self.grasp_planner = GraspPlanner()
        self.arm_controller = ArmController()

        # Create manipulation thread
        self.manipulation_thread = None
        self.lab_running = False

        self.get_logger().info("Object Manipulation Lab initialized")
        self.get_logger().info(f"Manipulation enabled: {self.manipulation_enabled}")

    def object_poses_callback(self, pose_array_msg):
        """
        Callback for receiving object poses from perception system

        Args:
            pose_array_msg: PoseArray message containing detected object poses
        """
        self.get_logger().info(
            f"Received {len(pose_array_msg.poses)} object poses for manipulation planning"
        )

        # Store detected objects
        self.detected_objects = pose_array_msg.poses

        # Filter objects based on confidence and criteria
        self.objects_to_manipulate = self.filter_objects_for_manipulation(
            self.detected_objects
        )

        # Plan manipulations if enabled
        if self.manipulation_enabled and self.objects_to_manipulate:
            self.plan_manipulations()

    def filter_objects_for_manipulation(self, objects: List[Pose]) -> List[Pose]:
        """
        Filter detected objects based on manipulation criteria

        Args:
            objects: List of detected object poses

        Returns:
            List of objects suitable for manipulation
        """
        # For this lab, we'll use all objects (in a real system, you might filter based on size, type, etc.)
        filtered_objects = []

        for i, obj_pose in enumerate(objects):
            # Calculate object distance from robot
            dist = np.sqrt(
                obj_pose.position.x**2
                + obj_pose.position.y**2
                + obj_pose.position.z**2
            )

            # Only consider objects within manipulation range
            if dist <= 1.5:  # 1.5m max reach
                filtered_objects.append(obj_pose)

        # Limit to max_objects
        return filtered_objects[: self.max_objects]

    def plan_manipulations(self):
        """
        Plan manipulations for detected objects
        """
        with self.manipulation_lock:
            # Clear previous manipulation queue
            self.manipulation_queue = []

            # Plan grasps for each object
            for obj_pose in self.objects_to_manipulate:
                # Create simple object info (in real system, this would come from perception)
                object_info = {
                    "size": (0.1, 0.1, 0.1),  # Default size
                    "shape": "cylinder",  # Default shape
                    "pose": obj_pose,
                }

                # Plan grasp for this object
                robot_info = {
                    "base_x": 0.0,
                    "base_y": 0.0,
                    "base_z": 0.0,
                    "max_reach": 1.5,
                }

                grasp_plan = self.grasp_planner.plan_grasp(
                    obj_pose, object_info, robot_info
                )

                if grasp_plan:
                    # Add to manipulation queue
                    manipulation_task = {
                        "grasp_pose": grasp_plan["pose"],
                        "object_pose": obj_pose,
                        "grasp_info": grasp_plan,
                    }
                    self.manipulation_queue.append(manipulation_task)

                    self.get_logger().info(
                        f"Added manipulation task for object at "
                        f"({obj_pose.position.x:.2f}, {obj_pose.position.y:.2f}, {obj_pose.position.z:.2f})"
                    )
                else:
                    self.get_logger().warn(
                        f"Could not plan grasp for object at "
                        f"({obj_pose.position.x:.2f}, {obj_pose.position.y:.2f}, {obj_pose.position.z:.2f})"
                    )

        # Start manipulation execution
        self.execute_manipulation_queue()

    def execute_manipulation_queue(self):
        """
        Execute the manipulation queue in a separate thread
        """
        if not self.manipulation_queue:
            self.get_logger().info("No manipulations to execute")
            return

        # Start manipulation thread if not already running
        if self.manipulation_thread is None or not self.manipulation_thread.is_alive():
            self.manipulation_thread = threading.Thread(target=self.manipulation_worker)
            self.manipulation_thread.start()

    def manipulation_worker(self):
        """
        Worker thread for executing manipulations
        """
        self.get_logger().info("Starting manipulation worker thread")

        with self.manipulation_lock:
            while self.manipulation_queue:
                # Get next manipulation task
                task = self.manipulation_queue.pop(0)

                self.get_logger().info("Executing manipulation task")

                # Calculate place pose (simple default for this lab)
                place_pose = self.calculate_place_pose(self.manipulation_completed)

                # Execute manipulation sequence
                success = self.arm_controller.execute_manipulation_sequence(
                    task["grasp_pose"], place_pose
                )

                if success:
                    self.manipulation_completed += 1
                    self.get_logger().info(
                        f"Manipulation {self.manipulation_completed} completed successfully"
                    )

                    # Publish manipulation status
                    status_header = Header()
                    status_header.stamp = self.get_clock().now().to_msg()
                    status_header.frame_id = (
                        f"manipulation_{self.manipulation_completed}_success"
                    )
                    self.manipulation_status_pub.publish(status_header)
                else:
                    self.get_logger().error(
                        f"Manipulation {self.manipulation_completed + 1} failed"
                    )

        self.get_logger().info("Manipulation worker thread completed")

    def calculate_place_pose(self, manipulation_count: int) -> Pose:
        """
        Calculate a place pose for an object based on manipulation count

        Args:
            manipulation_count: Number of manipulations completed so far

        Returns:
            Pose for placing the object
        """
        place_pose = Pose()

        # Create a grid of place positions
        grid_size = 3
        grid_spacing = 0.15  # 15cm between positions
        grid_center_x = 0.6  # Place at x=0.6m
        grid_center_y = 0.0  # Center at y=0.0m

        row = manipulation_count // grid_size
        col = manipulation_count % grid_size

        place_pose.position.x = grid_center_x
        place_pose.position.y = (
            grid_center_y + (col - (grid_size - 1) / 2) * grid_spacing
        )
        place_pose.position.z = 0.1  # Place at 10cm height

        # Default orientation
        place_pose.orientation.w = 1.0
        place_pose.orientation.x = 0.0
        place_pose.orientation.y = 0.0
        place_pose.orientation.z = 0.0

        return place_pose

    def publish_lab_visualization(self):
        """
        Publish visualization markers for the lab
        """
        if not self.enable_visualization or not self.visualization_pub:
            return

        marker_array = MarkerArray()
        current_time = self.get_clock().now()

        header = Header()
        header.stamp = current_time.to_msg()
        header.frame_id = "map"

        # Visualize detected objects
        for i, obj_pose in enumerate(self.detected_objects):
            marker = Marker()
            marker.header = header
            marker.ns = "detected_objects"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose = obj_pose
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.08

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6

            marker_array.markers.append(marker)

        # Visualize objects selected for manipulation
        for i, obj_pose in enumerate(self.objects_to_manipulate):
            marker = Marker()
            marker.header = header
            marker.ns = "objects_to_manipulate"
            marker.id = i + 100
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose = obj_pose
            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.06

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        # Visualize planned grasp poses
        with self.manipulation_lock:
            for i, task in enumerate(self.manipulation_queue):
                marker = Marker()
                marker.header = header
                marker.ns = "planned_grasps"
                marker.id = i + 200
                marker.type = Marker.ARROW
                marker.action = Marker.ADD

                # Arrow from object center to grasp position
                start_point = create_point(
                    task["object_pose"].position.x,
                    task["object_pose"].position.y,
                    task["object_pose"].position.z,
                )
                end_point = task["grasp_pose"].position

                marker.points = [start_point, end_point]

                marker.scale.x = 0.01
                marker.scale.y = 0.02
                marker.scale.z = 0.0

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8

                marker_array.markers.append(marker)

        self.visualization_pub.publish(marker_array)

    def start_lab(self):
        """
        Start the object manipulation lab
        """
        self.get_logger().info("Starting Object Manipulation Lab")
        self.lab_running = True

        # Publish lab start status
        status_header = Header()
        status_header.stamp = self.get_clock().now().to_msg()
        status_header.frame_id = "lab_started"
        self.lab_status_pub.publish(status_header)

        # Lab runs continuously, processing objects as they are detected

    def stop_lab(self):
        """
        Stop the object manipulation lab
        """
        self.get_logger().info("Stopping Object Manipulation Lab")
        self.lab_running = False

        # Publish lab stop status
        status_header = Header()
        status_header.stamp = self.get_clock().now().to_msg()
        status_header.frame_id = "lab_stopped"
        self.lab_status_pub.publish(status_header)


def main(args=None):
    """
    Main function to run the Object Manipulation Lab
    """
    rclpy.init(args=args)

    object_manipulation_lab = ObjectManipulationLab()

    try:
        object_manipulation_lab.start_lab()

        # Run visualization publishing at 10 Hz
        while rclpy.ok() and object_manipulation_lab.lab_running:
            object_manipulation_lab.publish_lab_visualization()
            time.sleep(0.1)

        rclpy.spin(object_manipulation_lab)

    except KeyboardInterrupt:
        object_manipulation_lab.get_logger().info("Interrupted by user")
    finally:
        object_manipulation_lab.stop_lab()
        object_manipulation_lab.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
