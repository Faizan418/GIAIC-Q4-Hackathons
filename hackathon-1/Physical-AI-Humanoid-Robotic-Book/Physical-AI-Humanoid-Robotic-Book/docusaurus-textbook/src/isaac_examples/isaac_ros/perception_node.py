#!/usr/bin/env python3

"""
Isaac ROS Perception Node

This node demonstrates GPU-accelerated perception using Isaac ROS.
It processes RGB-D data to perform object detection and pose estimation.
"""

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseArray

# Import common utilities
from isaac_examples.common.isaac_ros_utils import (
    create_point,
    create_pose,
    cv2_to_image_msg,
    get_transform,
    image_msg_to_cv2,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class IsaacPerceptionNode(Node):
    """
    Isaac ROS Perception Node for processing sensor data with GPU acceleration
    """

    def __init__(self):
        super().__init__("isaac_perception_node")

        # Declare parameters
        self.declare_parameter("camera_namespace", "/camera")
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("detection_confidence_threshold", 0.5)
        self.declare_parameter("object_classes", ["person", "bottle", "cup", "chair"])

        # Get parameters
        self.camera_namespace = self.get_parameter("camera_namespace").value
        self.enable_visualization = self.get_parameter("enable_visualization").value
        self.confidence_threshold = self.get_parameter(
            "detection_confidence_threshold"
        ).value
        self.object_classes = self.get_parameter("object_classes").value

        # Initialize CvBridge
        self.bridge = CvBridge()

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.detection_pub = self.create_publisher(PoseArray, "object_detections", 10)
        self.visualization_pub = (
            self.create_publisher(MarkerArray, "detection_markers", 10)
            if self.enable_visualization
            else None
        )

        # Create QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers for RGB and Depth images
        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            f"{self.camera_namespace}/rgb/image_raw",
            qos_profile=qos_profile,
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            f"{self.camera_namespace}/depth/image_raw",
            qos_profile=qos_profile,
        )

        # Approximate time synchronizer for RGB and Depth
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        # Store camera info (in a real implementation, this would come from a camera_info topic)
        self.camera_info = None

        # Object detection simulation (in a real implementation, this would use Isaac ROS perception packages)
        self.get_logger().info("Isaac Perception Node initialized")
        self.get_logger().info(
            f"Listening to camera namespace: {self.camera_namespace}"
        )

    def sync_callback(self, rgb_msg, depth_msg):
        """
        Callback for synchronized RGB and Depth images
        """
        try:
            # Convert ROS images to OpenCV
            rgb_image = image_msg_to_cv2(rgb_msg)
            depth_image = image_msg_to_cv2(depth_msg, desired_encoding="32FC1")

            if rgb_image is None or depth_image is None:
                self.get_logger().warn("Failed to convert images")
                return

            # Perform object detection (simulated)
            detections = self.simulate_object_detection(rgb_image)

            # Estimate 3D poses from 2D detections and depth
            object_poses = self.estimate_poses_from_detections(
                detections, depth_image, rgb_msg.header
            )

            # Publish detections
            self.publish_detections(object_poses, rgb_msg.header)

            # Publish visualization markers
            if self.enable_visualization and self.visualization_pub:
                self.publish_visualization_markers(object_poses, rgb_msg.header)

        except Exception as e:
            self.get_logger().error(f"Error in sync_callback: {e}")

    def simulate_object_detection(self, image):
        """
        Simulate object detection (in a real implementation, this would use Isaac ROS detection nodes)

        Args:
            image: Input RGB image

        Returns:
            List of detections with format [x, y, width, height, confidence, class_name]
        """
        # This is a simulation - in a real implementation, this would use Isaac ROS detection nodes
        # For demonstration, we'll simulate detecting some objects

        height, width = image.shape[:2]
        detections = []

        # Simulate detecting a few objects at known positions
        # Format: [x, y, width, height, confidence, class_name]
        simulated_objects = [
            [
                int(0.3 * width),
                int(0.4 * height),
                int(0.1 * width),
                int(0.1 * height),
                0.85,
                "bottle",
            ],
            [
                int(0.6 * width),
                int(0.5 * height),
                int(0.08 * width),
                int(0.08 * height),
                0.78,
                "cup",
            ],
            [
                int(0.2 * width),
                int(0.6 * height),
                int(0.12 * width),
                int(0.15 * height),
                0.92,
                "person",
            ],
        ]

        # Filter based on confidence threshold and class list
        for det in simulated_objects:
            x, y, w, h, conf, class_name = det
            if conf >= self.confidence_threshold and class_name in self.object_classes:
                detections.append(det)

        return detections

    def estimate_poses_from_detections(self, detections, depth_image, header):
        """
        Estimate 3D poses from 2D detections and depth information

        Args:
            detections: List of 2D detections
            depth_image: Depth image
            header: ROS header for the pose array

        Returns:
            PoseArray with estimated 3D poses
        """
        poses = PoseArray()
        poses.header = header

        height, width = depth_image.shape

        for det in detections:
            x, y, w, h, conf, class_name = det

            # Calculate center of bounding box
            center_x = x + w // 2
            center_y = y + h // 2

            # Get depth at center of bounding box (with some averaging)
            depth_region = depth_image[
                max(0, center_y - 5) : min(height, center_y + 5),
                max(0, center_x - 5) : min(width, center_x + 5),
            ]

            # Calculate average depth (ignore invalid values)
            valid_depths = depth_region[depth_region > 0]
            if len(valid_depths) > 0:
                avg_depth = np.mean(valid_depths)
            else:
                avg_depth = 1.0  # Default depth if no valid readings

            # Convert pixel coordinates to 3D world coordinates (simplified)
            # In a real implementation, you would use camera intrinsics
            # For now, we'll use a simple approximation
            world_x = (center_x - width / 2) * avg_depth * 0.001  # Scale factor
            world_y = (center_y - height / 2) * avg_depth * 0.001  # Scale factor
            world_z = avg_depth

            # Create pose
            pose = create_pose(world_x, world_y, world_z)
            poses.poses.append(pose)

            self.get_logger().info(
                f"Detected {class_name} at 3D position: ({world_x:.2f}, {world_y:.2f}, {world_z:.2f})"
            )

        return poses

    def publish_detections(self, poses, header):
        """
        Publish object detection results

        Args:
            poses: PoseArray with object poses
            header: Header to use for the published message
        """
        poses.header = header
        self.detection_pub.publish(poses)

    def publish_visualization_markers(self, poses, header):
        """
        Publish visualization markers for detected objects

        Args:
            poses: PoseArray with object poses
            header: Header to use for the published message
        """
        marker_array = MarkerArray()

        for i, pose in enumerate(poses.poses):
            # Create a marker for each detected object
            marker = Marker()
            marker.header = header
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position and orientation
            marker.pose = pose
            marker.pose.position.z += 0.1  # Slightly above the object

            # Scale (size of the marker)
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Color (blue for detected objects)
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8  # Alpha

            marker_array.markers.append(marker)

        self.visualization_pub.publish(marker_array)

    def set_camera_info(self, camera_info_msg):
        """
        Set camera information for 3D reconstruction

        Args:
            camera_info_msg: CameraInfo message with intrinsic parameters
        """
        self.camera_info = camera_info_msg


def main(args=None):
    """
    Main function to run the Isaac Perception Node
    """
    rclpy.init(args=args)

    perception_node = IsaacPerceptionNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
