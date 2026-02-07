#!/usr/bin/env python3

"""
Isaac ROS Vision Pipeline

This module implements a comprehensive vision pipeline using Isaac ROS components
for robotic perception. The pipeline includes RGB-D processing, object detection,
pose estimation, and semantic segmentation capabilities.
"""

import threading
from collections import deque
from typing import Dict, List, Optional, Tuple

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
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class IsaacVisionPipeline(Node):
    """
    Isaac ROS Vision Pipeline for comprehensive robotic perception
    """

    def __init__(self):
        super().__init__("isaac_vision_pipeline")

        # Declare parameters
        self.declare_parameter("camera_namespace", "/camera")
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("detection_confidence_threshold", 0.6)
        self.declare_parameter("tracking_enabled", True)
        self.declare_parameter("max_tracking_objects", 10)
        self.declare_parameter(
            "object_classes", ["person", "bottle", "cup", "chair", "monitor"]
        )

        # Get parameters
        self.camera_namespace = self.get_parameter("camera_namespace").value
        self.enable_visualization = self.get_parameter("enable_visualization").value
        self.confidence_threshold = self.get_parameter(
            "detection_confidence_threshold"
        ).value
        self.tracking_enabled = self.get_parameter("tracking_enabled").value
        self.max_tracking_objects = self.get_parameter("max_tracking_objects").value
        self.object_classes = self.get_parameter("object_classes").value

        # Initialize CvBridge
        self.bridge = CvBridge()

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.object_poses_pub = self.create_publisher(PoseArray, "object_poses", 10)
        self.visualization_pub = (
            self.create_publisher(MarkerArray, "vision_markers", 10)
            if self.enable_visualization
            else None
        )
        self.tracked_objects_pub = self.create_publisher(
            PoseArray, "tracked_objects", 10
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
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            f"{self.camera_namespace}/camera_info",
            self.camera_info_callback,
            10,
        )

        # Approximate time synchronizer for RGB and Depth
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.vision_callback)

        # Store camera intrinsics
        self.camera_intrinsics = None
        self.camera_distortion = None

        # Object tracking components
        self.object_trackers = {}  # Dictionary to store object trackers
        self.object_history = {}  # Dictionary to store object history
        self.object_ids = {}  # Dictionary to maintain consistent IDs
        self.next_object_id = 0

        # Threading for performance
        self.processing_lock = threading.Lock()

        # Performance metrics
        self.frame_count = 0
        self.processing_times = deque(maxlen=30)  # Last 30 frames for FPS calculation

        self.get_logger().info("Isaac Vision Pipeline initialized")
        self.get_logger().info(
            f"Listening to camera namespace: {self.camera_namespace}"
        )
        self.get_logger().info(f"Object classes: {self.object_classes}")

    def camera_info_callback(self, camera_info_msg):
        """
        Callback for camera info to get intrinsic parameters

        Args:
            camera_info_msg: CameraInfo message with intrinsic parameters
        """
        self.camera_intrinsics = np.array(camera_info_msg.k).reshape(3, 3)
        self.camera_distortion = np.array(camera_info_msg.d)

    def vision_callback(self, rgb_msg, depth_msg):
        """
        Main vision pipeline callback for synchronized RGB and Depth images

        Args:
            rgb_msg: RGB image message
            depth_msg: Depth image message
        """
        start_time = self.get_clock().now()

        try:
            # Convert ROS images to OpenCV
            rgb_image = image_msg_to_cv2(rgb_msg)
            depth_image = image_msg_to_cv2(depth_msg, desired_encoding="32FC1")

            if rgb_image is None or depth_image is None:
                self.get_logger().warn("Failed to convert images")
                return

            # Process the vision pipeline
            detections = self.run_vision_pipeline(
                rgb_image, depth_image, rgb_msg.header
            )

            # Publish results
            self.publish_vision_results(detections, rgb_msg.header)

            # Calculate and log performance
            end_time = self.get_clock().now()
            processing_time = (
                end_time - start_time
            ).nanoseconds / 1e6  # in milliseconds
            self.processing_times.append(processing_time)

            if self.frame_count % 30 == 0:  # Log every 30 frames
                avg_processing_time = sum(self.processing_times) / len(
                    self.processing_times
                )
                fps = 1000.0 / avg_processing_time if avg_processing_time > 0 else 0
                self.get_logger().info(
                    f"Vision pipeline: {avg_processing_time:.2f}ms ({fps:.1f} FPS)"
                )

            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f"Error in vision_callback: {e}")
            import traceback

            traceback.print_exc()

    def run_vision_pipeline(
        self, rgb_image: np.ndarray, depth_image: np.ndarray, header: Header
    ) -> List[Dict]:
        """
        Run the complete vision pipeline

        Args:
            rgb_image: Input RGB image
            depth_image: Input depth image
            header: ROS header for the results

        Returns:
            List of detected objects with properties
        """
        # 1. Object Detection
        detections = self.perform_object_detection(rgb_image)

        # 2. 3D Pose Estimation
        objects_with_poses = self.estimate_3d_poses(detections, depth_image, header)

        # 3. Object Tracking (if enabled)
        if self.tracking_enabled:
            tracked_objects = self.update_object_tracking(objects_with_poses, header)
        else:
            tracked_objects = objects_with_poses

        # 4. Semantic Segmentation (simulated)
        segmented_objects = self.apply_segmentation_masks(tracked_objects, rgb_image)

        return segmented_objects

    def perform_object_detection(self, image: np.ndarray) -> List[Dict]:
        """
        Perform object detection on the input image

        Args:
            image: Input RGB image

        Returns:
            List of detections with [x, y, width, height, confidence, class_name]
        """
        # This is a simulation of object detection
        # In a real implementation, this would use Isaac ROS detection nodes
        height, width = image.shape[:2]
        detections = []

        # Simulate detecting objects based on color regions
        # In a real implementation, this would use trained models
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Define color ranges for different object classes
        color_ranges = {
            "person": (np.array([0, 20, 70]), np.array([20, 150, 255])),  # Skin tones
            "bottle": (
                np.array([80, 50, 50]),
                np.array([130, 255, 255]),
            ),  # Blue-ish objects
            "cup": (
                np.array([15, 100, 100]),
                np.array([35, 255, 255]),
            ),  # Yellow-ish objects
            "chair": (
                np.array([10, 50, 50]),
                np.array([30, 255, 255]),
            ),  # Brown-ish objects
            "monitor": (
                np.array([90, 50, 50]),
                np.array([120, 255, 255]),
            ),  # Blue screens
        }

        for class_name in self.object_classes:
            if class_name in color_ranges:
                lower, upper = color_ranges[class_name]
                mask = cv2.inRange(hsv, lower, upper)

                # Find contours
                contours, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 500:  # Filter small contours
                        # Get bounding box
                        x, y, w, h = cv2.boundingRect(contour)

                        # Calculate confidence based on area (larger objects = higher confidence)
                        confidence = min(0.95, area / (width * height * 0.5))

                        if confidence >= self.confidence_threshold:
                            detections.append([x, y, w, h, confidence, class_name])

        # Limit to top detections to avoid too many objects
        detections.sort(key=lambda x: x[4], reverse=True)  # Sort by confidence
        detections = detections[:5]  # Keep top 5 detections

        return detections

    def estimate_3d_poses(
        self, detections: List[List], depth_image: np.ndarray, header: Header
    ) -> List[Dict]:
        """
        Estimate 3D poses from 2D detections and depth information

        Args:
            detections: List of 2D detections
            depth_image: Depth image
            header: ROS header

        Returns:
            List of objects with 3D poses
        """
        objects_with_poses = []

        height, width = depth_image.shape

        for det in detections:
            x, y, w, h, conf, class_name = det

            # Calculate center of bounding box
            center_x = x + w // 2
            center_y = y + h // 2

            # Get depth at center of bounding box (with some averaging)
            depth_region = depth_image[
                max(0, center_y - 10) : min(height, center_y + 10),
                max(0, center_x - 10) : min(width, center_x + 10),
            ]

            # Calculate average depth (ignore invalid values)
            valid_depths = depth_region[depth_region > 0]
            if len(valid_depths) > 0:
                avg_depth = np.mean(valid_depths)
            else:
                avg_depth = 1.0  # Default depth if no valid readings

            # Convert pixel coordinates to 3D world coordinates using camera intrinsics
            if self.camera_intrinsics is not None:
                # Use camera intrinsics for more accurate conversion
                cx = self.camera_intrinsics[0, 2]
                cy = self.camera_intrinsics[1, 2]
                fx = self.camera_intrinsics[0, 0]
                fy = self.camera_intrinsics[1, 1]

                world_x = (center_x - cx) * avg_depth / fx
                world_y = (center_y - cy) * avg_depth / fy
                world_z = avg_depth
            else:
                # Fallback conversion
                world_x = (center_x - width / 2) * avg_depth * 0.001
                world_y = (center_y - height / 2) * avg_depth * 0.001
                world_z = avg_depth

            # Create pose
            pose = create_pose(world_x, world_y, world_z)

            # Store object information
            object_info = {
                "class": class_name,
                "confidence": conf,
                "pose": pose,
                "bbox": (x, y, w, h),
                "depth": avg_depth,
            }
            objects_with_poses.append(object_info)

        return objects_with_poses

    def update_object_tracking(
        self, detected_objects: List[Dict], header: Header
    ) -> List[Dict]:
        """
        Update object tracking with detected objects

        Args:
            detected_objects: List of newly detected objects
            header: ROS header

        Returns:
            List of tracked objects with consistent IDs
        """
        if not self.object_trackers:
            # Initialize tracking for first frame
            for i, obj in enumerate(detected_objects):
                obj_id = f"obj_{self.next_object_id}"
                self.next_object_id += 1
                obj["id"] = obj_id
                self.object_ids[obj_id] = obj
                self.object_history[obj_id] = [obj]
            return detected_objects

        # For this simulation, we'll use a simple nearest neighbor approach
        # In a real implementation, this would use more sophisticated tracking algorithms
        tracked_objects = []

        for detected_obj in detected_objects:
            best_match = None
            min_distance = float("inf")

            # Find best matching tracked object
            for obj_id, tracked_obj in self.object_ids.items():
                # Calculate distance between detected and tracked poses
                dist = np.sqrt(
                    (detected_obj["pose"].position.x - tracked_obj["pose"].position.x)
                    ** 2
                    + (detected_obj["pose"].position.y - tracked_obj["pose"].position.y)
                    ** 2
                    + (detected_obj["pose"].position.z - tracked_obj["pose"].position.z)
                    ** 2
                )

                if dist < min_distance and dist < 0.2:  # 20cm threshold
                    min_distance = dist
                    best_match = obj_id

            if best_match is not None:
                # Update existing tracked object
                self.object_ids[best_match].update(detected_obj)
                self.object_ids[best_match]["id"] = best_match  # Ensure ID is preserved
                self.object_history[best_match].append(detected_obj)

                # Keep only recent history
                if len(self.object_history[best_match]) > 10:
                    self.object_history[best_match] = self.object_history[best_match][
                        -10:
                    ]

                tracked_objects.append(self.object_ids[best_match])
            else:
                # Create new track for this object
                if len(self.object_ids) < self.max_tracking_objects:
                    obj_id = f"obj_{self.next_object_id}"
                    self.next_object_id += 1
                    detected_obj["id"] = obj_id
                    self.object_ids[obj_id] = detected_obj
                    self.object_history[obj_id] = [detected_obj]
                    tracked_objects.append(detected_obj)

        # Remove old tracks that haven't been seen recently
        current_time = header.stamp.sec + header.stamp.nanosec / 1e9
        objects_to_remove = []
        for obj_id, history in self.object_history.items():
            if len(history) == 0:
                objects_to_remove.append(obj_id)

        for obj_id in objects_to_remove:
            self.object_ids.pop(obj_id, None)
            self.object_history.pop(obj_id, None)

        return tracked_objects

    def apply_segmentation_masks(
        self, objects: List[Dict], rgb_image: np.ndarray
    ) -> List[Dict]:
        """
        Apply segmentation masks to objects (simulated)

        Args:
            objects: List of objects with poses
            rgb_image: Input RGB image

        Returns:
            List of objects with segmentation information
        """
        # In a real implementation, this would apply semantic segmentation
        # For this simulation, we'll just add a dummy segmentation property
        for obj in objects:
            obj["segmentation_mask"] = True  # Placeholder for segmentation mask

        return objects

    def publish_vision_results(self, objects: List[Dict], header: Header):
        """
        Publish vision pipeline results

        Args:
            objects: List of detected/tracked objects
            header: ROS header for published messages
        """
        # Create and publish PoseArray for object poses
        pose_array = PoseArray()
        pose_array.header = header

        for obj in objects:
            pose_array.poses.append(obj["pose"])

        self.object_poses_pub.publish(pose_array)

        # Create and publish tracked objects
        tracked_pose_array = PoseArray()
        tracked_pose_array.header = header
        for obj in objects:
            if "id" in obj:
                # For visualization, we'll use the pose
                tracked_pose_array.poses.append(obj["pose"])

        self.tracked_objects_pub.publish(tracked_pose_array)

        # Publish visualization markers if enabled
        if self.enable_visualization and self.visualization_pub:
            self.publish_vision_markers(objects, header)

    def publish_vision_markers(self, objects: List[Dict], header: Header):
        """
        Publish visualization markers for detected objects

        Args:
            objects: List of detected objects
            header: Header for the published markers
        """
        marker_array = MarkerArray()

        for i, obj in enumerate(objects):
            # Create a marker for the object
            marker = Marker()
            marker.header = header
            marker.ns = "vision_pipeline"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position and orientation
            marker.pose = obj["pose"]

            # Scale based on detection confidence
            scale_factor = (
                0.1 + (obj["confidence"] - 0.5) * 0.1
            )  # Scale between 0.1 and 0.15
            marker.scale.x = scale_factor
            marker.scale.y = scale_factor
            marker.scale.z = scale_factor

            # Color based on object class
            if obj["class"] == "person":
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif obj["class"] == "bottle":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif obj["class"] == "cup":
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif obj["class"] == "chair":
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif obj["class"] == "monitor":
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5

            marker.color.a = 0.8  # Alpha

            # Add text label
            label_marker = Marker()
            label_marker.header = header
            label_marker.ns = "vision_labels"
            label_marker.id = i + 1000  # Different ID range to avoid conflicts
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD

            # Position text slightly above the object
            label_marker.pose = obj["pose"]
            label_marker.pose.position.z += 0.15

            label_marker.scale.z = 0.1  # Text scale
            label_marker.color.r = 1.0
            label_marker.color.g = 1.0
            label_marker.color.b = 1.0
            label_marker.color.a = 1.0

            label_marker.text = f"{obj['class']} ({obj['confidence']:.2f})"

            marker_array.markers.append(marker)
            marker_array.markers.append(label_marker)

        self.visualization_pub.publish(marker_array)


def main(args=None):
    """
    Main function to run the Isaac Vision Pipeline
    """
    rclpy.init(args=args)

    vision_pipeline = IsaacVisionPipeline()

    try:
        rclpy.spin(vision_pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        vision_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
