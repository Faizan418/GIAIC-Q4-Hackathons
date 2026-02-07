"""
Isaac ROS specific utilities and common imports
"""

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# Initialize CvBridge for image conversion
bridge = CvBridge()


def initialize_ros():
    """Initialize ROS 2 context"""
    if not rclpy.ok():
        rclpy.init()


def shutdown_ros():
    """Shutdown ROS 2 context"""
    rclpy.shutdown()


def image_msg_to_cv2(image_msg, desired_encoding="passthrough"):
    """Convert ROS Image message to OpenCV image"""
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding=desired_encoding)
        return cv_image
    except Exception as e:
        print(f"Error converting image message to CV2: {e}")
        return None


def cv2_to_image_msg(cv_image, encoding="bgr8"):
    """Convert OpenCV image to ROS Image message"""
    try:
        image_msg = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        return image_msg
    except Exception as e:
        print(f"Error converting CV2 image to message: {e}")
        return None


def create_pose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """Create a Pose message with given position and orientation"""
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def create_point(x=0.0, y=0.0, z=0.0):
    """Create a Point message with given coordinates"""
    return Point(x=x, y=y, z=z)


def create_vector3(x=0.0, y=0.0, z=0.0):
    """Create a Vector3 message with given coordinates"""
    return Vector3(x=x, y=y, z=z)


def create_marker(marker_id, marker_type, pose, scale, color, frame_id="base_link"):
    """Create a visualization marker"""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = Time()
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale = scale
    marker.color = color
    marker.lifetime = Time()
    return marker


def get_transform(tf_buffer, target_frame, source_frame, timeout=1.0):
    """Get transform between two frames"""
    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=timeout),
        )
        return transform
    except tf2_ros.TransformException as e:
        print(f"Transform lookup failed: {e}")
        return None
