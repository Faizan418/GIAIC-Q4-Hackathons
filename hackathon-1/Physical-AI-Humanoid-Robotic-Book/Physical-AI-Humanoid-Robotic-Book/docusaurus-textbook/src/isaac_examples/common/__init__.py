"""
Common utilities and dependencies for Isaac ROS examples
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

# Initialize CvBridge for image conversion
bridge = CvBridge()


def initialize_ros():
    """Initialize ROS 2 context"""
    rclpy.init()


def shutdown_ros():
    """Shutdown ROS 2 context"""
    rclpy.shutdown()


def image_msg_to_cv2(image_msg):
    """Convert ROS Image message to OpenCV image"""
    return bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")


def cv2_to_image_msg(cv_image, encoding="bgr8"):
    """Convert OpenCV image to ROS Image message"""
    return bridge.cv2_to_imgmsg(cv_image, encoding=encoding)


def create_pose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """Create a Pose message with given position and orientation"""
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose
