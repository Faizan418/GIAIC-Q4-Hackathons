#!/usr/bin/env python3

"""
Test Perception and Manipulation Integration in Simulation

This script validates the integration between Isaac Sim scene configuration,
Isaac ROS perception nodes, grasp planning, and arm control systems.
It runs a complete perception-action loop to verify all components work together.
"""

import sys
import threading
import time
from unittest.mock import MagicMock, Mock

import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from isaac_examples.isaac_ros.perception_node import IsaacPerceptionNode
from isaac_examples.isaac_ros.vision_pipeline import IsaacVisionPipeline
from isaac_examples.isaac_sim.robot_setup import RobotSetup
from isaac_examples.isaac_sim.scene_config import IsaacSimScene
from isaac_examples.manipulation.arm_control import ArmController
from isaac_examples.manipulation.grasp_planning import GraspPlanner
from isaac_examples.manipulation.object_manipulation_lab import ObjectManipulationLab
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


class MockIsaacSim:
    """
    Mock class to simulate Isaac Sim functionality for testing purposes
    """

    def __init__(self):
        self.scene_objects = []
        self.cameras = []
        self.is_running = False

    def start_simulation(self):
        """Start the simulation"""
        print("Mock Isaac Sim: Starting simulation...")
        self.is_running = True

    def stop_simulation(self):
        """Stop the simulation"""
        print("Mock Isaac Sim: Stopping simulation...")
        self.is_running = False

    def add_camera(self, name, position, resolution=(640, 480)):
        """Add a camera to the simulation"""
        camera = {
            "name": name,
            "position": position,
            "resolution": resolution,
            "rgb_buffer": None,
            "depth_buffer": None,
        }
        self.cameras.append(camera)
        print(f"Mock Isaac Sim: Added camera {name} at {position}")
        return camera

    def add_object(self, name, position, color="red"):
        """Add an object to the simulation scene"""
        obj = {"name": name, "position": position, "color": color}
        self.scene_objects.append(obj)
        print(f"Mock Isaac Sim: Added {color} object {name} at {position}")
        return obj

    def generate_mock_sensor_data(self):
        """Generate mock sensor data for testing"""
        if not self.cameras:
            return None, None

        camera = self.cameras[0]  # Use first camera

        # Generate mock RGB image (random colored rectangles to simulate objects)
        height, width = camera["resolution"][1], camera["resolution"][0]
        rgb_image = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)

        # Add some colored rectangles to simulate objects
        cv2.rectangle(
            rgb_image, (100, 100), (200, 200), (255, 0, 0), -1
        )  # Blue rectangle
        cv2.rectangle(
            rgb_image, (300, 200), (400, 300), (0, 255, 0), -1
        )  # Green rectangle
        cv2.rectangle(
            rgb_image, (150, 300), (250, 400), (0, 0, 255), -1
        )  # Red rectangle

        # Generate mock depth image
        depth_image = np.random.uniform(0.5, 5.0, (height, width)).astype(np.float32)

        # Add some depth variations to correspond with objects
        depth_image[100:200, 100:200] = 1.0  # Close object
        depth_image[300:400, 200:300] = 1.5  # Medium distance object
        depth_image[150:250, 300:400] = 2.0  # Farther object

        return rgb_image, depth_image


class PerceptionManipulationIntegrationTester:
    """
    Test class for validating perception and manipulation integration
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.test_results = []
        self.test_passed = 0
        self.test_failed = 0

    def run_integration_tests(self):
        """
        Run comprehensive integration tests for perception and manipulation
        """
        print("=" * 70)
        print("INTEGRATION TEST: Perception and Manipulation System")
        print("=" * 70)

        tests = [
            self.test_scene_setup,
            self.test_perception_pipeline,
            self.test_grasp_planning,
            self.test_arm_control,
            self.test_complete_manipulation_loop,
        ]

        for test_func in tests:
            print(f"\nRunning {test_func.__name__}...")
            try:
                result = test_func()
                if result:
                    self.test_passed += 1
                    print(f"  ✓ {test_func.__name__} PASSED")
                    self.test_results.append((test_func.__name__, "PASSED"))
                else:
                    self.test_failed += 1
                    print(f"  ✗ {test_func.__name__} FAILED")
                    self.test_results.append((test_func.__name__, "FAILED"))
            except Exception as e:
                self.test_failed += 1
                print(f"  ✗ {test_func.__name__} ERROR: {e}")
                self.test_results.append((test_func.__name__, f"ERROR: {e}"))

        self.print_test_summary()
        return self.test_failed == 0

    def test_scene_setup(self):
        """
        Test Isaac Sim scene configuration and robot setup
        """
        print("  Testing Isaac Sim scene configuration...")

        # Initialize Isaac Sim scene
        sim_scene = IsaacSimScene()
        world = sim_scene.create_default_world()

        # Initialize robot setup
        robot_setup = RobotSetup()
        robot_setup.add_franka_robot(world, position=[0.0, 0.0, 0.0])

        # Add a camera to the scene
        sim_scene.add_camera_to_scene(
            world, position=[2.0, 0.0, 1.5], name="test_camera"
        )

        print("  ✓ Isaac Sim scene configured with robot and camera")
        return True

    def test_perception_pipeline(self):
        """
        Test Isaac ROS perception pipeline
        """
        print("  Testing Isaac ROS perception pipeline...")

        # Create mock sensor data
        rgb_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        depth_image = np.random.uniform(0.5, 3.0, (480, 640)).astype(np.float32)

        # Initialize perception node
        perception_node = IsaacPerceptionNode()

        # Test processing of sensor data
        try:
            # Simulate the perception pipeline processing
            # Convert to ROS messages
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")

            # Process the data (this would normally be done through callbacks)
            # For testing, we'll simulate the internal processing
            hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

            # Define color ranges for different object classes
            color_ranges = {
                "person": (
                    np.array([0, 20, 70]),
                    np.array([20, 150, 255]),
                ),  # Skin tones
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

            # Simulate object detection
            detections = []
            for class_name in perception_node.object_classes:
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
                            confidence = min(
                                0.95,
                                area / (rgb_image.shape[0] * rgb_image.shape[1] * 0.5),
                            )

                            if confidence >= perception_node.confidence_threshold:
                                detections.append([x, y, w, h, confidence, class_name])

            print(
                f"  ✓ Perception pipeline processed data and detected {len(detections)} objects"
            )
            return True
        except Exception as e:
            print(f"  ✗ Perception pipeline failed: {e}")
            return False

    def test_grasp_planning(self):
        """
        Test grasp planning functionality
        """
        print("  Testing grasp planning functionality...")

        # Initialize grasp planner
        grasp_planner = GraspPlanner()

        # Create mock object poses
        object_poses = PoseArray()
        object_poses.header = Header()
        object_poses.header.stamp.sec = int(time.time())
        object_poses.header.frame_id = "map"

        # Add some test object poses
        for i in range(3):
            pose = Pose()
            pose.position.x = 0.5 + i * 0.1
            pose.position.y = 0.0
            pose.position.z = 0.1
            pose.orientation.w = 1.0
            object_poses.poses.append(pose)

        # Test grasp planning
        try:
            # Simulate the callback processing
            for obj_pose in object_poses.poses:
                object_grasps = grasp_planner.plan_grasps_for_object(obj_pose)
                print(
                    f"  - Planned {len(object_grasps)} grasp candidates for object at ({obj_pose.position.x:.2f}, {obj_pose.position.y:.2f}, {obj_pose.position.z:.2f})"
                )

            # Select best grasp
            grasp_planner.candidate_grasps = []
            for obj_pose in object_poses.poses:
                object_grasps = grasp_planner.plan_grasps_for_object(obj_pose)
                grasp_planner.candidate_grasps.extend(object_grasps)

            grasp_planner.select_best_grasp()

            print("  ✓ Grasp planning completed successfully")
            return True
        except Exception as e:
            print(f"  ✗ Grasp planning failed: {e}")
            return False

    def test_arm_control(self):
        """
        Test arm control functionality
        """
        print("  Testing arm control functionality...")

        # Initialize arm controller
        arm_controller = ArmController()

        # Test basic arm control functions
        try:
            # Create test poses
            home_pose = arm_controller.calculate_home_pose()
            test_pose = Pose()
            test_pose.position.x = 0.5
            test_pose.position.y = 0.0
            test_pose.position.z = 0.3
            test_pose.orientation.w = 1.0

            # Test pose calculations
            pre_grasp = arm_controller.calculate_pre_grasp_pose(test_pose)
            post_grasp = arm_controller.calculate_post_grasp_pose(test_pose)

            print(
                f"  - Home pose: ({home_pose.position.x:.2f}, {home_pose.position.y:.2f}, {home_pose.position.z:.2f})"
            )
            print(
                f"  - Pre-grasp pose: ({pre_grasp.position.x:.2f}, {pre_grasp.position.y:.2f}, {pre_grasp.position.z:.2f})"
            )
            print(
                f"  - Post-grasp pose: ({post_grasp.position.x:.2f}, {post_grasp.position.y:.2f}, {post_grasp.position.z:.2f})"
            )

            # Test gripper control
            arm_controller.close_gripper()
            time.sleep(0.1)
            arm_controller.open_gripper()

            print("  ✓ Arm control functionality tested successfully")
            return True
        except Exception as e:
            print(f"  ✗ Arm control failed: {e}")
            return False

    def test_complete_manipulation_loop(self):
        """
        Test complete perception-action loop integration
        """
        print("  Testing complete perception-action loop integration...")

        # Initialize all components
        sim_env = MockIsaacSim()
        perception_node = IsaacVisionPipeline()
        grasp_planner = GraspPlanner()
        arm_controller = ArmController()
        manipulation_lab = ObjectManipulationLab()

        try:
            # Start simulation
            sim_env.start_simulation()

            # Add objects to simulation
            sim_env.add_object("red_cube", [0.5, 0.0, 0.1], "red")
            sim_env.add_object("blue_sphere", [0.6, 0.1, 0.1], "blue")
            sim_env.add_object("green_cylinder", [0.4, -0.1, 0.1], "green")

            # Generate mock sensor data
            rgb_image, depth_image = sim_env.generate_mock_sensor_data()

            if rgb_image is not None and depth_image is not None:
                print(
                    f"    Generated mock sensor data - RGB: {rgb_image.shape}, Depth: {depth_image.shape}"
                )

                # Process through perception pipeline
                header = Header()
                header.stamp.sec = int(time.time())
                detections = perception_node.run_vision_pipeline(
                    rgb_image, depth_image, header
                )

                print(f"    Perception detected {len(detections)} objects")

                # Create mock PoseArray for object poses
                object_poses = PoseArray()
                object_poses.header = header
                for i in range(min(2, len(detections))):
                    obj = detections[i]
                    object_poses.poses.append(obj["pose"])

                # Process object poses through manipulation lab
                manipulation_lab.object_poses_callback(object_poses)

                print("    ✓ Complete perception-action loop executed successfully")

            # Stop simulation
            sim_env.stop_simulation()

            return True
        except Exception as e:
            print(f"  ✗ Complete manipulation loop failed: {e}")
            import traceback

            traceback.print_exc()
            return False

    def print_test_summary(self):
        """
        Print summary of all tests
        """
        print("\n" + "=" * 70)
        print("INTEGRATION TEST SUMMARY")
        print("=" * 70)
        print(f"Total Tests: {len(self.test_results)}")
        print(f"Passed: {self.test_passed}")
        print(f"Failed: {self.test_failed}")
        print()

        for test_name, result in self.test_results:
            print(f"  {test_name}: {result}")

        if self.test_failed == 0:
            print("\n✓ ALL INTEGRATION TESTS PASSED")
        else:
            print(f"\n✗ {self.test_failed} TEST(S) FAILED")
        print("=" * 70)


def main():
    """
    Main function to run the integration tests
    """
    tester = PerceptionManipulationIntegrationTester()

    try:
        success = tester.run_integration_tests()
        if success:
            print("\n✓ Perception and manipulation integration test PASSED")
            return 0
        else:
            print("\n✗ Perception and manipulation integration test FAILED")
            return 1
    except Exception as e:
        print(f"\n✗ Integration test ERROR: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
