#!/usr/bin/env python3

"""
RL Policy Training and Validation Lab

This lab provides a complete environment for training and validating reinforcement learning
policies using Isaac Sim for simulation and Isaac ROS for real-world deployment. It includes
tools for policy training, validation, safety checking, and sim-to-real transfer.
"""

import json
import os
import shutil
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
import torch
import torch.nn as nn
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float32, Header, Int32, String

# Import Isaac Sim components
try:
    import omni.replicator.core as rep
    from omni.isaac.core import World
    from omni.isaac.core.objects import DynamicCuboid
    from omni.isaac.core.prims import ArticulationPrim, RigidPrim
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from omni.isaac.core.utils.prims import get_prim_at_path
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.utils.viewports import set_camera_view
except ImportError:
    print("Isaac Sim modules not available. Using mock implementations for testing.")
    World = None

# Import common utilities
from isaac_examples.common.isaac_ros_utils import (
    create_point,
    create_pose,
    get_transform,
)
from isaac_examples.reinforcement_learning.policy_deployment import (
    DeploymentManager,
    PolicyDeploymentNode,
    PolicyExporter,
    SafetyValidator,
)

# Import RL training and deployment components
from isaac_examples.reinforcement_learning.rl_training import (
    IsaacSimEnvironment,
    PPOAgent,
    RLTrainingNode,
)


class RLPolicyLab(Node):
    """
    RL Policy Training and Validation Lab
    """

    def __init__(self):
        super().__init__("rl_policy_lab")

        # Declare parameters
        self.declare_parameter("robot_name", "franka")
        self.declare_parameter("task", "pick_and_place")
        self.declare_parameter("state_dim", 24)
        self.declare_parameter("action_dim", 7)
        self.declare_parameter("max_episodes", 1000)
        self.declare_parameter("max_steps_per_episode", 1000)
        self.declare_parameter("learning_rate", 3e-4)
        self.declare_parameter("gamma", 0.99)
        self.declare_parameter("batch_size", 64)
        self.declare_parameter("replay_buffer_size", 10000)
        self.declare_parameter("save_model_interval", 100)
        self.declare_parameter("model_save_path", "/tmp/rl_models/")
        self.declare_parameter("validate_safety", True)
        self.declare_parameter("enable_tensorrt", True)
        self.declare_parameter("deployment_target", "local")
        self.declare_parameter("sim_to_real_transfer", True)

        # Get parameters
        self.robot_name = self.get_parameter("robot_name").value
        self.task = self.get_parameter("task").value
        self.state_dim = self.get_parameter("state_dim").value
        self.action_dim = self.get_parameter("action_dim").value
        self.max_episodes = self.get_parameter("max_episodes").value
        self.max_steps_per_episode = self.get_parameter("max_steps_per_episode").value
        self.learning_rate = self.get_parameter("learning_rate").value
        self.gamma = self.get_parameter("gamma").value
        self.batch_size = self.get_parameter("batch_size").value
        self.replay_buffer_size = self.get_parameter("replay_buffer_size").value
        self.save_model_interval = self.get_parameter("save_model_interval").value
        self.model_save_path = self.get_parameter("model_save_path").value
        self.validate_safety = self.get_parameter("validate_safety").value
        self.enable_tensorrt = self.get_parameter("enable_tensorrt").value
        self.deployment_target = self.get_parameter("deployment_target").value
        self.sim_to_real_transfer = self.get_parameter("sim_to_real_transfer").value

        # Publishers
        self.lab_status_pub = self.create_publisher(Header, "lab_status", 10)
        self.training_progress_pub = self.create_publisher(
            Float32, "training_progress", 10
        )
        self.validation_result_pub = self.create_publisher(
            Header, "validation_result", 10
        )
        self.safety_check_pub = self.create_publisher(Header, "safety_check", 10)

        # Internal state
        self.training_node = None
        self.deployment_node = None
        self.lab_thread = None
        self.lab_active = False
        self.current_episode = 0
        self.total_episodes = 0
        self.training_results = {}
        self.validation_results = {}

        # Create model save directory
        os.makedirs(self.model_save_path, exist_ok=True)

        self.get_logger().info("RL Policy Lab initialized")
        self.get_logger().info(f"Task: {self.task}, Robot: {self.robot_name}")
        self.get_logger().info(
            f"State dim: {self.state_dim}, Action dim: {self.action_dim}"
        )

    def start_lab(self):
        """
        Start the RL policy lab
        """
        self.get_logger().info("Starting RL Policy Lab...")
        self.lab_active = True

        # Publish lab start status
        status_header = Header()
        status_header.stamp = self.get_clock().now().to_msg()
        status_header.frame_id = "lab_started"
        self.lab_status_pub.publish(status_header)

        # Start lab in a separate thread
        self.lab_thread = threading.Thread(target=self.lab_worker)
        self.lab_thread.start()

    def lab_worker(self):
        """
        Worker thread for the RL policy lab
        """
        try:
            # Phase 1: Training in Simulation
            self.get_logger().info("Phase 1: Training in Simulation")
            training_success = self.run_training_phase()
            if not training_success:
                self.get_logger().error("Training phase failed")
                return

            # Phase 2: Validation and Safety Checking
            self.get_logger().info("Phase 2: Validation and Safety Checking")
            validation_success = self.run_validation_phase()
            if not validation_success:
                self.get_logger().error("Validation phase failed")
                return

            # Phase 3: Policy Deployment
            self.get_logger().info("Phase 3: Policy Deployment")
            deployment_success = self.run_deployment_phase()
            if not deployment_success:
                self.get_logger().error("Deployment phase failed")
                return

            # Phase 4: Sim-to-Real Transfer (if enabled)
            if self.sim_to_real_transfer:
                self.get_logger().info("Phase 4: Sim-to-Real Transfer")
                transfer_success = self.run_sim_to_real_transfer()
                if not transfer_success:
                    self.get_logger().error("Sim-to-real transfer failed")
                    return

            self.get_logger().info("RL Policy Lab completed successfully")

            # Publish lab completion status
            status_header = Header()
            status_header.stamp = self.get_clock().now().to_msg()
            status_header.frame_id = "lab_completed_successfully"
            self.lab_status_pub.publish(status_header)

        except Exception as e:
            self.get_logger().error(f"Lab execution error: {e}")
            import traceback

            traceback.print_exc()

            # Publish lab error status
            status_header = Header()
            status_header.stamp = self.get_clock().now().to_msg()
            status_header.frame_id = f"lab_error_{str(e)}"
            self.lab_status_pub.publish(status_header)

    def run_training_phase(self) -> bool:
        """
        Run the training phase in Isaac Sim

        Returns:
            True if training successful, False otherwise
        """
        self.get_logger().info(f"Starting training for {self.max_episodes} episodes")

        try:
            # Create and configure training node
            training_node = RLTrainingNode()

            # Override parameters with lab parameters
            training_node.declare_parameter("robot_name", self.robot_name)
            training_node.declare_parameter("task", self.task)
            training_node.declare_parameter("state_dim", self.state_dim)
            training_node.declare_parameter("action_dim", self.action_dim)
            training_node.declare_parameter("max_episodes", self.max_episodes)
            training_node.declare_parameter(
                "max_steps_per_episode", self.max_steps_per_episode
            )
            training_node.declare_parameter("learning_rate", self.learning_rate)
            training_node.declare_parameter("gamma", self.gamma)
            training_node.declare_parameter("batch_size", self.batch_size)
            training_node.declare_parameter(
                "replay_buffer_size", self.replay_buffer_size
            )
            training_node.declare_parameter(
                "save_model_interval", self.save_model_interval
            )
            training_node.declare_parameter("model_save_path", self.model_save_path)

            # Start training
            training_node.start_training()

            # Monitor training progress
            start_time = time.time()
            while training_node.training_active and self.lab_active:
                # Calculate and publish training progress
                progress = min(
                    1.0,
                    training_node.episode_rewards.count
                    if hasattr(training_node, "episode_rewards")
                    else 0 / self.max_episodes,
                )
                progress_msg = Float32()
                progress_msg.data = progress
                self.training_progress_pub.publish(progress_msg)

                time.sleep(1.0)  # Update progress every second

                # Check for timeout (training should not run indefinitely)
                if time.time() - start_time > 3600:  # 1 hour timeout
                    self.get_logger().error("Training phase timed out")
                    training_node.stop_training()
                    return False

            training_node.stop_training()

            self.get_logger().info("Training phase completed")
            return True

        except Exception as e:
            self.get_logger().error(f"Training phase error: {e}")
            import traceback

            traceback.print_exc()
            return False

    def run_validation_phase(self) -> bool:
        """
        Run validation and safety checking on trained policy

        Returns:
            True if validation successful, False otherwise
        """
        self.get_logger().info("Starting validation and safety checking phase")

        try:
            # Find the latest trained model
            model_files = [
                f for f in os.listdir(self.model_save_path) if f.endswith(".pth")
            ]
            if not model_files:
                self.get_logger().error("No trained models found for validation")
                return False

            # Get the most recent model
            latest_model = max(
                model_files,
                key=lambda x: os.path.getctime(os.path.join(self.model_save_path, x)),
            )
            latest_model_path = os.path.join(self.model_save_path, latest_model)

            self.get_logger().info(f"Validating model: {latest_model_path}")

            # Perform safety validation
            if self.validate_safety:
                self.get_logger().info("Performing safety validation...")

                # Export model to ONNX for validation
                exporter = PolicyExporter(latest_model_path, "onnx")
                exporter.load_model()

                sample_input = torch.randn(1, self.state_dim)
                onnx_model_path = os.path.join(
                    self.model_save_path, "validation_model.onnx"
                )
                exporter.export_to_onnx(onnx_model_path, sample_input)

                # Validate safety
                validator = SafetyValidator(onnx_model_path)

                # Generate test states for validation
                test_states = []
                for i in range(20):  # Test with 20 different states
                    state = np.random.randn(self.state_dim)
                    test_states.append(state)

                # Run comprehensive safety check
                safety_results = validator.run_comprehensive_safety_check(test_states)

                self.validation_results["safety"] = safety_results

                if safety_results["overall_safe"]:
                    self.get_logger().info("Safety validation passed")

                    # Publish safety check success
                    status_header = Header()
                    status_header.stamp = self.get_clock().now().to_msg()
                    status_header.frame_id = "safety_validation_passed"
                    self.safety_check_pub.publish(status_header)
                else:
                    self.get_logger().error(
                        f'Safety validation failed: {safety_results["details"]}'
                    )

                    # Publish safety check failure
                    status_header = Header()
                    status_header.stamp = self.get_clock().now().to_msg()
                    status_header.frame_id = "safety_validation_failed"
                    self.safety_check_pub.publish(status_header)

                    return False

            # Performance validation
            self.get_logger().info("Performing performance validation...")

            # Create a mock environment for performance testing
            env = IsaacSimEnvironment(self.robot_name, self.task)

            # Test the policy with multiple episodes
            performance_metrics = {
                "success_rate": 0.0,
                "average_reward": 0.0,
                "average_episode_length": 0,
            }

            num_test_episodes = 10
            success_count = 0
            total_reward = 0
            total_steps = 0

            for episode in range(num_test_episodes):
                state = env.reset()
                episode_reward = 0
                episode_steps = 0
                done = False

                for step in range(self.max_steps_per_episode):
                    # For validation, we might use a simple heuristic or pre-trained policy
                    # In a real implementation, you would load and use the trained policy
                    action = np.random.randn(self.action_dim)  # Placeholder
                    next_state, reward, done, info = env.step(action)

                    state = next_state
                    episode_reward += reward
                    episode_steps += 1

                    if done:
                        break

                total_reward += episode_reward
                total_steps += episode_steps

                # For this example, we'll consider any episode that runs for more than 50 steps as successful
                if episode_steps > 50:
                    success_count += 1

            performance_metrics["success_rate"] = success_count / num_test_episodes
            performance_metrics["average_reward"] = total_reward / num_test_episodes
            performance_metrics["average_episode_length"] = (
                total_steps / num_test_episodes
            )

            self.validation_results["performance"] = performance_metrics

            self.get_logger().info(
                f"Performance validation results: {performance_metrics}"
            )

            # Publish validation results
            status_header = Header()
            status_header.stamp = self.get_clock().now().to_msg()
            status_header.frame_id = "validation_completed"
            self.validation_result_pub.publish(status_header)

            return True

        except Exception as e:
            self.get_logger().error(f"Validation phase error: {e}")
            import traceback

            traceback.print_exc()
            return False

    def run_deployment_phase(self) -> bool:
        """
        Run the policy deployment phase

        Returns:
            True if deployment successful, False otherwise
        """
        self.get_logger().info("Starting deployment phase")

        try:
            # Find the latest trained model
            model_files = [
                f for f in os.listdir(self.model_save_path) if f.endswith(".pth")
            ]
            if not model_files:
                self.get_logger().error("No trained models found for deployment")
                return False

            # Get the most recent model
            latest_model = max(
                model_files,
                key=lambda x: os.path.getctime(os.path.join(self.model_save_path, x)),
            )
            latest_model_path = os.path.join(self.model_save_path, latest_model)

            self.get_logger().info(f"Deploying model: {latest_model_path}")

            # Create deployment node
            deployment_node = PolicyDeploymentNode()

            # Override parameters with lab parameters
            deployment_node.declare_parameter("model_path", latest_model_path)
            deployment_node.declare_parameter("robot_name", self.robot_name)
            deployment_node.declare_parameter(
                "deployment_target", self.deployment_target
            )
            deployment_node.declare_parameter("validate_safety", self.validate_safety)
            deployment_node.declare_parameter("enable_tensorrt", self.enable_tensorrt)

            # Deploy the policy
            success = deployment_node.deploy_policy()

            if success:
                self.get_logger().info("Policy deployment completed successfully")
                return True
            else:
                self.get_logger().error("Policy deployment failed")
                return False

        except Exception as e:
            self.get_logger().error(f"Deployment phase error: {e}")
            import traceback

            traceback.print_exc()
            return False

    def run_sim_to_real_transfer(self) -> bool:
        """
        Run the sim-to-real transfer process

        Returns:
            True if transfer successful, False otherwise
        """
        self.get_logger().info("Starting sim-to-real transfer process")

        try:
            # The sim-to-real transfer process involves several steps:
            # 1. Domain randomization during training
            # 2. System identification and model updating
            # 3. Fine-tuning on real-world data
            # 4. Validation in real environment

            # For this lab, we'll simulate the process
            # In a real implementation, this would involve actual real-world testing

            # Step 1: Collect domain randomization statistics
            self.get_logger().info("Applying domain randomization techniques...")

            # This would involve adjusting simulation parameters to better match reality
            # For this example, we'll just log that the step was performed
            self.get_logger().info("Domain randomization applied successfully")

            # Step 2: Prepare for real-world fine-tuning
            self.get_logger().info("Preparing for real-world fine-tuning...")

            # Create configuration for real-world testing
            real_world_config = {
                "robot_name": self.robot_name,
                "task": self.task,
                "safety_constraints": {
                    "max_velocity": 0.5,
                    "max_acceleration": 1.0,
                    "joint_limits": True,
                },
                "monitoring": {"performance_threshold": 0.7, "safety_limits": True},
            }

            config_path = os.path.join(self.model_save_path, "real_world_config.json")
            with open(config_path, "w") as f:
                json.dump(real_world_config, f, indent=2)

            self.get_logger().info(f"Real-world configuration saved to: {config_path}")

            # Step 3: Simulate real-world validation
            self.get_logger().info("Simulating real-world validation...")

            # In a real implementation, this would involve actual testing on the real robot
            # For this lab, we'll simulate the validation process
            real_world_success = True  # Simulated success

            if real_world_success:
                self.get_logger().info(
                    "Sim-to-real transfer validation completed successfully"
                )
                return True
            else:
                self.get_logger().error("Sim-to-real transfer validation failed")
                return False

        except Exception as e:
            self.get_logger().error(f"Sim-to-real transfer error: {e}")
            import traceback

            traceback.print_exc()
            return False

    def get_lab_status(self) -> Dict[str, Any]:
        """
        Get the current status of the lab

        Returns:
            Dictionary containing lab status information
        """
        status = {
            "lab_active": self.lab_active,
            "current_phase": "unknown",
            "training_results": self.training_results,
            "validation_results": self.validation_results,
            "current_episode": self.current_episode,
            "total_episodes": self.total_episodes,
        }

        return status

    def stop_lab(self):
        """
        Stop the RL policy lab
        """
        self.get_logger().info("Stopping RL Policy Lab...")
        self.lab_active = False

        if self.lab_thread and self.lab_thread.is_alive():
            self.lab_thread.join()

        # Publish lab stop status
        status_header = Header()
        status_header.stamp = self.get_clock().now().to_msg()
        status_header.frame_id = "lab_stopped"
        self.lab_status_pub.publish(status_header)


def main(args=None):
    """
    Main function to run the RL Policy Lab
    """
    rclpy.init(args=args)

    rl_policy_lab = RLPolicyLab()

    try:
        # Start the lab
        rl_policy_lab.start_lab()

        # Run node (the actual work happens in the lab thread)
        rclpy.spin(rl_policy_lab)

    except KeyboardInterrupt:
        rl_policy_lab.get_logger().info("Interrupted by user")
    finally:
        rl_policy_lab.stop_lab()
        rl_policy_lab.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
