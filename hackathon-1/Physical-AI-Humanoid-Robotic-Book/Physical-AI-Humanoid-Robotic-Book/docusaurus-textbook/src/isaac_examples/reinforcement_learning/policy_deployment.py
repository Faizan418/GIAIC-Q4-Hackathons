#!/usr/bin/env python3

"""
Policy Export and Deployment Pipeline

This module implements the complete pipeline for exporting trained reinforcement learning
policies from simulation to real-world deployment. It includes model conversion,
optimization, safety validation, and deployment to real robots.
"""

import json
import os
import shutil
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
import onnx
import onnxruntime as ort
import rclpy
import tensorflow as tf
import torch
import torch.nn as nn
import torch.onnx
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Twist

# Import common utilities
from isaac_examples.common.isaac_ros_utils import (
    create_point,
    create_pose,
    get_transform,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Header, String
from tensorflow.python.framework.convert_to_constants import (
    convert_variables_to_constants_v2,
)


class PolicyExporter:
    """
    Class for exporting trained RL policies to various formats
    """

    def __init__(self, model_path: str, export_format: str = "onnx"):
        self.model_path = model_path
        self.export_format = export_format
        self.model = None
        self.input_shape = None

    def load_model(self, model_path: str = None) -> torch.nn.Module:
        """
        Load the trained model from checkpoint

        Args:
            model_path: Path to the model checkpoint (optional, uses self.model_path if not provided)

        Returns:
            Loaded PyTorch model
        """
        if model_path is None:
            model_path = self.model_path

        # Load checkpoint
        checkpoint = torch.load(model_path, map_location=torch.device("cpu"))

        # Assuming the model architecture is ActorCriticNetwork from rl_training.py
        # In a real implementation, you would need to reconstruct the model based on saved architecture
        if "model_state_dict" in checkpoint:
            # For now, we'll create a generic model structure
            # In practice, you would load the actual architecture
            self.model = self._create_model_from_checkpoint(checkpoint)
            self.model.load_state_dict(checkpoint["model_state_dict"])
        else:
            # If it's a direct model save
            self.model = torch.load(model_path, map_location=torch.device("cpu"))

        self.model.eval()  # Set to evaluation mode
        return self.model

    def _create_model_from_checkpoint(self, checkpoint: Dict) -> torch.nn.Module:
        """
        Create model architecture from checkpoint metadata

        Args:
            checkpoint: Model checkpoint dictionary

        Returns:
            PyTorch model with correct architecture
        """
        # This is a simplified version - in practice, you'd have more robust architecture reconstruction
        # For now, we'll create a generic Actor-Critic network
        # In a real implementation, you'd store the model architecture in the checkpoint
        state_dim = checkpoint.get("state_dim", 24)  # Default from rl_training.py
        action_dim = checkpoint.get("action_dim", 7)  # Default from rl_training.py
        hidden_dim = checkpoint.get("hidden_dim", 256)

        # Create a simple Actor-Critic network
        class SimpleActorCritic(nn.Module):
            def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 256):
                super().__init__()
                self.shared_layers = nn.Sequential(
                    nn.Linear(state_dim, hidden_dim),
                    nn.ReLU(),
                    nn.Linear(hidden_dim, hidden_dim),
                    nn.ReLU(),
                )
                self.actor_mean = nn.Linear(hidden_dim, action_dim)
                self.critic = nn.Linear(hidden_dim, 1)

            def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
                shared_out = self.shared_layers(state)
                action_mean = torch.tanh(self.actor_mean(shared_out))
                value = self.critic(shared_out)
                return action_mean, value

        return SimpleActorCritic(state_dim, action_dim, hidden_dim)

    def export_to_onnx(self, output_path: str, input_sample: torch.Tensor) -> str:
        """
        Export model to ONNX format

        Args:
            output_path: Path to save the ONNX model
            input_sample: Sample input tensor for tracing

        Returns:
            Path to the exported ONNX model
        """
        if self.model is None:
            self.load_model()

        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        # Export to ONNX
        torch.onnx.export(
            self.model,
            input_sample,
            output_path,
            export_params=True,
            opset_version=11,
            do_constant_folding=True,
            input_names=["input"],
            output_names=["action_mean", "value"],
            dynamic_axes={
                "input": {0: "batch_size"},
                "action_mean": {0: "batch_size"},
                "value": {0: "batch_size"},
            },
        )

        print(f"Model exported to ONNX format: {output_path}")
        return output_path

    def export_to_tensorflow(self, output_path: str) -> str:
        """
        Export model to TensorFlow format

        Args:
            output_path: Path to save the TensorFlow model

        Returns:
            Path to the exported TensorFlow model
        """
        if self.model is None:
            self.load_model()

        # Convert PyTorch model to TensorFlow (using ONNX as intermediate)
        with tempfile.NamedTemporaryFile(suffix=".onnx", delete=False) as tmp_onnx:
            # First export to ONNX
            self.export_to_onnx(tmp_onnx.name, torch.randn(1, self.model.state_dim))

            # Then convert ONNX to TensorFlow
            import tf2onnx

            # Convert ONNX to TensorFlow
            onnx_model = onnx.load(tmp_onnx.name)
            tf_rep = tf2onnx.convert.from_onnx(onnx_model)
            tf_rep.export_graph(output_path)

            # Clean up temporary file
            os.unlink(tmp_onnx.name)

        print(f"Model exported to TensorFlow format: {output_path}")
        return output_path

    def export_to_trt(self, output_path: str, input_sample: torch.Tensor) -> str:
        """
        Export model to TensorRT format for NVIDIA GPU optimization

        Args:
            output_path: Path to save the TensorRT model
            input_sample: Sample input tensor for optimization

        Returns:
            Path to the exported TensorRT model
        """
        # First export to ONNX
        with tempfile.NamedTemporaryFile(suffix=".onnx", delete=False) as tmp_onnx:
            self.export_to_onnx(tmp_onnx.name, input_sample)

            # Convert ONNX to TensorRT using NVIDIA's tools
            try:
                import tensorrt as trt
                from polygraphy.backend.trt import (
                    create_network,
                    engine_from_network,
                    save_engine,
                )

                # Load ONNX model
                onnx_model = onnx.load(tmp_onnx.name)

                # Create TensorRT network
                builder, network, parser = create_network()
                parser.parse_onnx_model(onnx_model)

                # Create TensorRT engine
                engine = engine_from_network((builder, network))

                # Save TensorRT engine
                save_engine(engine, output_path)

                print(f"Model exported to TensorRT format: {output_path}")
            except ImportError:
                print("TensorRT not available, skipping TRT export")
                return None
            finally:
                # Clean up temporary file
                os.unlink(tmp_onnx.name)

        return output_path


class SafetyValidator:
    """
    Class for validating policy safety before deployment
    """

    def __init__(self, model_path: str):
        self.model_path = model_path
        self.session = None

    def load_model(self, model_path: str = None) -> Any:
        """
        Load the model for validation (using ONNX Runtime for cross-platform compatibility)

        Args:
            model_path: Path to the model file (ONNX format)

        Returns:
            ONNX Runtime inference session
        """
        if model_path is None:
            model_path = self.model_path

        self.session = ort.InferenceSession(model_path)
        return self.session

    def validate_action_space(self, state: np.ndarray) -> bool:
        """
        Validate that the model outputs actions within safe bounds

        Args:
            state: Input state to the model

        Returns:
            True if action is within safe bounds, False otherwise
        """
        if self.session is None:
            self.load_model()

        # Get model output
        input_name = self.session.get_inputs()[0].name
        output = self.session.run(None, {input_name: state.astype(np.float32)})

        action_mean = output[0][0]  # Assuming first output is action mean

        # Define safe action bounds (example: joint limits, velocity limits)
        action_bounds = {
            "min": np.array([-1.0] * len(action_mean)),  # Example bounds
            "max": np.array([1.0] * len(action_mean)),
        }

        # Check if action is within bounds
        if np.any(action_mean < action_bounds["min"]) or np.any(
            action_mean > action_bounds["max"]
        ):
            print(f"Action out of bounds: {action_mean}")
            return False

        return True

    def validate_stability(
        self, initial_state: np.ndarray, num_steps: int = 100
    ) -> bool:
        """
        Validate policy stability over multiple steps

        Args:
            initial_state: Starting state for validation
            num_steps: Number of simulation steps to validate

        Returns:
            True if policy is stable, False otherwise
        """
        if self.session is None:
            self.load_model()

        state = initial_state.copy()
        input_name = self.session.get_inputs()[0].name

        for step in range(num_steps):
            # Get action from model
            output = self.session.run(None, {input_name: state.astype(np.float32)})
            action = output[0][0]

            # Validate action bounds
            if not self.validate_action_space(state):
                print(f"Unstable action at step {step}")
                return False

            # Simulate state transition (simplified)
            # In a real implementation, you'd have a proper state transition model
            state = state + 0.01 * action  # Simplified state transition

        return True

    def run_comprehensive_safety_check(
        self, test_states: List[np.ndarray]
    ) -> Dict[str, Any]:
        """
        Run comprehensive safety validation on multiple test states

        Args:
            test_states: List of test states to validate

        Returns:
            Dictionary with validation results
        """
        results = {
            "action_space_valid": True,
            "stability_valid": True,
            "overall_safe": True,
            "failed_states": [],
            "details": [],
        }

        for i, state in enumerate(test_states):
            # Validate action space
            action_valid = self.validate_action_space(state)
            if not action_valid:
                results["action_space_valid"] = False
                results["failed_states"].append(i)
                results["details"].append(f"State {i}: Action space violation")

            # Validate stability for this state
            stability_valid = self.validate_stability(state, num_steps=50)
            if not stability_valid:
                results["stability_valid"] = False
                results["failed_states"].append(i)
                results["details"].append(f"State {i}: Stability violation")

        results["overall_safe"] = (
            results["action_space_valid"] and results["stability_valid"]
        )
        return results


class DeploymentManager:
    """
    Class for managing policy deployment to real robots
    """

    def __init__(self, robot_name: str = "franka", deployment_target: str = "local"):
        self.robot_name = robot_name
        self.deployment_target = deployment_target
        self.deployment_path = f"/opt/ros/robot_models/{robot_name}/policies"
        self.model_config = {}

    def prepare_deployment_package(
        self, model_path: str, config: Dict[str, Any]
    ) -> str:
        """
        Prepare a deployment package with model and configuration

        Args:
            model_path: Path to the trained model
            config: Configuration dictionary for the deployment

        Returns:
            Path to the deployment package
        """
        # Create deployment directory
        deployment_dir = os.path.join(
            self.deployment_path, f"policy_{int(time.time())}"
        )
        os.makedirs(deployment_dir, exist_ok=True)

        # Copy model to deployment directory
        model_filename = os.path.basename(model_path)
        deployment_model_path = os.path.join(deployment_dir, model_filename)
        shutil.copy2(model_path, deployment_model_path)

        # Save configuration
        config_path = os.path.join(deployment_dir, "config.json")
        with open(config_path, "w") as f:
            json.dump(config, f, indent=2)

        # Create deployment manifest
        manifest = {
            "model_path": model_filename,
            "config_path": "config.json",
            "robot_name": self.robot_name,
            "deployment_timestamp": time.time(),
            "deployment_target": self.deployment_target,
        }
        manifest_path = os.path.join(deployment_dir, "manifest.json")
        with open(manifest_path, "w") as f:
            json.dump(manifest, f, indent=2)

        print(f"Deployment package prepared at: {deployment_dir}")
        return deployment_dir

    def deploy_to_robot(self, deployment_package_path: str) -> bool:
        """
        Deploy the policy to the target robot

        Args:
            deployment_package_path: Path to the deployment package

        Returns:
            True if deployment successful, False otherwise
        """
        try:
            if self.deployment_target == "local":
                # For local deployment, just copy files
                target_dir = f"/opt/robot_policies/{self.robot_name}"
                os.makedirs(target_dir, exist_ok=True)
                shutil.copytree(deployment_package_path, target_dir, dirs_exist_ok=True)
                print(f"Policy deployed to local robot at: {target_dir}")

            elif self.deployment_target.startswith("ssh://"):
                # For remote deployment via SSH
                remote_host = self.deployment_target[6:]  # Remove ssh:// prefix
                cmd = [
                    "scp",
                    "-r",
                    deployment_package_path,
                    f"{remote_host}:/opt/robot_policies/",
                ]
                result = subprocess.run(cmd, capture_output=True, text=True)
                if result.returncode != 0:
                    print(f"Deployment failed: {result.stderr}")
                    return False
                print(f"Policy deployed to remote robot: {remote_host}")

            else:
                print(f"Unknown deployment target: {self.deployment_target}")
                return False

            return True

        except Exception as e:
            print(f"Deployment error: {e}")
            return False

    def validate_deployment(self, deployment_package_path: str) -> bool:
        """
        Validate that the deployment was successful

        Args:
            deployment_package_path: Path to the deployment package

        Returns:
            True if validation successful, False otherwise
        """
        # Check if deployment files exist
        manifest_path = os.path.join(deployment_package_path, "manifest.json")
        if not os.path.exists(manifest_path):
            print("Deployment manifest not found")
            return False

        # Load manifest and verify contents
        with open(manifest_path, "r") as f:
            manifest = json.load(f)

        required_fields = ["model_path", "config_path", "robot_name"]
        for field in required_fields:
            if field not in manifest:
                print(f"Missing required field in manifest: {field}")
                return False

        # Verify model file exists
        model_path = os.path.join(deployment_package_path, manifest["model_path"])
        if not os.path.exists(model_path):
            print(f"Model file not found: {model_path}")
            return False

        # Verify config file exists
        config_path = os.path.join(deployment_package_path, manifest["config_path"])
        if not os.path.exists(config_path):
            print(f"Config file not found: {config_path}")
            return False

        print("Deployment validation passed")
        return True


class PolicyDeploymentNode(Node):
    """
    ROS 2 node for managing policy deployment
    """

    def __init__(self):
        super().__init__("policy_deployment_node")

        # Declare parameters
        self.declare_parameter("model_path", "/tmp/rl_models/rl_model_episode_100.pth")
        self.declare_parameter("export_format", "onnx")
        self.declare_parameter("robot_name", "franka")
        self.declare_parameter("deployment_target", "local")
        self.declare_parameter("validate_safety", True)
        self.declare_parameter("enable_tensorrt", True)
        self.declare_parameter("deployment_config_path", "/tmp/deployment_config.json")

        # Get parameters
        self.model_path = self.get_parameter("model_path").value
        self.export_format = self.get_parameter("export_format").value
        self.robot_name = self.get_parameter("robot_name").value
        self.deployment_target = self.get_parameter("deployment_target").value
        self.validate_safety = self.get_parameter("validate_safety").value
        self.enable_tensorrt = self.get_parameter("enable_tensorrt").value
        self.deployment_config_path = self.get_parameter("deployment_config_path").value

        # Publishers
        self.deployment_status_pub = self.create_publisher(
            Header, "deployment_status", 10
        )
        self.safety_validation_pub = self.create_publisher(
            Header, "safety_validation", 10
        )
        self.export_status_pub = self.create_publisher(Header, "export_status", 10)

        # Internal state
        self.deployment_thread = None
        self.deployment_active = False

        self.get_logger().info("Policy Deployment Node initialized")
        self.get_logger().info(f"Model path: {self.model_path}")
        self.get_logger().info(f"Deployment target: {self.deployment_target}")

    def deploy_policy(self):
        """
        Deploy the trained policy to the target robot
        """
        self.get_logger().info("Starting policy deployment process...")

        # Publish deployment start status
        status_header = Header()
        status_header.stamp = self.get_clock().now().to_msg()
        status_header.frame_id = "deployment_started"
        self.deployment_status_pub.publish(status_header)

        try:
            # Step 1: Export the model to the desired format
            exported_model_path = self.export_model()
            if not exported_model_path:
                self.get_logger().error("Model export failed")
                return False

            # Step 2: Validate safety if enabled
            if self.validate_safety:
                safety_valid = self.validate_policy_safety(exported_model_path)
                if not safety_valid:
                    self.get_logger().error("Safety validation failed")
                    return False

            # Step 3: Prepare deployment package
            config = self.load_deployment_config()
            deployment_package_path = self.prepare_deployment_package(
                exported_model_path, config
            )
            if not deployment_package_path:
                self.get_logger().error("Deployment package preparation failed")
                return False

            # Step 4: Validate deployment package
            if not self.validate_deployment_package(deployment_package_path):
                self.get_logger().error("Deployment package validation failed")
                return False

            # Step 5: Deploy to target
            deployment_success = self.deploy_to_target(deployment_package_path)
            if not deployment_success:
                self.get_logger().error("Deployment to target failed")
                return False

            self.get_logger().info("Policy deployment completed successfully")

            # Publish deployment completion status
            status_header = Header()
            status_header.stamp = self.get_clock().now().to_msg()
            status_header.frame_id = "deployment_completed"
            self.deployment_status_pub.publish(status_header)

            return True

        except Exception as e:
            self.get_logger().error(f"Deployment error: {e}")
            import traceback

            traceback.print_exc()

            # Publish deployment error status
            status_header = Header()
            status_header.stamp = self.get_clock().now().to_msg()
            status_header.frame_id = f"deployment_error_{str(e)}"
            self.deployment_status_pub.publish(status_header)

            return False

    def export_model(self) -> str:
        """
        Export the trained model to the specified format

        Returns:
            Path to the exported model, or None if export failed
        """
        self.get_logger().info(f"Exporting model to {self.export_format} format")

        # Create exporter
        exporter = PolicyExporter(self.model_path, self.export_format)

        # Determine export path
        base_name = os.path.splitext(os.path.basename(self.model_path))[0]
        export_path = f"/tmp/exported_models/{base_name}.{self.export_format}"

        try:
            # Load the model
            exporter.load_model()

            # Create sample input for tracing (this should match your actual input dimensions)
            sample_input = torch.randn(
                1, 24
            )  # Assuming 24-dim state from rl_training.py

            # Export based on format
            if self.export_format.lower() == "onnx":
                exported_path = exporter.export_to_onnx(export_path, sample_input)
            elif self.export_format.lower() == "tensorflow":
                exported_path = exporter.export_to_tensorflow(export_path)
            elif self.export_format.lower() == "trt" and self.enable_tensorrt:
                exported_path = exporter.export_to_trt(export_path, sample_input)
            else:
                # Default to ONNX
                exported_path = exporter.export_to_onnx(export_path, sample_input)

            # Publish export success status
            status_header = Header()
            status_header.stamp = self.get_clock().now().to_msg()
            status_header.frame_id = f"export_success_{self.export_format}"
            self.export_status_pub.publish(status_header)

            self.get_logger().info(f"Model exported successfully to: {exported_path}")
            return exported_path

        except Exception as e:
            self.get_logger().error(f"Model export failed: {e}")
            import traceback

            traceback.print_exc()

            # Publish export error status
            status_header = Header()
            status_header.stamp = self.get_clock().now().to_msg()
            status_header.frame_id = f"export_error_{str(e)}"
            self.export_status_pub.publish(status_header)

            return None

    def validate_policy_safety(self, model_path: str) -> bool:
        """
        Validate the safety of the policy before deployment

        Args:
            model_path: Path to the exported model

        Returns:
            True if policy is safe, False otherwise
        """
        self.get_logger().info("Validating policy safety...")

        try:
            # Create safety validator
            validator = SafetyValidator(model_path)

            # Generate test states for validation
            test_states = []
            for i in range(10):  # Test with 10 different states
                state = np.random.randn(24)  # Assuming 24-dim state
                test_states.append(state)

            # Run comprehensive safety check
            results = validator.run_comprehensive_safety_check(test_states)

            self.get_logger().info(f"Safety validation results: {results}")

            if results["overall_safe"]:
                self.get_logger().info("Policy safety validation passed")

                # Publish safety validation success
                status_header = Header()
                status_header.stamp = self.get_clock().now().to_msg()
                status_header.frame_id = "safety_validation_passed"
                self.safety_validation_pub.publish(status_header)

                return True
            else:
                self.get_logger().error(
                    f'Policy safety validation failed: {results["details"]}'
                )

                # Publish safety validation failure
                status_header = Header()
                status_header.stamp = self.get_clock().now().to_msg()
                status_header.frame_id = "safety_validation_failed"
                self.safety_validation_pub.publish(status_header)

                return False

        except Exception as e:
            self.get_logger().error(f"Safety validation error: {e}")
            import traceback

            traceback.print_exc()

            # Publish safety validation error
            status_header = Header()
            status_header.stamp = self.get_clock().now().to_msg()
            status_header.frame_id = f"safety_validation_error_{str(e)}"
            self.safety_validation_pub.publish(status_header)

            return False

    def load_deployment_config(self) -> Dict[str, Any]:
        """
        Load deployment configuration from file

        Returns:
            Configuration dictionary
        """
        if os.path.exists(self.deployment_config_path):
            with open(self.deployment_config_path, "r") as f:
                return json.load(f)
        else:
            # Return default configuration
            return {
                "robot_name": self.robot_name,
                "deployment_target": self.deployment_target,
                "model_format": self.export_format,
                "safety_constraints": {
                    "max_velocity": 0.5,
                    "max_acceleration": 1.0,
                    "joint_limits": True,
                },
                "execution_parameters": {
                    "control_frequency": 50,
                    "prediction_horizon": 10,
                },
            }

    def prepare_deployment_package(
        self, model_path: str, config: Dict[str, Any]
    ) -> str:
        """
        Prepare deployment package with model and configuration

        Args:
            model_path: Path to the exported model
            config: Configuration dictionary

        Returns:
            Path to the deployment package
        """
        self.get_logger().info("Preparing deployment package...")

        try:
            # Create deployment manager
            deployment_manager = DeploymentManager(
                self.robot_name, self.deployment_target
            )

            # Prepare deployment package
            deployment_package_path = deployment_manager.prepare_deployment_package(
                model_path, config
            )

            self.get_logger().info(
                f"Deployment package prepared at: {deployment_package_path}"
            )
            return deployment_package_path

        except Exception as e:
            self.get_logger().error(f"Deployment package preparation failed: {e}")
            import traceback

            traceback.print_exc()
            return None

    def validate_deployment_package(self, deployment_package_path: str) -> bool:
        """
        Validate the deployment package

        Args:
            deployment_package_path: Path to the deployment package

        Returns:
            True if validation passes, False otherwise
        """
        try:
            # Create deployment manager
            deployment_manager = DeploymentManager(
                self.robot_name, self.deployment_target
            )

            # Validate the deployment
            is_valid = deployment_manager.validate_deployment(deployment_package_path)

            if is_valid:
                self.get_logger().info("Deployment package validation passed")
            else:
                self.get_logger().error("Deployment package validation failed")

            return is_valid

        except Exception as e:
            self.get_logger().error(f"Deployment package validation error: {e}")
            import traceback

            traceback.print_exc()
            return False

    def deploy_to_target(self, deployment_package_path: str) -> bool:
        """
        Deploy the package to the target robot

        Args:
            deployment_package_path: Path to the deployment package

        Returns:
            True if deployment successful, False otherwise
        """
        self.get_logger().info(f"Deploying to target: {self.deployment_target}")

        try:
            # Create deployment manager
            deployment_manager = DeploymentManager(
                self.robot_name, self.deployment_target
            )

            # Deploy to target
            success = deployment_manager.deploy_to_robot(deployment_package_path)

            if success:
                self.get_logger().info("Deployment to target successful")
            else:
                self.get_logger().error("Deployment to target failed")

            return success

        except Exception as e:
            self.get_logger().error(f"Deployment to target error: {e}")
            import traceback

            traceback.print_exc()
            return False


def main(args=None):
    """
    Main function to run the Policy Deployment Node
    """
    rclpy.init(args=args)

    policy_deployment_node = PolicyDeploymentNode()

    try:
        # Deploy the policy
        success = policy_deployment_node.deploy_policy()

        if success:
            policy_deployment_node.get_logger().info(
                "Policy deployment completed successfully"
            )
        else:
            policy_deployment_node.get_logger().error("Policy deployment failed")

    except KeyboardInterrupt:
        policy_deployment_node.get_logger().info("Interrupted by user")
    finally:
        policy_deployment_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
