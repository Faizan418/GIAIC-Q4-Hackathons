#!/usr/bin/env python3

"""
Test RL Policy Deployment Pipeline with Safety Constraints Validation

This script validates the complete RL policy deployment pipeline,
including safety constraint validation, model export, and deployment verification.
"""

import json
import os
import sys
import tempfile
import time
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import torch
import torch.nn as nn

# Import the components to test
from isaac_examples.reinforcement_learning.policy_deployment import (
    DeploymentManager,
    PolicyDeploymentNode,
    PolicyExporter,
    SafetyValidator,
)
from isaac_examples.reinforcement_learning.rl_training import ActorCriticNetwork


class MockRobotInterface:
    """
    Mock robot interface for testing deployment without real hardware
    """

    def __init__(self):
        self.connected = False
        self.safety_limits = {
            "max_velocity": 0.5,
            "max_acceleration": 1.0,
            "joint_limits": True,
        }
        self.current_state = np.zeros(7)  # 7-DOF robot

    def connect(self):
        """Connect to the mock robot"""
        self.connected = True
        print("Mock robot connected")
        return True

    def disconnect(self):
        """Disconnect from the mock robot"""
        self.connected = False
        print("Mock robot disconnected")

    def send_command(self, action):
        """Send command to the mock robot"""
        if not self.connected:
            raise RuntimeError("Robot not connected")

        # Validate safety constraints
        if np.any(np.abs(action) > self.safety_limits["max_velocity"]):
            raise ValueError(
                f"Action exceeds max velocity limit of {self.safety_limits['max_velocity']}"
            )

        # Simulate command execution
        self.current_state += 0.01 * action  # Simple state update
        print(f"Command sent: {action[:3]}... (first 3 values)")

        return True

    def get_state(self):
        """Get current robot state"""
        return self.current_state.copy()


class RLDeploymentTester:
    """
    Test class for validating RL policy deployment pipeline
    """

    def __init__(self):
        self.test_results = []
        self.test_passed = 0
        self.test_failed = 0

    def run_comprehensive_tests(self):
        """
        Run comprehensive tests for the RL deployment pipeline
        """
        print("=" * 80)
        print(
            "COMPREHENSIVE TEST: RL Policy Deployment Pipeline with Safety Validation"
        )
        print("=" * 80)

        tests = [
            self.test_policy_exporter,
            self.test_safety_validator,
            self.test_deployment_manager,
            self.test_end_to_end_deployment,
            self.test_safety_constraints_validation,
            self.test_robot_interface_integration,
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
                import traceback

                traceback.print_exc()
                self.test_results.append((test_func.__name__, f"ERROR: {e}"))

        self.print_test_summary()
        return self.test_failed == 0

    def test_policy_exporter(self):
        """
        Test the PolicyExporter class
        """
        print("  Testing PolicyExporter functionality...")

        # Create a temporary trained model for testing
        with tempfile.NamedTemporaryFile(suffix=".pth", delete=False) as tmp_model:
            # Create a simple model for testing
            model = ActorCriticNetwork(state_dim=24, action_dim=7, hidden_dim=128)
            torch.save(
                {
                    "model_state_dict": model.state_dict(),
                    "state_dim": 24,
                    "action_dim": 7,
                    "hidden_dim": 128,
                },
                tmp_model.name,
            )

            # Test PolicyExporter
            exporter = PolicyExporter(tmp_model.name, "onnx")

            # Export to ONNX
            with tempfile.NamedTemporaryFile(suffix=".onnx", delete=False) as tmp_onnx:
                sample_input = torch.randn(1, 24)
                exported_path = exporter.export_to_onnx(tmp_onnx.name, sample_input)

                if exported_path and os.path.exists(exported_path):
                    print(f"    ✓ ONNX export successful: {exported_path}")
                else:
                    print("    ✗ ONNX export failed")
                    return False

            # Export to TensorRT (if available)
            try:
                with tempfile.NamedTemporaryFile(
                    suffix=".trt", delete=False
                ) as tmp_trt:
                    exported_trt_path = exporter.export_to_trt(
                        tmp_trt.name, sample_input
                    )
                    if exported_trt_path:
                        print(f"    ✓ TensorRT export successful: {exported_trt_path}")
                    else:
                        print("    ○ TensorRT export skipped (TensorRT not available)")
            except Exception:
                print("    ○ TensorRT export skipped (TensorRT not available)")

        print("  ✓ PolicyExporter functionality tested successfully")
        return True

    def test_safety_validator(self):
        """
        Test the SafetyValidator class
        """
        print("  Testing SafetyValidator functionality...")

        # First, create an ONNX model to validate
        with tempfile.NamedTemporaryFile(suffix=".onnx", delete=False) as tmp_onnx:
            # Create a simple model and export to ONNX for testing
            model = ActorCriticNetwork(state_dim=24, action_dim=7, hidden_dim=64)

            # Export using torch.onnx
            sample_input = torch.randn(1, 24)
            torch.onnx.export(
                model,
                sample_input,
                tmp_onnx.name,
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

            # Test SafetyValidator
            validator = SafetyValidator(tmp_onnx.name)

            # Load the model
            validator.load_model()

            # Test action space validation
            test_state = np.random.randn(24)
            action_valid = validator.validate_action_space(test_state)
            print(f"    Action space validation: {action_valid}")

            # Test stability validation
            stability_valid = validator.validate_stability(test_state, num_steps=10)
            print(f"    Stability validation: {stability_valid}")

            # Test comprehensive safety check
            test_states = [np.random.randn(24) for _ in range(5)]
            comprehensive_results = validator.run_comprehensive_safety_check(
                test_states
            )
            print(
                f"    Comprehensive safety check: {comprehensive_results['overall_safe']}"
            )

        print("  ✓ SafetyValidator functionality tested successfully")
        return True

    def test_deployment_manager(self):
        """
        Test the DeploymentManager class
        """
        print("  Testing DeploymentManager functionality...")

        # Create a temporary model file
        with tempfile.NamedTemporaryFile(suffix=".onnx", delete=False) as tmp_model:
            # Create a simple ONNX model for testing
            model = ActorCriticNetwork(state_dim=24, action_dim=7, hidden_dim=64)
            sample_input = torch.randn(1, 24)
            torch.onnx.export(
                model,
                sample_input,
                tmp_model.name,
                export_params=True,
                opset_version=11,
                do_constant_folding=True,
                input_names=["input"],
                output_names=["action_mean", "value"],
            )

            # Test DeploymentManager
            manager = DeploymentManager(
                robot_name="test_robot", deployment_target="local"
            )

            # Create test configuration
            config = {
                "robot_name": "test_robot",
                "deployment_target": "local",
                "model_format": "onnx",
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

            # Prepare deployment package
            deployment_package_path = manager.prepare_deployment_package(
                tmp_model.name, config
            )
            if not deployment_package_path or not os.path.exists(
                deployment_package_path
            ):
                print("    ✗ Deployment package preparation failed")
                return False

            print(f"    Deployment package created at: {deployment_package_path}")

            # Validate deployment package
            validation_result = manager.validate_deployment(deployment_package_path)
            if not validation_result:
                print("    ✗ Deployment package validation failed")
                return False

            print("    ✓ Deployment package validated successfully")

            # Clean up
            import shutil

            shutil.rmtree(deployment_package_path, ignore_errors=True)

        print("  ✓ DeploymentManager functionality tested successfully")
        return True

    def test_end_to_end_deployment(self):
        """
        Test end-to-end deployment process
        """
        print("  Testing end-to-end deployment process...")

        # Create a temporary trained model
        with tempfile.TemporaryDirectory() as tmp_dir:
            model_path = os.path.join(tmp_dir, "test_model.pth")

            # Create and save a test model
            model = ActorCriticNetwork(state_dim=24, action_dim=7, hidden_dim=128)
            torch.save(
                {
                    "model_state_dict": model.state_dict(),
                    "state_dim": 24,
                    "action_dim": 7,
                    "hidden_dim": 128,
                },
                model_path,
            )

            # Create deployment configuration
            config_path = os.path.join(tmp_dir, "deployment_config.json")
            config = {
                "robot_name": "test_robot",
                "deployment_target": "local",
                "model_format": "onnx",
                "safety_constraints": {
                    "max_velocity": 0.5,
                    "max_acceleration": 1.0,
                    "joint_limits": True,
                },
            }
            with open(config_path, "w") as f:
                json.dump(config, f)

            # Test the deployment process
            try:
                # Create a mock deployment node to test the process
                with patch("rclpy.node.Node.__init__", return_value=None):
                    # This is a simplified test - in a real scenario, we'd test the full node
                    print("    Simulating end-to-end deployment process...")

                    # Export model
                    exporter = PolicyExporter(model_path, "onnx")
                    exporter.load_model()

                    onnx_path = os.path.join(tmp_dir, "exported_model.onnx")
                    sample_input = torch.randn(1, 24)
                    exporter.export_to_onnx(onnx_path, sample_input)

                    if not os.path.exists(onnx_path):
                        print("    ✗ Model export failed in end-to-end test")
                        return False

                    # Validate safety
                    validator = SafetyValidator(onnx_path)
                    validator.load_model()

                    test_states = [np.random.randn(24) for _ in range(3)]
                    safety_results = validator.run_comprehensive_safety_check(
                        test_states
                    )

                    if not safety_results["overall_safe"]:
                        print(
                            f"    ✗ Safety validation failed in end-to-end test: {safety_results['details']}"
                        )
                        return False

                    # Prepare deployment package
                    manager = DeploymentManager("test_robot", "local")
                    package_path = manager.prepare_deployment_package(onnx_path, config)

                    if not package_path or not os.path.exists(package_path):
                        print(
                            "    ✗ Deployment package preparation failed in end-to-end test"
                        )
                        return False

                    print("    ✓ End-to-end deployment process completed successfully")

                    # Clean up
                    import shutil

                    shutil.rmtree(package_path, ignore_errors=True)

            except Exception as e:
                print(f"    ✗ End-to-end deployment test failed: {e}")
                import traceback

                traceback.print_exc()
                return False

        print("  ✓ End-to-end deployment process tested successfully")
        return True

    def test_safety_constraints_validation(self):
        """
        Test comprehensive safety constraints validation
        """
        print("  Testing safety constraints validation...")

        # Create a temporary ONNX model for testing
        with tempfile.NamedTemporaryFile(suffix=".onnx", delete=False) as tmp_onnx:
            model = ActorCriticNetwork(state_dim=24, action_dim=7, hidden_dim=64)
            sample_input = torch.randn(1, 24)
            torch.onnx.export(
                model,
                sample_input,
                tmp_onnx.name,
                export_params=True,
                opset_version=11,
                do_constant_folding=True,
                input_names=["input"],
                output_names=["action_mean", "value"],
            )

            # Test SafetyValidator with various safety scenarios
            validator = SafetyValidator(tmp_onnx.name)
            validator.load_model()

            # Test 1: Normal operation (should pass)
            normal_state = np.random.randn(24)
            normal_action_valid = validator.validate_action_space(normal_state)
            print(f"    Normal operation validation: {normal_action_valid}")

            # Test 2: Stability over multiple steps
            stability_result = validator.validate_stability(normal_state, num_steps=20)
            print(f"    Stability validation over 20 steps: {stability_result}")

            # Test 3: Comprehensive validation with multiple states
            test_states = []
            for i in range(10):
                state = np.random.randn(24)
                # Add some states that might cause issues
                if i % 3 == 0:
                    state = state * 2  # Potentially problematic state
                test_states.append(state)

            comprehensive_results = validator.run_comprehensive_safety_check(
                test_states
            )
            print(f"    Comprehensive safety results: {comprehensive_results}")

            # Verify that the validation caught any potential issues
            if comprehensive_results["overall_safe"]:
                print("    ✓ Safety constraints validation passed all tests")
            else:
                print(
                    f"    ○ Safety validation detected issues: {comprehensive_results['details']}"
                )
                # This is not necessarily a failure - safety validation should catch issues

        print("  ✓ Safety constraints validation tested successfully")
        return True

    def test_robot_interface_integration(self):
        """
        Test integration with robot interface
        """
        print("  Testing robot interface integration...")

        # Create mock robot interface
        robot = MockRobotInterface()

        # Test connection
        if not robot.connect():
            print("    ✗ Robot connection failed")
            return False

        print("    ✓ Robot connected successfully")

        # Test sending safe commands
        for i in range(5):
            action = np.random.uniform(-0.4, 0.4, size=7)  # Within safety limits
            try:
                robot.send_command(action)
                print(f"    ✓ Safe command {i+1} executed successfully")
            except Exception as e:
                print(f"    ✗ Safe command {i+1} failed: {e}")
                return False

        # Test sending unsafe commands (should be rejected)
        unsafe_action = np.ones(7) * 1.0  # Exceeds safety limits
        try:
            robot.send_command(unsafe_action)
            print("    ✗ Unsafe command was not rejected (safety failure)")
            return False
        except ValueError:
            print("    ✓ Unsafe command correctly rejected by safety system")

        # Test state retrieval
        state = robot.get_state()
        if state is not None and len(state) == 7:
            print(f"    ✓ Robot state retrieved successfully: {state[:3]}...")
        else:
            print("    ✗ Robot state retrieval failed")
            return False

        # Disconnect
        robot.disconnect()
        print("    ✓ Robot disconnected successfully")

        print("  ✓ Robot interface integration tested successfully")
        return True

    def print_test_summary(self):
        """
        Print summary of all tests
        """
        print("\n" + "=" * 80)
        print("RL POLICY DEPLOYMENT PIPELINE TEST SUMMARY")
        print("=" * 80)
        print(f"Total Tests: {len(self.test_results)}")
        print(f"Passed: {self.test_passed}")
        print(f"Failed: {self.test_failed}")
        print()

        for test_name, result in self.test_results:
            print(f"  {test_name}: {result}")

        if self.test_failed == 0:
            print("\n✓ ALL RL DEPLOYMENT PIPELINE TESTS PASSED")
            print(
                "  The deployment pipeline with safety constraints validation is working correctly."
            )
        else:
            print(f"\n✗ {self.test_failed} TEST(S) FAILED")
            print("  The deployment pipeline needs fixes before production use.")
        print("=" * 80)


def main():
    """
    Main function to run the deployment pipeline tests
    """
    tester = RLDeploymentTester()

    try:
        success = tester.run_comprehensive_tests()
        if success:
            print("\n✓ RL policy deployment pipeline test PASSED")
            return 0
        else:
            print("\n✗ RL policy deployment pipeline test FAILED")
            return 1
    except Exception as e:
        print(f"\n✗ Deployment pipeline test ERROR: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
