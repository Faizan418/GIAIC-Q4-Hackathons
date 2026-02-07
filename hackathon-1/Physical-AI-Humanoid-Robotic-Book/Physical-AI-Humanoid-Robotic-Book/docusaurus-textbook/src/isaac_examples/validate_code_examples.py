#!/usr/bin/env python3

"""
Code Example Validation Script

This script validates that all code examples in the Isaac examples directory
have correct syntax and follow proper structure. It checks for import issues,
syntax errors, and structural consistency.
"""

import ast
import os
import subprocess
import sys
import tempfile
from pathlib import Path


def validate_python_file(filepath):
    """
    Validate a Python file for syntax and basic structure

    Args:
        filepath: Path to the Python file to validate

    Returns:
        Tuple of (is_valid, error_message)
    """
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            content = f.read()

        # Parse the file to check for syntax errors
        ast.parse(content)

        # Check for common issues
        lines = content.split("\n")

        # Check if it has ROS 2 structure
        has_ros_imports = any("rclpy" in line for line in lines)
        has_node_class = any("Node" in line and "class" in line for line in lines)
        has_main_function = any("def main" in line for line in lines)
        has_ros_spin = any("rclpy.spin" in line for line in lines)

        # Report findings
        issues = []
        if has_ros_imports and not (has_node_class and has_main_function):
            issues.append(
                "File has ROS imports but missing proper Node class or main function"
            )

        if has_ros_spin and not has_main_function:
            issues.append("File has rclpy.spin but missing main function structure")

        if issues:
            return True, f"Potential structural issues: {'; '.join(issues)}"

        return True, None

    except SyntaxError as e:
        return False, f"Syntax error at line {e.lineno}: {e.msg}"
    except Exception as e:
        return False, f"Error reading file: {str(e)}"


def validate_all_examples():
    """
    Validate all Python examples in the isaac_examples directory
    """
    print("Validating all Isaac code examples...")
    print("=" * 60)

    # Define the directory to check
    examples_dir = Path(
        "/mnt/e/Hackathon 1/Physical-AI-Humanoid-Robotic-Book/src/isaac_examples"
    )

    if not examples_dir.exists():
        print(f"Directory does not exist: {examples_dir}")
        return False

    all_files = list(examples_dir.rglob("*.py"))
    results = []

    for file_path in all_files:
        print(f"Validating: {file_path}")

        is_valid, error_msg = validate_python_file(file_path)

        if is_valid:
            if error_msg:
                print(f"  ⚠️  Warning: {error_msg}")
            else:
                print(f"  ✓ Valid")
            results.append((file_path, True, error_msg))
        else:
            print(f"  ✗ Error: {error_msg}")
            results.append((file_path, False, error_msg))

    print("\n" + "=" * 60)
    print("VALIDATION SUMMARY")
    print("=" * 60)

    total_files = len(results)
    valid_files = sum(1 for _, is_valid, _ in results if is_valid)
    invalid_files = total_files - valid_files

    print(f"Total files checked: {total_files}")
    print(f"Valid files: {valid_files}")
    print(f"Files with issues: {invalid_files}")

    if invalid_files > 0:
        print("\nFiles with errors:")
        for file_path, is_valid, error_msg in results:
            if not is_valid:
                print(f"  - {file_path}: {error_msg}")

    print("\nFile validation completed.")

    # Check for the most critical files that should exist
    critical_files = [
        "isaac_sim/scene_config.py",
        "isaac_sim/robot_setup.py",
        "isaac_ros/perception_node.py",
        "isaac_ros/vision_pipeline.py",
        "manipulation/grasp_planning.py",
        "manipulation/arm_control.py",
        "manipulation/object_manipulation_lab.py",
        "reinforcement_learning/rl_training.py",
        "reinforcement_learning/policy_deployment.py",
        "reinforcement_learning/rl_lab.py",
    ]

    print(f"\nCritical files check:")
    for critical_file in critical_files:
        expected_path = examples_dir / critical_file
        exists = expected_path.exists()
        status = "✓" if exists else "✗"
        print(f"  {status} {critical_file}")

    all_critical_exist = all((examples_dir / f).exists() for f in critical_files)

    if all_critical_exist:
        print(f"\n✓ All critical files are present")
    else:
        print(f"\n⚠️  Some critical files are missing")

    # Overall result
    overall_success = invalid_files == 0 and all_critical_exist
    print(
        f"\nOverall validation result: {'✓ PASSED' if overall_success else '✗ FAILED'}"
    )

    return overall_success


def check_imports():
    """
    Check if required imports are available in a test environment
    """
    print("\nChecking required imports...")

    required_imports = [
        "rclpy",
        "sensor_msgs",
        "geometry_msgs",
        "std_msgs",
        "torch",
        "numpy",
        "cv2",
        "onnx",
    ]

    successful_imports = []
    failed_imports = []

    for imp in required_imports:
        try:
            if imp == "sensor_msgs":
                exec("from sensor_msgs.msg import Image, CameraInfo")
            elif imp == "geometry_msgs":
                exec("from geometry_msgs.msg import Pose, Point")
            elif imp == "std_msgs":
                exec("from std_msgs.msg import Header")
            elif imp == "torch":
                exec("import torch")
            elif imp == "cv2":
                exec("import cv2")
            elif imp == "onnx":
                exec("import onnx")
            else:
                exec(f"import {imp}")
            successful_imports.append(imp)
            print(f"  ✓ {imp}")
        except ImportError as e:
            failed_imports.append((imp, str(e)))
            print(f"  ✗ {imp}: {e}")

    print(
        f"\nImport check: {len(successful_imports)} successful, {len(failed_imports)} failed"
    )

    return len(failed_imports) == 0


def main():
    """
    Main function to run all validations
    """
    print("Isaac Examples Code Validation")
    print("=" * 60)

    # Validate all code examples
    code_validation_passed = validate_all_examples()

    # Check imports
    import_check_passed = check_imports()

    print("\n" + "=" * 60)
    print("FINAL VALIDATION RESULTS")
    print("=" * 60)
    print(
        f"Code syntax validation: {'✓ PASSED' if code_validation_passed else '✗ FAILED'}"
    )
    print(
        f"Import availability check: {'✓ PASSED' if import_check_passed else '✗ FAILED'}"
    )

    overall_result = code_validation_passed and import_check_passed

    print(
        f"\nOverall result: {'✓ ALL VALIDATIONS PASSED' if overall_result else '✗ SOME VALIDATIONS FAILED'}"
    )

    if overall_result:
        print(
            "\nThe code examples appear to be syntactically correct and ready for simulation."
        )
        print(
            "Note: This validation checks syntax and structure. Actual runtime behavior"
        )
        print("in the Isaac Sim environment should be tested separately.")
    else:
        print(
            "\nSome issues were detected. Please review the validation results above."
        )

    return 0 if overall_result else 1


if __name__ == "__main__":
    sys.exit(main())
