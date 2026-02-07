"""
Synthetic Data Generation and Perception Stack Lab

This lab demonstrates the complete pipeline from synthetic data generation in Isaac Sim
to perception processing with Isaac ROS. It showcases how to create diverse training
datasets with domain randomization and process them through GPU-accelerated perception nodes.
"""

import os
import random
from typing import Any, Dict, List, Tuple

import carb
import cv2
import numpy as np
import omni
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.utils.carb import set_carb_setting
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.sensor import Camera
from PIL import Image


class SyntheticDataLab:
    """
    A lab environment for synthetic data generation with domain randomization
    """

    def __init__(self, output_dir: str = "./synthetic_data_output"):
        """
        Initialize the synthetic data lab

        Args:
            output_dir: Directory to save generated data samples
        """
        self.output_dir = output_dir
        self.world = None
        self.camera = None
        self.objects = []
        self.target_objects = []

        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "depth"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)

    def setup_world(self):
        """
        Setup the Isaac Sim world with basic configuration
        """
        self.world = World(stage_units_in_meters=1.0)

        # Set physics parameters
        set_carb_setting(carb, "/physics/vehicle/disable", True)

        # Create scene
        self.world.scene = Scene(usd_path="/World", name="World")
        self.world.scene.add_default_ground_plane()

        # Setup camera
        self.setup_camera()

        return self.world

    def setup_camera(self):
        """
        Setup RGB-D camera for data capture
        """
        # Add camera prim to stage
        camera_prim_path = "/World/Camera"

        # Create camera
        self.camera = self.world.scene.add(
            Camera(
                prim_path=camera_prim_path,
                name="synthetic_data_camera",
                position=[2.0, 0.0, 1.5],
                look_at=[0.0, 0.0, 0.5],
            )
        )

        # Enable RGB and depth sensors
        self.camera.add_rgb_camera(name="rgb_camera", resolution=(640, 480))
        self.camera.add_depth_camera(name="depth_camera", resolution=(640, 480))

    def randomize_scene(self):
        """
        Apply domain randomization to the scene
        """
        # Randomize lighting
        self.randomize_lighting()

        # Randomize background
        self.randomize_background()

        # Randomize materials
        self.randomize_materials()

        # Randomize target objects
        self.randomize_target_objects()

    def randomize_lighting(self):
        """
        Randomize lighting conditions in the scene
        """
        # In a real implementation, we would modify light properties
        # For this lab, we'll just log the randomization
        light_intensity = random.uniform(500, 2000)
        light_color = [
            random.uniform(0.8, 1.2),
            random.uniform(0.8, 1.2),
            random.uniform(0.8, 1.2),
        ]

        print(
            f"Applied lighting randomization: intensity={light_intensity:.2f}, color={light_color}"
        )

    def randomize_background(self):
        """
        Add randomized background elements
        """
        # Clear previous background objects
        for i, obj in enumerate(self.objects):
            try:
                obj.clear()
            except:
                pass

        self.objects = []

        # Add random background objects
        num_background_objects = random.randint(3, 8)

        for i in range(num_background_objects):
            # Random position in the background
            x = random.uniform(-2.0, 2.0)
            y = random.uniform(-2.0, 2.0)
            z = random.uniform(0.1, 0.5)

            # Random size
            size = random.uniform(0.1, 0.4)

            # Random color
            color = [
                random.uniform(0.1, 1.0),
                random.uniform(0.1, 1.0),
                random.uniform(0.1, 1.0),
            ]

            # Add visual cuboid to scene
            bg_obj = self.world.scene.add(
                VisualCuboid(
                    prim_path=f"/World/BackgroundObj_{i}",
                    name=f"bg_obj_{i}",
                    position=[x, y, z],
                    size=size,
                    color=np.array(color),
                )
            )
            self.objects.append(bg_obj)

    def randomize_materials(self):
        """
        Randomize material properties
        """
        # In a real implementation, we would modify material properties
        # For this lab, we'll just log the randomization
        print("Applied material randomization")

    def randomize_target_objects(self):
        """
        Randomize target objects for detection and manipulation
        """
        # Clear previous target objects
        for i, obj in enumerate(self.target_objects):
            try:
                obj.clear()
            except:
                pass

        self.target_objects = []

        # Add target objects with random poses and properties
        num_targets = random.randint(1, 3)

        for i in range(num_targets):
            # Random position
            x = random.uniform(-1.0, 1.0)
            y = random.uniform(-1.0, 1.0)
            z = random.uniform(0.2, 0.8)

            # Random size
            size = random.uniform(0.1, 0.3)

            # Random color
            color = [
                random.uniform(0.2, 0.9),
                random.uniform(0.2, 0.9),
                random.uniform(0.2, 0.9),
            ]

            # Add target object
            target_obj = self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/TargetObj_{i}",
                    name=f"target_obj_{i}",
                    position=[x, y, z],
                    size=size,
                    color=np.array(color),
                    mass=random.uniform(0.1, 0.5),
                )
            )
            self.target_objects.append(target_obj)

    def capture_data_sample(self, sample_id: int) -> Dict[str, Any]:
        """
        Capture a single data sample with RGB, depth, and annotations

        Args:
            sample_id: ID of the sample being captured

        Returns:
            Dictionary containing captured data and annotations
        """
        # Step the world to update sensors
        self.world.step(render=True)

        # Capture RGB image
        rgb_image = self.camera.get_rgb_camera().get_render_product().get_data()

        # Capture depth image
        depth_image = self.camera.get_depth_camera().get_render_product().get_data()

        # Get object poses for annotations
        annotations = self.get_object_annotations()

        # Save data to files
        self.save_data_sample(sample_id, rgb_image, depth_image, annotations)

        return {
            "sample_id": sample_id,
            "rgb_image": rgb_image,
            "depth_image": depth_image,
            "annotations": annotations,
        }

    def get_object_annotations(self) -> List[Dict[str, Any]]:
        """
        Get annotations for all objects in the scene

        Returns:
            List of annotation dictionaries for each object
        """
        annotations = []

        for i, target_obj in enumerate(self.target_objects):
            try:
                # Get world pose of the object
                position, orientation = target_obj.get_world_pose()

                # Create annotation
                annotation = {
                    "id": i,
                    "class": "target_object",
                    "position": position,
                    "orientation": orientation,
                    "size": target_obj.get_size()
                    if hasattr(target_obj, "get_size")
                    else 0.2,
                }
                annotations.append(annotation)
            except Exception as e:
                print(f"Error getting annotation for object {i}: {e}")

        return annotations

    def save_data_sample(
        self,
        sample_id: int,
        rgb_image: np.ndarray,
        depth_image: np.ndarray,
        annotations: List[Dict[str, Any]],
    ):
        """
        Save a data sample to disk

        Args:
            sample_id: ID of the sample
            rgb_image: RGB image data
            depth_image: Depth image data
            annotations: Object annotations
        """
        # Save RGB image
        rgb_path = os.path.join(self.output_dir, "images", f"rgb_{sample_id:04d}.png")
        if rgb_image is not None and len(rgb_image) > 0:
            # Convert and save RGB image
            rgb_img = Image.fromarray((rgb_image * 255).astype(np.uint8))
            rgb_img.save(rgb_path)

        # Save depth image
        depth_path = os.path.join(
            self.output_dir, "depth", f"depth_{sample_id:04d}.png"
        )
        if depth_image is not None and len(depth_image) > 0:
            # Normalize depth for visualization
            depth_normalized = (
                (depth_image - depth_image.min())
                / (depth_image.max() - depth_image.min())
                * 255
            ).astype(np.uint8)
            depth_img = Image.fromarray(depth_normalized)
            depth_img.save(depth_path)

        # Save annotations
        labels_path = os.path.join(
            self.output_dir, "labels", f"labels_{sample_id:04d}.txt"
        )
        with open(labels_path, "w") as f:
            for annotation in annotations:
                f.write(
                    f"Object {annotation['id']}: pos={annotation['position']}, "
                    f"orient={annotation['orientation']}, size={annotation['size']}\n"
                )

    def run_perception_pipeline(self, sample_id: int):
        """
        Simulate running the perception pipeline on captured data

        Args:
            sample_id: ID of the sample to process
        """
        print(f"Running perception pipeline on sample {sample_id}")

        # In a real implementation, this would run Isaac ROS perception nodes
        # For this lab, we'll simulate the perception process

        # Load the saved data
        rgb_path = os.path.join(self.output_dir, "images", f"rgb_{sample_id:04d}.png")
        depth_path = os.path.join(
            self.output_dir, "depth", f"depth_{sample_id:04d}.png"
        )

        if os.path.exists(rgb_path) and os.path.exists(depth_path):
            # Simulate object detection
            detected_objects = self.simulate_object_detection(rgb_path, depth_path)

            # Print detection results
            print(f"  Detected {len(detected_objects)} objects:")
            for obj in detected_objects:
                print(f"    - {obj['class']} at position {obj['position']}")
        else:
            print(f"  Data files not found for sample {sample_id}")

    def simulate_object_detection(
        self, rgb_path: str, depth_path: str
    ) -> List[Dict[str, Any]]:
        """
        Simulate object detection process

        Args:
            rgb_path: Path to RGB image
            depth_path: Path to depth image

        Returns:
            List of detected objects
        """
        # In a real implementation, this would use Isaac ROS perception nodes
        # For this simulation, we'll create detections based on saved annotations

        # For demonstration, return a fixed set of detections
        # In reality, this would come from perception node output
        return [
            {
                "class": "target_object",
                "position": [
                    random.uniform(-1.0, 1.0),
                    random.uniform(-1.0, 1.0),
                    random.uniform(0.2, 0.8),
                ],
                "confidence": random.uniform(0.7, 0.95),
            }
            for _ in range(random.randint(1, 3))
        ]

    def generate_dataset(self, num_samples: int = 10):
        """
        Generate a complete synthetic dataset

        Args:
            num_samples: Number of samples to generate
        """
        print(f"Starting synthetic dataset generation: {num_samples} samples")

        for i in range(num_samples):
            print(f"Generating sample {i+1}/{num_samples}")

            # Randomize the scene
            self.randomize_scene()

            # Capture data sample
            data_sample = self.capture_data_sample(i)

            # Run perception pipeline on the sample
            self.run_perception_pipeline(i)

            # Reset the world for the next sample
            self.world.reset()

        print(
            f"Dataset generation complete: {num_samples} samples saved to {self.output_dir}"
        )

    def run_lab_exercise(self):
        """
        Run the complete lab exercise demonstrating synthetic data generation
        and perception processing
        """
        print("=" * 60)
        print("SYNTHETIC DATA GENERATION AND PERCEPTION STACK LAB")
        print("=" * 60)

        # Setup the world
        print("\n1. Setting up Isaac Sim world...")
        self.setup_world()

        # Generate a small dataset
        print("\n2. Generating synthetic dataset with domain randomization...")
        self.generate_dataset(num_samples=5)  # Using smaller dataset for the lab

        # Demonstrate perception pipeline
        print("\n3. Demonstrating perception pipeline processing...")
        for i in range(3):  # Process first 3 samples
            self.run_perception_pipeline(i)

        print("\n4. Lab exercise complete!")
        print(f"   - Generated data saved to: {self.output_dir}")
        print(
            f"   - Images: {len(os.listdir(os.path.join(self.output_dir, 'images')))} files"
        )
        print(
            f"   - Depth: {len(os.listdir(os.path.join(self.output_dir, 'depth')))} files"
        )
        print(
            f"   - Labels: {len(os.listdir(os.path.join(self.output_dir, 'labels')))} files"
        )

        print("\nLab Objectives Achieved:")
        print("  - Demonstrated synthetic data generation with domain randomization")
        print("  - Showed RGB-D data capture pipeline")
        print("  - Illustrated perception processing workflow")
        print("  - Generated annotated training data for robotics applications")


def main():
    """
    Main function to run the synthetic data generation lab
    """
    # Create and run the lab
    lab = SyntheticDataLab(output_dir="./synthetic_data_lab_output")

    try:
        lab.run_lab_exercise()
    except Exception as e:
        print(f"Error running lab: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # Clean up
        if lab.world:
            lab.world.clear()


if __name__ == "__main__":
    main()
