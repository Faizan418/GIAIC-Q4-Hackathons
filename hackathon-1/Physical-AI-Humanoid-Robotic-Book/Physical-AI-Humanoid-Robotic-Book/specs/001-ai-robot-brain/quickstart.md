# Quickstart Guide: Module 3 — AI Robot Brain

## Prerequisites

Before starting with the AI Robot Brain module, ensure you have:

- Ubuntu 22.04 LTS (recommended)
- NVIDIA GPU with CUDA support (RTX series recommended)
- ROS 2 Humble Hawksbill installed
- NVIDIA Isaac Sim installed
- Isaac ROS packages installed
- Docusaurus development environment
- Python 3.8+ with PyTorch and ONNX support
- OpenCV and related computer vision libraries

## Environment Setup

### 1. Install NVIDIA Isaac Sim
```bash
# Download and install Isaac Sim from NVIDIA Developer website
# Follow the installation guide for your specific platform
```

### 2. Set up Isaac ROS
```bash
# Clone Isaac ROS repositories
git clone https://github.com/NVIDIA-ISAAC-ROS
# Follow the setup instructions for your ROS 2 distribution
```

### 3. Install Additional Dependencies
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install onnx onnxruntime
pip3 install opencv-python
pip3 install numpy matplotlib
```

### 4. Verify Installation
```bash
# Test Isaac Sim
./isaac-sim/python.sh
# Test ROS 2 integration
ros2 run isaac_ros_test test_node
```

## Running the Examples

### Chapter 1: NVIDIA Isaac Platform
1. Navigate to the Isaac Sim examples directory
2. Run the robot setup example:
```bash
cd src/isaac_examples/isaac_sim
python robot_setup.py
```
3. Launch the perception node:
```bash
cd src/isaac_examples/isaac_ros
ros2 run perception_node
```
4. Run the synthetic data generation lab:
```bash
cd src/isaac_examples/isaac_sim
python synthetic_data_lab.py
```

### Chapter 2: Perception and Manipulation
1. Start the vision pipeline:
```bash
cd src/isaac_examples/isaac_ros
python vision_pipeline.py
```
2. Run the grasp planning example:
```bash
cd src/isaac_examples/manipulation
python grasp_planning.py
```
3. Execute the manipulation sequence:
```bash
cd src/isaac_examples/manipulation
python arm_control.py
```
4. Run the complete perception-manipulation integration lab:
```bash
cd src/isaac_examples/manipulation
python object_manipulation_lab.py
```

### Chapter 3: Reinforcement Learning and Sim-to-Real
1. Launch the RL training environment:
```bash
cd src/isaac_examples/reinforcement_learning
python rl_training.py
```
2. Export and deploy the trained policy:
```bash
cd src/isaac_examples/reinforcement_learning
python policy_deployment.py
```
3. Run the complete RL policy training and validation lab:
```bash
cd src/isaac_examples/reinforcement_learning
python rl_lab.py
```
4. Test the deployment pipeline with safety validation:
```bash
cd src/isaac_examples/reinforcement_learning
python test_deployment.py
```

## Testing and Validation

### Integration Tests
Run the complete perception-manipulation integration test:
```bash
cd src/isaac_examples/manipulation
python test_integration.py
```

### System Validation
Validate the complete AI Robot Brain system:
```bash
cd src/isaac_examples/reinforcement_learning
python test_deployment.py
```

## Building the Documentation

To build and serve the documentation locally:

```bash
cd docusaurus-textbook
npm install
npm run start
```

The documentation will be available at `http://localhost:3000`.

## Architecture Overview

The AI Robot Brain module includes several key components:

- **Isaac Sim Scene Configuration**: High-fidelity simulation environment
- **Isaac ROS Perception Pipeline**: GPU-accelerated object detection and pose estimation
- **Manipulation Intelligence**: Grasp planning and arm control systems
- **Reinforcement Learning**: Training and deployment pipeline for robotic behaviors
- **Sim-to-Real Transfer**: Techniques for bridging simulation and reality

## Troubleshooting

- If Isaac Sim fails to launch, verify GPU drivers and CUDA installation
- If ROS 2 nodes don't communicate, check network configuration and ROS_DOMAIN_ID
- For perception pipeline issues, ensure sensor data is properly configured
- For RL training issues, verify GPU memory and PyTorch installation
- For deployment failures, check safety constraints and robot connectivity
