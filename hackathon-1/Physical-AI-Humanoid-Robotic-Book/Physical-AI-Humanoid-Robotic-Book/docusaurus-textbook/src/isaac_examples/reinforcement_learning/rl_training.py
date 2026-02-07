#!/usr/bin/env python3

"""
Reinforcement Learning Training Loop in Simulation

This module implements a complete reinforcement learning training loop using Isaac Sim
for generating training data and training policies that can be deployed to real robots.
The implementation includes simulation environment setup, policy training, and validation.
"""

import os
import pickle
import random
import threading
import time
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float32, Header, Int32
from torch.distributions import Normal

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


class ActorCriticNetwork(nn.Module):
    """
    Actor-Critic neural network for reinforcement learning
    """

    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 256):
        super(ActorCriticNetwork, self).__init__()

        # Shared layers
        self.shared_layers = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )

        # Actor (policy) network
        self.actor_mean = nn.Linear(hidden_dim, action_dim)
        self.actor_std = nn.Linear(hidden_dim, action_dim)

        # Critic (value) network
        self.critic = nn.Linear(hidden_dim, 1)

    def forward(
        self, state: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Forward pass through the network

        Args:
            state: Current state tensor

        Returns:
            Tuple of (action_mean, action_std, value)
        """
        shared_out = self.shared_layers(state)

        # Actor output
        action_mean = torch.tanh(self.actor_mean(shared_out))
        action_std = F.softplus(self.actor_std(shared_out))

        # Critic output
        value = self.critic(shared_out)

        return action_mean, action_std, value


class ReplayBuffer:
    """
    Experience replay buffer for storing and sampling experiences
    """

    def __init__(self, capacity: int):
        self.buffer = deque(maxlen=capacity)

    def push(
        self,
        state: np.ndarray,
        action: np.ndarray,
        reward: float,
        next_state: np.ndarray,
        done: bool,
    ):
        """
        Add experience to buffer

        Args:
            state: Current state
            action: Action taken
            reward: Reward received
            next_state: Next state after action
            done: Whether episode is done
        """
        experience = (state, action, reward, next_state, done)
        self.buffer.append(experience)

    def sample(
        self, batch_size: int
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Sample experiences from buffer

        Args:
            batch_size: Number of experiences to sample

        Returns:
            Tuple of (states, actions, rewards, next_states, dones)
        """
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state, done = map(np.stack, zip(*batch))
        return state, action, reward, next_state, done

    def __len__(self) -> int:
        """
        Return current size of buffer
        """
        return len(self.buffer)


class PPOAgent:
    """
    Proximal Policy Optimization (PPO) agent implementation
    """

    def __init__(
        self,
        state_dim: int,
        action_dim: int,
        lr: float = 3e-4,
        gamma: float = 0.99,
        eps_clip: float = 0.2,
        k_epochs: int = 4,
    ):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.k_epochs = k_epochs

        self.network = ActorCriticNetwork(state_dim, action_dim)
        self.optimizer = optim.Adam(self.network.parameters(), lr=lr)

        self.MseLoss = nn.MSELoss()

    def select_action(self, state: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Select action based on current state

        Args:
            state: Current state

        Returns:
            Tuple of (action, log_prob)
        """
        state = torch.FloatTensor(state).unsqueeze(0)
        action_mean, action_std, _ = self.network(state)

        # Create normal distribution for sampling
        dist = Normal(action_mean, action_std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)

        return action.detach().cpu().numpy()[0], log_prob.detach().cpu().numpy()[0]

    def update(
        self,
        states: torch.Tensor,
        actions: torch.Tensor,
        log_probs: torch.Tensor,
        rewards: torch.Tensor,
        next_states: torch.Tensor,
        dones: torch.Tensor,
    ):
        """
        Update network parameters using PPO

        Args:
            states: Batch of states
            actions: Batch of actions
            log_probs: Batch of log probabilities
            rewards: Batch of rewards
            next_states: Batch of next states
            dones: Batch of done flags
        """
        # Calculate discounted rewards
        discounted_rewards = []
        running_reward = 0

        for reward, done in zip(reversed(rewards), reversed(dones)):
            if done:
                running_reward = 0
            running_reward = reward + self.gamma * running_reward
            discounted_rewards.insert(0, running_reward)

        discounted_rewards = torch.FloatTensor(discounted_rewards)
        discounted_rewards = (discounted_rewards - discounted_rewards.mean()) / (
            discounted_rewards.std() + 1e-5
        )

        for _ in range(self.k_epochs):
            # Get current policy values
            action_means, action_stds, state_values = self.network(states)

            # Create distribution
            dist = Normal(action_means, action_stds)
            new_log_probs = dist.log_prob(actions).sum(dim=-1)
            entropy = dist.entropy().sum(dim=-1)

            # Calculate ratios
            ratios = torch.exp(new_log_probs - log_probs)

            # Calculate advantages
            advantages = discounted_rewards - state_values.squeeze()

            # Calculate surrogates
            surr1 = ratios * advantages
            surr2 = (
                torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages
            )

            # Calculate losses
            actor_loss = -torch.min(surr1, surr2).mean()
            critic_loss = self.MseLoss(state_values.squeeze(), discounted_rewards)
            entropy_loss = entropy.mean()

            # Total loss
            loss = actor_loss + 0.5 * critic_loss - 0.01 * entropy_loss

            # Update network
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()


class IsaacSimEnvironment:
    """
    Isaac Sim environment wrapper for reinforcement learning
    """

    def __init__(self, robot_name: str = "franka", task: str = "pick_and_place"):
        self.robot_name = robot_name
        self.task = task
        self.world = None
        self.robot = None
        self.objects = []

        # Initialize simulation if Isaac Sim is available
        if World is not None:
            self.world = World(stage_units_in_meters=1.0)
            self.setup_environment()
        else:
            print("Using mock Isaac Sim environment for testing")

    def setup_environment(self):
        """
        Set up the Isaac Sim environment
        """
        if self.world is None:
            return

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Add robot
        if self.robot_name == "franka":
            # Add Franka robot
            self.robot = self.world.scene.add(
                Robot(
                    prim_path="/World/Franka",
                    name="franka_robot",
                    usd_path="/Isaac/Robots/Franka/franka_alt_fingers.usd",
                    position=[0, 0, 0],
                    orientation=[0, 0, 0, 1],
                )
            )
        else:
            # Add default robot
            self.robot = self.world.scene.add(
                Robot(
                    prim_path="/World/Robot",
                    name="default_robot",
                    usd_path="/Isaac/Robots/UniversalRobots/UR10/ur10.usd",
                    position=[0, 0, 0],
                    orientation=[0, 0, 0, 1],
                )
            )

        # Add objects for manipulation task
        for i in range(5):
            obj = self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/Object_{i}",
                    name=f"object_{i}",
                    position=[0.5 + i * 0.1, 0, 0.1],
                    size=0.05,
                    color=np.random.rand(3),
                )
            )
            self.objects.append(obj)

        print(f"Isaac Sim environment set up with {len(self.objects)} objects")

    def reset(self) -> np.ndarray:
        """
        Reset the environment to initial state

        Returns:
            Initial state as numpy array
        """
        if self.world is not None:
            self.world.reset()

            # Reset robot position
            if self.robot is not None:
                self.robot.set_world_poses(
                    positions=torch.tensor([[0.0, 0.0, 0.0]]),
                    orientations=torch.tensor([[0.0, 0.0, 0.0, 1.0]]),
                )

        # Return initial state (simplified for this example)
        initial_state = np.zeros(24)  # 24-dim state vector
        return initial_state

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """
        Take a step in the environment

        Args:
            action: Action to take

        Returns:
            Tuple of (next_state, reward, done, info)
        """
        if self.world is not None:
            # Apply action to robot in simulation
            # This is a simplified implementation
            # In a real implementation, you would control the robot joints

            # Simulate the action
            time.sleep(0.01)  # Simulate time delay

            # Get new state from simulation
            next_state = np.random.rand(24)  # Simplified state

            # Calculate reward (simplified)
            reward = np.random.rand() * 2 - 1  # Random reward between -1 and 1

            # Determine if episode is done
            done = random.random() < 0.01  # 1% chance of episode ending

            info = {"step": 0}  # Additional info
        else:
            # Mock implementation for testing
            next_state = np.random.rand(24)
            reward = np.random.rand() * 2 - 1
            done = random.random() < 0.01
            info = {"step": 0}

        return next_state, reward, done, info

    def close(self):
        """
        Close the environment
        """
        if self.world is not None:
            self.world.clear()
            self.world = None


class RLTrainingNode(Node):
    """
    ROS 2 node for reinforcement learning training
    """

    def __init__(self):
        super().__init__("rl_training_node")

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

        # Create publishers for training metrics
        self.episode_reward_pub = self.create_publisher(Float32, "episode_reward", 10)
        self.episode_length_pub = self.create_publisher(Int32, "episode_length", 10)
        self.training_status_pub = self.create_publisher(Header, "training_status", 10)

        # Initialize components
        self.environment = IsaacSimEnvironment(self.robot_name, self.task)
        self.agent = PPOAgent(
            self.state_dim, self.action_dim, self.learning_rate, self.gamma
        )
        self.replay_buffer = ReplayBuffer(self.replay_buffer_size)

        # Training metrics
        self.episode_rewards = []
        self.episode_lengths = []
        self.training_thread = None
        self.training_active = False

        # Create model save directory
        os.makedirs(self.model_save_path, exist_ok=True)

        self.get_logger().info("RL Training Node initialized")
        self.get_logger().info(
            f"Training parameters: episodes={self.max_episodes}, "
            f"state_dim={self.state_dim}, action_dim={self.action_dim}"
        )

    def start_training(self):
        """
        Start the reinforcement learning training process
        """
        self.get_logger().info("Starting RL training...")
        self.training_active = True

        # Publish training start status
        status_header = Header()
        status_header.stamp = self.get_clock().now().to_msg()
        status_header.frame_id = "training_started"
        self.training_status_pub.publish(status_header)

        # Start training in a separate thread
        self.training_thread = threading.Thread(target=self.training_worker)
        self.training_thread.start()

    def training_worker(self):
        """
        Worker thread for training the RL agent
        """
        for episode in range(self.max_episodes):
            if not self.training_active:
                break

            self.get_logger().info(
                f"Starting episode {episode + 1}/{self.max_episodes}"
            )

            # Reset environment
            state = self.environment.reset()
            episode_reward = 0
            episode_length = 0

            # Run episode
            for step in range(self.max_steps_per_episode):
                if not self.training_active:
                    break

                # Select action
                action, log_prob = self.agent.select_action(state)

                # Take action in environment
                next_state, reward, done, info = self.environment.step(action)

                # Store experience in replay buffer
                self.replay_buffer.push(state, action, reward, next_state, done)

                # Update state
                state = next_state
                episode_reward += reward
                episode_length += 1

                # Update network if buffer has enough samples
                if len(self.replay_buffer) > self.batch_size:
                    self.update_agent()

                if done:
                    break

            # Store episode metrics
            self.episode_rewards.append(episode_reward)
            self.episode_lengths.append(episode_length)

            # Publish episode metrics
            reward_msg = Float32()
            reward_msg.data = float(episode_reward)
            self.episode_reward_pub.publish(reward_msg)

            length_msg = Int32()
            length_msg.data = episode_length
            self.episode_length_pub.publish(length_msg)

            self.get_logger().info(
                f"Episode {episode + 1}: Reward={episode_reward:.2f}, "
                f"Length={episode_length}"
            )

            # Save model periodically
            if (episode + 1) % self.save_model_interval == 0:
                self.save_model(episode + 1)

        self.get_logger().info("RL training completed")

        # Publish training completion status
        status_header = Header()
        status_header.stamp = self.get_clock().now().to_msg()
        status_header.frame_id = "training_completed"
        self.training_status_pub.publish(status_header)

    def update_agent(self):
        """
        Update the agent using experiences from the replay buffer
        """
        # Sample batch from replay buffer
        states, actions, rewards, next_states, dones = self.replay_buffer.sample(
            self.batch_size
        )

        # Convert to tensors
        states = torch.FloatTensor(states)
        actions = torch.FloatTensor(actions)
        rewards = torch.FloatTensor(rewards)
        next_states = torch.FloatTensor(next_states)
        dones = torch.BoolTensor(dones)

        # Get log probabilities for current policy
        with torch.no_grad():
            action_means, action_stds, _ = self.agent.network(states)
            dist = Normal(action_means, action_stds)
            log_probs = dist.log_prob(actions).sum(dim=-1)

        # Update agent
        self.agent.update(states, actions, log_probs, rewards, next_states, dones)

    def save_model(self, episode: int):
        """
        Save the trained model

        Args:
            episode: Current episode number for naming
        """
        model_path = os.path.join(
            self.model_save_path, f"rl_model_episode_{episode}.pth"
        )

        # Save model state
        torch.save(
            {
                "episode": episode,
                "model_state_dict": self.agent.network.state_dict(),
                "optimizer_state_dict": self.agent.optimizer.state_dict(),
                "episode_rewards": self.episode_rewards,
                "episode_lengths": self.episode_lengths,
            },
            model_path,
        )

        self.get_logger().info(f"Model saved to {model_path}")

    def load_model(self, model_path: str):
        """
        Load a trained model

        Args:
            model_path: Path to the model file
        """
        checkpoint = torch.load(model_path)
        self.agent.network.load_state_dict(checkpoint["model_state_dict"])
        self.agent.optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
        self.episode_rewards = checkpoint["episode_rewards"]
        self.episode_lengths = checkpoint["episode_lengths"]

        self.get_logger().info(f"Model loaded from {model_path}")

    def stop_training(self):
        """
        Stop the training process
        """
        self.get_logger().info("Stopping RL training...")
        self.training_active = False

        if self.training_thread and self.training_thread.is_alive():
            self.training_thread.join()

        # Close environment
        self.environment.close()

        # Publish training stop status
        status_header = Header()
        status_header.stamp = self.get_clock().now().to_msg()
        status_header.frame_id = "training_stopped"
        self.training_status_pub.publish(status_header)


def main(args=None):
    """
    Main function to run the RL Training Node
    """
    rclpy.init(args=args)

    rl_training_node = RLTrainingNode()

    try:
        # Start training
        rl_training_node.start_training()

        # Run node
        rclpy.spin(rl_training_node)

    except KeyboardInterrupt:
        rl_training_node.get_logger().info("Interrupted by user")
    finally:
        rl_training_node.stop_training()
        rl_training_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
