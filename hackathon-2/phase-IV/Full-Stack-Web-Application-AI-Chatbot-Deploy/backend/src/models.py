# Dynamically import models from the models subdirectory to avoid circular imports
import importlib.util
import os
from pathlib import Path

# Get the directory containing this file
current_dir = Path(__file__).parent
models_subdir = current_dir / "models"

# Load the User class from models/user.py
user_spec = importlib.util.spec_from_file_location("user", models_subdir / "user.py")
user_module = importlib.util.module_from_spec(user_spec)
user_spec.loader.exec_module(user_module)
User = user_module.User

# Load the Task class from models/task.py
task_spec = importlib.util.spec_from_file_location("task", models_subdir / "task.py")
task_module = importlib.util.module_from_spec(task_spec)
task_spec.loader.exec_module(task_module)
Task = task_module.Task