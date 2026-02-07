"""
In-memory storage management for tasks.
"""

from typing import Optional, List
from .models import Task

# In-memory storage structures
tasks_list: List[Task] = []  # Primary storage, maintains order
tasks_index: dict[int, Task] = {}  # Fast O(1) lookups by ID
next_id: int = 1  # ID counter for unique task IDs


def initialize_storage() -> None:
    """Initialize or reset the storage structures."""
    global tasks_list, tasks_index, next_id
    tasks_list = []
    tasks_index = {}
    next_id = 1


def get_next_id() -> int:
    """
    Generate the next unique task ID.

    Returns:
        int: Next available task ID
    """
    global next_id
    current_id = next_id
    next_id += 1
    return current_id


def add_task_to_storage(task: Task) -> None:
    """
    Add a task to storage.

    Args:
        task: Task object to add
    """
    tasks_list.append(task)
    tasks_index[task.id] = task


def get_task_by_id(task_id: int) -> Optional[Task]:
    """
    Retrieve a task by its ID.

    Args:
        task_id: ID of the task to retrieve

    Returns:
        Task object if found, None otherwise
    """
    return tasks_index.get(task_id)


def get_all_tasks() -> List[Task]:
    """
    Get all tasks from storage.

    Returns:
        Copy of the tasks list
    """
    return tasks_list.copy()


def delete_task_from_storage(task_id: int) -> bool:
    """
    Delete a task from storage by ID.

    Args:
        task_id: ID of the task to delete

    Returns:
        True if task was deleted, False if not found
    """
    if task_id not in tasks_index:
        return False

    task = tasks_index[task_id]
    tasks_list.remove(task)
    del tasks_index[task_id]
    return True


def clear_storage() -> None:
    """Clear all tasks from storage."""
    global tasks_list, tasks_index, next_id
    tasks_list = []
    tasks_index = {}
    next_id = 1
