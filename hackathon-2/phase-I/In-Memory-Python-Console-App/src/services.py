"""
Business logic and CRUD operations for tasks.
"""

from typing import Optional, List
from datetime import datetime, timedelta
from .models import Task, RecurrencePattern
from . import storage


def validate_title(title: str) -> bool:
    """
    Validate that a task title is non-empty.

    Args:
        title: The title string to validate

    Returns:
        True if valid, False otherwise
    """
    return bool(title and title.strip())


def validate_priority(priority: str) -> bool:
    """
    Validate that a priority is valid.

    Args:
        priority: The priority string to validate

    Returns:
        True if valid (High/Medium/Low), False otherwise
    """
    return priority in ["High", "Medium", "Low"]


def create_task(title: str, priority: str = "Medium", tags: Optional[List[str]] = None) -> Task:
    """
    Create a new task.

    Args:
        title: Task title (required, non-empty)
        priority: Priority level (High/Medium/Low, default: Medium)
        tags: List of tags (optional)

    Returns:
        Created Task object

    Raises:
        ValueError: If title is empty or priority is invalid
    """
    if not validate_title(title):
        raise ValueError("Task title cannot be empty")

    if not validate_priority(priority):
        raise ValueError("Priority must be High, Medium, or Low")

    task = Task(
        id=storage.get_next_id(),
        title=title.strip(),
        priority=priority,
        tags=tags if tags else []
    )
    storage.add_task_to_storage(task)
    return task


def read_task(task_id: int) -> Optional[Task]:
    """
    Retrieve a task by ID.

    Args:
        task_id: ID of the task to retrieve

    Returns:
        Task object if found, None otherwise
    """
    return storage.get_task_by_id(task_id)


def read_all_tasks() -> List[Task]:
    """
    Get all tasks.

    Returns:
        List of all tasks
    """
    return storage.get_all_tasks()


def update_task(task_id: int, title: Optional[str] = None,
                priority: Optional[str] = None, tags: Optional[List[str]] = None) -> bool:
    """
    Update a task's attributes.

    Args:
        task_id: ID of the task to update
        title: New title (optional)
        priority: New priority (optional)
        tags: New tags list (optional)

    Returns:
        True if task was updated, False if not found

    Raises:
        ValueError: If new title is empty or priority is invalid
    """
    task = storage.get_task_by_id(task_id)
    if not task:
        return False

    if title is not None:
        if not validate_title(title):
            raise ValueError("Task title cannot be empty")
        task.title = title.strip()

    if priority is not None:
        if not validate_priority(priority):
            raise ValueError("Priority must be High, Medium, or Low")
        task.priority = priority

    if tags is not None:
        task.tags = tags

    return True


def delete_task(task_id: int) -> bool:
    """
    Delete a task by ID.

    Args:
        task_id: ID of the task to delete

    Returns:
        True if task was deleted, False if not found
    """
    return storage.delete_task_from_storage(task_id)


def toggle_completion(task_id: int) -> bool:
    """
    Toggle a task's completion status.
    If task is recurring and being marked complete, regenerate next occurrence.

    Args:
        task_id: ID of the task to toggle

    Returns:
        True if task was toggled, False if not found
    """
    task = storage.get_task_by_id(task_id)
    if not task:
        return False

    # Toggle completion
    task.completed = not task.completed

    # If marking complete and task is recurring, regenerate
    if task.completed and task.recurrence:
        regenerate_recurring_task(task_id)

    return True


# Organization operations

def search_tasks(keyword: str) -> List[Task]:
    """
    Search tasks by keyword in title (case-insensitive).

    Args:
        keyword: Search term

    Returns:
        List of matching tasks
    """
    if not keyword or not keyword.strip():
        return []

    keyword_lower = keyword.strip().lower()
    return [task for task in storage.get_all_tasks()
            if keyword_lower in task.title.lower()]


def filter_by_status(completed: bool) -> List[Task]:
    """
    Filter tasks by completion status.

    Args:
        completed: True for completed, False for pending

    Returns:
        List of filtered tasks
    """
    return [task for task in storage.get_all_tasks()
            if task.completed == completed]


def filter_by_priority(priority: str) -> List[Task]:
    """
    Filter tasks by priority level.

    Args:
        priority: Priority level (High/Medium/Low)

    Returns:
        List of tasks with matching priority

    Raises:
        ValueError: If priority is invalid
    """
    if not validate_priority(priority):
        raise ValueError("Priority must be High, Medium, or Low")

    return [task for task in storage.get_all_tasks()
            if task.priority == priority]


def filter_by_tag(tag: str) -> List[Task]:
    """
    Filter tasks by tag.

    Args:
        tag: Tag name to search for

    Returns:
        List of tasks containing the tag
    """
    if not tag or not tag.strip():
        return []

    tag_lower = tag.strip().lower()
    return [task for task in storage.get_all_tasks()
            if any(t.lower() == tag_lower for t in task.tags)]


def sort_tasks(tasks: List[Task], by: str, reverse: bool = False) -> List[Task]:
    """
    Sort tasks by different criteria.

    Args:
        tasks: List of tasks to sort
        by: Sort criteria ('priority', 'date', 'title')
        reverse: True for descending order

    Returns:
        Sorted list of tasks
    """
    if by == "priority":
        priority_order = {"High": 3, "Medium": 2, "Low": 1}
        return sorted(tasks, key=lambda t: priority_order.get(t.priority, 0), reverse=reverse)
    elif by == "date":
        return sorted(tasks, key=lambda t: t.created_at, reverse=reverse)
    elif by == "title":
        return sorted(tasks, key=lambda t: t.title.lower(), reverse=reverse)
    else:
        return tasks


# Advanced operations (due dates and recurring tasks)

def validate_date(date_str: str) -> Optional[datetime]:
    """
    Parse and validate a datetime string.

    Args:
        date_str: Date string in format YYYY-MM-DD HH:MM or YYYY-MM-DD

    Returns:
        Parsed datetime object or None if invalid
    """
    if not date_str or not date_str.strip():
        return None

    formats = [
        "%Y-%m-%d %H:%M",      # ISO 8601: 2025-12-30 14:00
        "%Y-%m-%d",            # Date only: 2025-12-30 (assumes 00:00)
    ]

    for fmt in formats:
        try:
            return datetime.strptime(date_str.strip(), fmt)
        except ValueError:
            continue

    return None


def set_due_date(task_id: int, due_date: datetime) -> bool:
    """
    Set due date for a task.

    Args:
        task_id: ID of the task
        due_date: Due date to set

    Returns:
        True if successful, False if task not found
    """
    task = storage.get_task_by_id(task_id)
    if not task:
        return False

    task.due_date = due_date
    return True


def set_recurrence(task_id: int, frequency: str, start_date: datetime) -> bool:
    """
    Set recurrence pattern for a task.

    Args:
        task_id: ID of the task
        frequency: Recurrence frequency (Daily/Weekly/Monthly)
        start_date: Start date for recurrence

    Returns:
        True if successful, False if task not found
    """
    task = storage.get_task_by_id(task_id)
    if not task:
        return False

    if frequency not in ["Daily", "Weekly", "Monthly"]:
        raise ValueError("Frequency must be Daily, Weekly, or Monthly")

    task.recurrence = RecurrencePattern(
        frequency=frequency,
        original_due_date=start_date
    )
    task.due_date = start_date
    return True


def check_reminders() -> List[Task]:
    """
    Get tasks that are due now (within 5 minute tolerance).

    Returns:
        List of tasks due now
    """
    now = datetime.now()
    due_tasks = []

    for task in storage.get_all_tasks():
        if task.due_date and not task.completed:
            diff_minutes = abs((task.due_date - now).total_seconds() / 60)
            if diff_minutes <= 5:
                due_tasks.append(task)

    return due_tasks


def is_overdue(task: Task) -> bool:
    """
    Check if a task is overdue.

    Args:
        task: Task to check

    Returns:
        True if overdue, False otherwise
    """
    if not task.due_date or task.completed:
        return False

    return datetime.now() > task.due_date


def calculate_next_occurrence(due_date: datetime, frequency: str) -> datetime:
    """
    Calculate next occurrence date for recurring task.

    Args:
        due_date: Current due date
        frequency: Recurrence frequency (Daily/Weekly/Monthly)

    Returns:
        Next occurrence datetime
    """
    if frequency == "Daily":
        return due_date + timedelta(days=1)
    elif frequency == "Weekly":
        return due_date + timedelta(weeks=1)
    elif frequency == "Monthly":
        # Simple month addition (may need adjustment for edge cases)
        year = due_date.year + (due_date.month // 12)
        month = (due_date.month % 12) + 1
        if month == 13:
            month = 1
            year += 1
        try:
            return due_date.replace(year=year, month=month)
        except ValueError:
            # Handle day overflow (e.g., Jan 31 -> Feb 31 invalid)
            # Fall back to last day of month
            next_month = due_date.replace(day=1)
            next_month = next_month.replace(year=year, month=month)
            # Go to next month and subtract a day
            following_month = next_month.replace(month=(month % 12) + 1)
            if following_month.month == 1:
                following_month = following_month.replace(year=following_month.year + 1)
            return following_month - timedelta(days=1)
    else:
        return due_date


def regenerate_recurring_task(task_id: int) -> Optional[Task]:
    """
    Create next occurrence of a recurring task.

    Args:
        task_id: ID of the recurring task to regenerate

    Returns:
        New task instance or None if not recurring
    """
    task = storage.get_task_by_id(task_id)
    if not task or not task.recurrence:
        return None

    # Calculate next due date
    next_due = calculate_next_occurrence(task.due_date, task.recurrence.frequency)

    # Create new task instance
    new_task = Task(
        id=storage.get_next_id(),
        title=task.title,
        completed=False,  # Reset completion
        priority=task.priority,
        tags=task.tags.copy(),
        due_date=next_due,
        recurrence=task.recurrence
    )

    storage.add_task_to_storage(new_task)
    return new_task
