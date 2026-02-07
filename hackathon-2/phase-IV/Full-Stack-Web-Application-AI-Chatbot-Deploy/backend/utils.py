from datetime import datetime, timedelta
from typing import Optional
from .models import RecurrenceEnum


def calculate_next_due_date(due_date: Optional[datetime], recurrence: RecurrenceEnum) -> Optional[datetime]:
    """
    Calculate the next due date based on recurrence pattern.

    Args:
        due_date: The original due date
        recurrence: The recurrence pattern

    Returns:
        Optional[datetime]: The next due date or None if no recurrence
    """
    if not due_date:
        return None

    if recurrence == RecurrenceEnum.daily:
        return due_date + timedelta(days=1)
    elif recurrence == RecurrenceEnum.weekly:
        return due_date + timedelta(weeks=1)
    else:
        return None  # No recurrence


def create_recurring_task(original_task_data: dict) -> dict:
    """
    Create data for a new recurring task based on the original task.

    Args:
        original_task_data: Dictionary containing the original task data

    Returns:
        dict: New task data with updated fields for the recurring instance
    """
    new_task_data = original_task_data.copy()

    # Reset completion status for the new instance
    new_task_data['completed'] = False

    # Update timestamps
    new_task_data['created_at'] = datetime.utcnow()
    new_task_data['updated_at'] = datetime.utcnow()

    # Calculate new due date based on recurrence
    recurrence = original_task_data.get('recurrence')
    if recurrence and recurrence != RecurrenceEnum.none and original_task_data.get('due_date'):
        new_due_date = calculate_next_due_date(
            original_task_data['due_date'],
            recurrence
        )
        new_task_data['due_date'] = new_due_date

    return new_task_data


def limit_recurrence_generations(task_data: dict, max_instances: int = 50) -> bool:
    """
    Check if a recurring task has reached its maximum allowed instances.

    Args:
        task_data: Dictionary containing task data
        max_instances: Maximum number of allowed instances (default 50)

    Returns:
        bool: True if the task should be allowed to create another instance
    """
    # In a real implementation, this would check against a counter in the database
    # For now, we'll just return True to allow the recurrence
    return True