from datetime import datetime, timedelta
from typing import Optional
from slowapi import Limiter
from slowapi.util import get_remote_address
from pytz import utc

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

def calculate_next_occurrence(due_date: datetime, recurrence_pattern: str) -> Optional[datetime]:
    """
    Calculate the next occurrence date based on the recurrence pattern

    Args:
        due_date: Original due date
        recurrence_pattern: 'none', 'daily', or 'weekly'

    Returns:
        New datetime for next occurrence or None if no recurrence
    """
    if recurrence_pattern == "none":
        return None
    elif recurrence_pattern == "daily":
        return due_date + timedelta(days=1)
    elif recurrence_pattern == "weekly":
        return due_date + timedelta(days=7)
    else:
        # Unknown recurrence pattern, return None
        return None


def get_next_occurrence_from_now(recurrence_pattern: str) -> Optional[datetime]:
    """
    Calculate the next occurrence date from the current time based on the recurrence pattern

    Args:
        recurrence_pattern: 'none', 'daily', or 'weekly'

    Returns:
        New datetime for next occurrence or None if no recurrence
    """
    if recurrence_pattern == "none":
        return None
    elif recurrence_pattern == "daily":
        return datetime.now() + timedelta(days=1)
    elif recurrence_pattern == "weekly":
        return datetime.now() + timedelta(days=7)
    else:
        # Unknown recurrence pattern, return None
        return None


def normalize_timezone(dt: Optional[datetime]) -> Optional[datetime]:
    """
    Normalize a datetime to UTC timezone if it has timezone info
    If the datetime is naive (no timezone), assume it's UTC

    Args:
        dt: Datetime to normalize

    Returns:
        Normalized datetime in UTC
    """
    if dt is None:
        return None

    if dt.tzinfo is None:
        # Naive datetime, assume UTC
        return dt.replace(tzinfo=utc)
    else:
        # Convert to UTC
        return dt.astimezone(utc)


def is_valid_recurrence_pattern(pattern: str) -> bool:
    """
    Check if the recurrence pattern is valid

    Args:
        pattern: Recurrence pattern to validate

    Returns:
        True if valid, False otherwise
    """
    return pattern in ["none", "daily", "weekly"]


def create_recurring_task_data(original_task_data: dict) -> dict:
    """
    Create data for a new recurring task instance based on the original task

    Args:
        original_task_data: Data of the original task that is recurring

    Returns:
        New task data with updated fields for the next occurrence
    """
    # Copy the original data
    new_task_data = original_task_data.copy()

    # Update the relevant fields for the new occurrence
    new_task_data['completed'] = False  # New tasks are not completed

    # Calculate and update due date based on recurrence
    if new_task_data.get('due_date') and new_task_data.get('recurrence'):
        new_due_date = calculate_next_occurrence(new_task_data['due_date'], new_task_data['recurrence'])
        if new_due_date:
            new_task_data['due_date'] = new_due_date

    # Remove the ID to generate a new one
    new_task_data.pop('id', None)

    # Update timestamps
    now = datetime.now()
    new_task_data['created_at'] = now
    new_task_data['updated_at'] = now

    return new_task_data