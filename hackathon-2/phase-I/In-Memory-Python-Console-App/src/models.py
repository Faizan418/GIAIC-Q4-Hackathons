"""
Task data models for the Todo application.
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional, List


@dataclass
class RecurrencePattern:
    """Represents a recurring task pattern."""
    frequency: str  # "Daily", "Weekly", "Monthly"
    original_due_date: datetime


@dataclass
class Task:
    """Represents a todo task with all its attributes."""
    id: int
    title: str
    completed: bool = False
    priority: str = "Medium"  # High, Medium, Low
    tags: List[str] = field(default_factory=list)
    created_at: datetime = field(default_factory=datetime.now)
    due_date: Optional[datetime] = None
    recurrence: Optional[RecurrencePattern] = None

    def __str__(self) -> str:
        """String representation of the task."""
        status = "✓" if self.completed else " "
        priority_icon = {"High": "🔴", "Medium": "🟡", "Low": "🟢"}.get(self.priority, "")
        tags_str = f" [{', '.join(self.tags)}]" if self.tags else ""

        # Add due date info
        due_str = ""
        if self.due_date:
            due_str = f" | Due: {self.due_date.strftime('%Y-%m-%d %H:%M')}"
            # Check if overdue
            if datetime.now() > self.due_date and not self.completed:
                due_str += " ⚠️ OVERDUE"

        # Add recurrence info
        rec_str = ""
        if self.recurrence:
            rec_str = f" | Recurring: {self.recurrence.frequency}"

        return f"[{status}] {priority_icon} #{self.id}: {self.title}{tags_str}{due_str}{rec_str}"
