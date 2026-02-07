from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime
import uuid
from enum import Enum
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
import sqlalchemy as sa


class TaskPriority(str, Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


class TaskType(str, Enum):
    DAILY = "daily"
    WEEKLY = "weekly"
    MONTHLY = "monthly"


class Task(SQLModel, table=True):
    """
    Task entity representing a user's task with title, description, and completion status
    """
    id: str = Field(
        sa_column=sa.Column(
            PostgresUUID(as_uuid=True),
            primary_key=True,
            default=uuid.uuid4
        )
    )
    user_id: str = Field(sa_column=sa.Column(sa.Text, sa.ForeignKey("user.id"), nullable=False))
    parent_id: Optional[str] = Field(default=None, sa_column=sa.Column(PostgresUUID(as_uuid=True), sa.ForeignKey("task.id")))  # For hierarchical tasks
    title: str = Field(sa_column=sa.Column(sa.String(200), nullable=False))
    description: Optional[str] = Field(sa_column=sa.Column(sa.String(1000)))
    is_completed: bool = Field(default=False, sa_column=sa.Column(sa.Boolean, nullable=False))  # Boolean field in database
    priority: str = Field(default="medium", sa_column=sa.Column(sa.Enum(TaskPriority, name="priorityenum"), nullable=False))  # Enum: low, medium, high
    due_date: Optional[datetime] = Field(sa_column=sa.Column(sa.DateTime))
    recurrence_pattern: Optional[str] = Field(default=None, sa_column=sa.Column(sa.Enum(TaskType, name="tasktypeenum")))  # Pattern for recurring tasks
    order_index: str = Field(default="0", sa_column=sa.Column(sa.String, nullable=False))  # For ordering tasks
    created_at: datetime = Field(default_factory=datetime.utcnow, sa_column=sa.Column(sa.DateTime, nullable=False))
    updated_at: datetime = Field(default_factory=datetime.utcnow, sa_column=sa.Column(sa.DateTime, nullable=False))

    # Add index hints for common queries
    __table_args__ = (
        sa.Index('idx_task_user_id', 'user_id'),
        {"sqlite_autoincrement": True},
    )