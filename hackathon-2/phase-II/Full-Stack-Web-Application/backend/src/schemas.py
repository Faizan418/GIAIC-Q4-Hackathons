from pydantic import BaseModel, field_validator
from typing import Optional, List
from datetime import datetime
from enum import Enum


class PriorityEnum(str, Enum):
    """Priority levels for tasks"""
    high = "high"
    medium = "medium"
    low = "low"


class RecurrenceEnum(str, Enum):
    """Recurrence patterns for tasks"""
    none = "none"
    daily = "daily"
    weekly = "weekly"


class TaskBase(BaseModel):
    """Base schema for task data"""
    title: str
    description: Optional[str] = None
    priority: Optional[PriorityEnum] = None
    tags: List[str] = []
    due_date: Optional[datetime] = None
    recurrence: Optional[RecurrenceEnum] = RecurrenceEnum.none

    @field_validator('title')
    @classmethod
    def validate_title(cls, v):
        if len(v) < 1 or len(v) > 100:
            raise ValueError('Title must be between 1 and 100 characters')
        return v

    @field_validator('description')
    @classmethod
    def validate_description(cls, v):
        if v and len(v) > 5000:
            raise ValueError('Description cannot exceed 5000 characters')
        return v


class TaskCreate(TaskBase):
    """Schema for creating a new task"""
    title: str  # Required field
    # Other fields inherited from TaskBase


class TaskUpdate(BaseModel):
    """Schema for updating an existing task (all fields optional for partial updates)"""
    title: Optional[str] = None
    description: Optional[str] = None
    completed: Optional[bool] = None
    priority: Optional[PriorityEnum] = None
    tags: Optional[List[str]] = None
    due_date: Optional[datetime] = None
    recurrence: Optional[RecurrenceEnum] = None

    @field_validator('title')
    @classmethod
    def validate_title(cls, v):
        if v is not None and (len(v) < 1 or len(v) > 100):
            raise ValueError('Title must be between 1 and 100 characters')
        return v

    @field_validator('description')
    @classmethod
    def validate_description(cls, v):
        if v and len(v) > 5000:
            raise ValueError('Description cannot exceed 5000 characters')
        return v


class TaskToggleComplete(BaseModel):
    """Schema for toggling task completion status"""
    completed: Optional[bool] = None


from pydantic import ConfigDict

class TaskResponse(TaskBase):
    """Schema for task response with additional fields"""
    id: str  # Changed from int to str to match the Task model
    user_id: str
    completed: bool  # This is computed from status field
    created_at: datetime
    updated_at: datetime
    status: str  # Added to match the Task model

    model_config = ConfigDict(from_attributes=True)  # Enable ORM mode for Pydantic v2


class ApiResponse(BaseModel):
    """Generic response wrapper"""
    success: bool
    data: Optional[dict] = None
    message: Optional[str] = None


class ErrorResponse(BaseModel):
    """Generic error response"""
    success: bool
    error: dict


class TaskListResponse(BaseModel):
    """Response for task list with pagination info"""
    success: bool
    data: dict