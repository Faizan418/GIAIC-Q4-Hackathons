from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime
from .models import PriorityEnum, RecurrenceEnum


# Base models
class TaskBase(BaseModel):
    title: str
    description: Optional[str] = None
    completed: bool = False
    priority: PriorityEnum = PriorityEnum.medium
    tags: Optional[List[str]] = []
    due_date: Optional[datetime] = None
    recurrence: RecurrenceEnum = RecurrenceEnum.none


# Models for API requests/responses
class TaskCreate(TaskBase):
    title: str


class TaskUpdate(BaseModel):
    title: Optional[str] = None
    description: Optional[str] = None
    completed: Optional[bool] = None
    priority: Optional[PriorityEnum] = None
    tags: Optional[List[str]] = None
    due_date: Optional[datetime] = None
    recurrence: Optional[RecurrenceEnum] = None


class TaskRead(TaskBase):
    id: int
    user_id: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class TaskUpdateStatus(BaseModel):
    completed: bool


class UserRead(BaseModel):
    id: str
    email: str
    name: str
    created_at: datetime

    class Config:
        from_attributes = True


# Response models
class TaskListResponse(BaseModel):
    tasks: List[TaskRead]
    total: int
    offset: int
    limit: int