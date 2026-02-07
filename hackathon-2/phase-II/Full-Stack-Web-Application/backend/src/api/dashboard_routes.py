from fastapi import APIRouter, HTTPException, status, Depends
from sqlmodel.ext.asyncio.session import AsyncSession
from typing import Dict, Any
from pydantic import BaseModel
from ..database.db import get_async_session
from ..services.task_service import TaskService
from ..models.user import User
from ..models.task import Task

router = APIRouter(prefix="/dashboard", tags=["dashboard"])

@router.get("/stats", response_model=Dict[str, Any])
async def get_dashboard_stats(
    session: AsyncSession = Depends(get_async_session)
):
    """
    Get dashboard statistics (public access).
    """
    try:
        # Get all tasks from the database for aggregate statistics
        from sqlmodel import select
        result = await session.execute(select(Task))
        all_tasks = result.scalars().all()

        # Calculate statistics based on the actual Task model fields
        total_tasks = len(all_tasks)
        completed_tasks = len([task for task in all_tasks if task.is_completed])
        pending_tasks = total_tasks - completed_tasks  # Calculate from total instead of checking for "pending"

        # Count tasks by recurrence pattern
        task_types = {"daily": 0, "weekly": 0, "monthly": 0}
        for task in all_tasks:
            if task.recurrence_pattern and task.recurrence_pattern in task_types:
                task_types[task.recurrence_pattern] += 1

        # Count tasks by priority
        task_priorities = {"low": 0, "medium": 0, "high": 0}
        for task in all_tasks:
            if task.priority and task.priority in task_priorities:
                task_priorities[task.priority] += 1

        return {
            "success": True,
            "data": {
                "total_tasks": total_tasks,
                "completed_tasks": completed_tasks,
                "pending_tasks": pending_tasks,
                "task_types": task_types,
                "task_priorities": task_priorities
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving dashboard stats: {str(e)}")