from fastapi import APIRouter, Depends, HTTPException, status
from sqlmodel.ext.asyncio.session import AsyncSession
from typing import List, Optional
from datetime import datetime
from ..database.db import get_async_session
from ..services.task_service import TaskService
from ..services.task_service import TaskCreate, TaskUpdate, TaskResponse


router = APIRouter(tags=["tasks"])

@router.get("/", response_model=dict)
async def get_tasks(
    status: Optional[str] = None,
    priority: Optional[str] = None,
    search: Optional[str] = None,
    limit: int = 100,
    offset: int = 0,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Get all tasks with optional filtering (public access model).
    """
    try:
        # Get tasks with filtering options
        tasks, total_count = await TaskService.get_filtered_tasks(session, status, priority, search, limit, offset)

        return {
            "success": True,
            "data": {
                "tasks": tasks,
                "total": total_count
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error fetching tasks: {str(e)}")


@router.post("/", response_model=dict)
async def create_task(
    task_data: TaskCreate,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Create a new task (public access model).
    """
    try:
        # For public access model, assign a default user ID
        default_user_id = "public-user"
        task = await TaskService.create_task(task_data, default_user_id, session)
        return {
            "success": True,
            "data": {
                "task": task
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating task: {str(e)}")


@router.get("/{task_id}", response_model=dict)
async def get_task(
    task_id: str,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Get a specific task by ID (public access model).
    """
    try:
        # In public access model, we'll allow access to any task
        task = await TaskService.get_task_by_id_public(task_id, session)
        if not task:
            raise HTTPException(status_code=404, detail="Task not found")

        return {
            "success": True,
            "data": {
                "task": task
            }
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving task: {str(e)}")


@router.put("/{task_id}", response_model=dict)
async def update_task(
    task_id: str,
    task_update: TaskUpdate,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Update a specific task by ID (public access model).
    """
    try:
        # For public access model, use a default user ID
        task = await TaskService.update_task_public(task_id, task_update, session)
        if not task:
            raise HTTPException(status_code=404, detail="Task not found")

        return {
            "success": True,
            "data": {
                "task": task
            }
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating task: {str(e)}")


@router.patch("/{task_id}/complete", response_model=dict)
async def toggle_task_completion(
    task_id: str,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Toggle the completion status of a task (public access model).
    """
    try:
        task = await TaskService.toggle_task_completion_public(task_id, session)
        if not task:
            raise HTTPException(status_code=404, detail="Task not found")

        return {
            "success": True,
            "data": {
                "task": task
            }
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error toggling task completion: {str(e)}")


@router.get("/search", response_model=dict)
async def search_tasks(
    q: str,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Search tasks by title or description (public access model).
    """
    try:
        # Search tasks using the service
        tasks = await TaskService.search_tasks(q, session)

        return {
            "success": True,
            "data": {
                "tasks": tasks
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error searching tasks: {str(e)}")


@router.delete("/{task_id}")
async def delete_task(
    task_id: str,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Delete a specific task by ID (public access model).
    """
    try:
        success = await TaskService.delete_task_public(task_id, session)
        if not success:
            raise HTTPException(status_code=404, detail="Task not found")

        return {
            "success": True,
            "message": "Task deleted successfully"
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting task: {str(e)}")