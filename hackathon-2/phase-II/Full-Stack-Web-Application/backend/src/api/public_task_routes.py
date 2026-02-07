from fastapi import APIRouter, Depends, HTTPException
from sqlmodel.ext.asyncio.session import AsyncSession
from typing import List, Optional
from ..database.db import get_async_session
from ..services.task_service import TaskService
from ..services.task_service import TaskCreate, TaskUpdate, TaskResponse
from pydantic import BaseModel

router = APIRouter(tags=["public-tasks"])

@router.get("/", response_model=dict)
async def get_all_tasks(
    status: Optional[str] = None,
    priority: Optional[str] = None,
    search: Optional[str] = None,
    limit: int = 100,
    offset: int = 0,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Get all tasks with optional filtering (public access).
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

@router.post("/", response_model=TaskResponse)
async def create_task(
    task_data: TaskCreate,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Create a new task (public access model).
    """
    # For public access model, assign a default user ID
    default_user_id = "public-user"
    task = await TaskService.create_task(task_data, default_user_id, session)
    return task

@router.get("/{task_id}", response_model=TaskResponse)
async def get_task_by_id(
    task_id: str,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Get a specific task by ID (public access).
    """
    task = await TaskService.get_task_by_id_public(task_id, session)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    return task

@router.put("/{task_id}", response_model=TaskResponse)
async def update_task(
    task_id: str,
    task_update: TaskUpdate,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Update a specific task by ID (public access model).
    """
    task = await TaskService.update_task_public(task_id, task_update, session)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    return task

@router.patch("/{task_id}/complete", response_model=TaskResponse)
async def toggle_task_completion(
    task_id: str,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Toggle the completion status of a task (public access model).
    """
    task = await TaskService.toggle_task_completion_public(task_id, session)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    return task

@router.get("/search", response_model=List[TaskResponse])
async def search_tasks(
    q: str,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Search tasks by title or description (public access).
    """
    tasks = await TaskService.search_tasks(q, session)
    return tasks


@router.delete("/{task_id}")
async def delete_task(
    task_id: str,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Delete a specific task by ID (public access model).
    """
    success = await TaskService.delete_task_public(task_id, session)
    if not success:
        raise HTTPException(status_code=404, detail="Task not found")
    return {"message": "Task deleted successfully"}