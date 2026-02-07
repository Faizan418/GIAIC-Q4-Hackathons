from fastapi import APIRouter, Depends, HTTPException, status, Query
from typing import List, Optional
from sqlmodel import Session, select, and_
from datetime import datetime
from ..db import get_session
from ..models import Task, User, TaskCreate, TaskUpdate
from ..schemas import TaskRead, TaskListResponse, TaskUpdateStatus
from ..auth import get_current_user_id, verify_user_owns_resource
from enum import Enum


router = APIRouter(prefix="/api/v1", tags=["tasks"])


@router.get("/{user_id}/tasks", response_model=TaskListResponse)
def get_tasks(
    user_id: str,
    current_user_id: str = Depends(get_current_user_id),
    completed: Optional[bool] = Query(None, description="Filter by completion status"),
    priority: Optional[str] = Query(None, description="Filter by priority level"),
    due_date: Optional[str] = Query(None, description="Filter by due date"),
    search: Optional[str] = Query(None, description="Search tasks by title or description"),
    sort: Optional[str] = Query(None, description="Sort field"),
    order: Optional[str] = Query("asc", description="Sort order"),
    limit: int = Query(50, ge=1, le=100, description="Number of tasks to return"),
    offset: int = Query(0, ge=0, description="Number of tasks to skip"),
    session: Session = Depends(get_session)
):
    # Verify user has permission to access these tasks
    verify_user_owns_resource(current_user_id, user_id)

    # Build query with filters
    query = select(Task).where(Task.user_id == user_id)

    if completed is not None:
        query = query.where(Task.completed == completed)

    if priority is not None:
        query = query.where(Task.priority == priority)

    if due_date is not None:
        # Assuming due_date is in YYYY-MM-DD format
        from datetime import datetime
        try:
            parsed_date = datetime.strptime(due_date, "%Y-%m-%d").date()
            query = query.where(Task.due_date.like(f"{parsed_date}%"))
        except ValueError:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid date format. Use YYYY-MM-DD."
            )

    if search is not None:
        query = query.where(
            Task.title.contains(search) | Task.description.contains(search)
        )

    # Apply sorting
    if sort == "due_date":
        if order == "desc":
            query = query.order_by(Task.due_date.desc())
        else:
            query = query.order_by(Task.due_date.asc())
    elif sort == "priority":
        if order == "desc":
            query = query.order_by(Task.priority.desc())
        else:
            query = query.order_by(Task.priority.asc())
    elif sort == "created_at":
        if order == "desc":
            query = query.order_by(Task.created_at.desc())
        else:
            query = query.order_by(Task.created_at.asc())
    elif sort == "title":
        if order == "desc":
            query = query.order_by(Task.title.desc())
        else:
            query = query.order_by(Task.title.asc())
    else:
        # Default sorting by creation date descending
        query = query.order_by(Task.created_at.desc())

    # Get total count
    total_query = select(Task).where(Task.user_id == user_id)
    if completed is not None:
        total_query = total_query.where(Task.completed == completed)
    if priority is not None:
        total_query = total_query.where(Task.priority == priority)
    if due_date is not None:
        parsed_date = datetime.strptime(due_date, "%Y-%m-%d").date()
        total_query = total_query.where(Task.due_date.like(f"{parsed_date}%"))
    if search is not None:
        total_query = total_query.where(
            Task.title.contains(search) | Task.description.contains(search)
        )

    total_count = session.exec(total_query).count()

    # Apply pagination
    query = query.offset(offset).limit(limit)
    tasks = session.exec(query).all()

    return TaskListResponse(
        tasks=[TaskRead.from_orm(task) for task in tasks],
        total=total_count,
        offset=offset,
        limit=limit
    )


@router.post("/{user_id}/tasks", response_model=TaskRead, status_code=status.HTTP_201_CREATED)
def create_task(
    user_id: str,
    task_create: TaskCreate,
    current_user_id: str = Depends(get_current_user_id),
    session: Session = Depends(get_session)
):
    # Verify user has permission to create tasks for this user_id
    verify_user_owns_resource(current_user_id, user_id)

    # Create task
    task_data = task_create.dict()
    task = Task(
        **task_data,
        user_id=user_id,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )

    session.add(task)
    session.commit()
    session.refresh(task)

    return TaskRead.from_orm(task)


@router.get("/{user_id}/tasks/{task_id}", response_model=TaskRead)
def get_task(
    user_id: str,
    task_id: int,
    current_user_id: str = Depends(get_current_user_id),
    session: Session = Depends(get_session)
):
    # Verify user has permission to access this task
    verify_user_owns_resource(current_user_id, user_id)

    # Get task
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Double-check that the task belongs to the user
    verify_user_owns_resource(task.user_id, user_id)

    return TaskRead.from_orm(task)


@router.put("/{user_id}/tasks/{task_id}", response_model=TaskRead)
def update_task(
    user_id: str,
    task_id: int,
    task_update: TaskUpdate,
    current_user_id: str = Depends(get_current_user_id),
    session: Session = Depends(get_session)
):
    # Verify user has permission to update this task
    verify_user_owns_resource(current_user_id, user_id)

    # Get task
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Double-check that the task belongs to the user
    verify_user_owns_resource(task.user_id, user_id)

    # Update task fields
    update_data = task_update.dict(exclude_unset=True)
    for field, value in update_data.items():
        setattr(task, field, value)

    task.updated_at = datetime.utcnow()

    session.add(task)
    session.commit()
    session.refresh(task)

    return TaskRead.from_orm(task)


@router.delete("/{user_id}/tasks/{task_id}", status_code=status.HTTP_204_NO_CONTENT)
def delete_task(
    user_id: str,
    task_id: int,
    current_user_id: str = Depends(get_current_user_id),
    session: Session = Depends(get_session)
):
    # Verify user has permission to delete this task
    verify_user_owns_resource(current_user_id, user_id)

    # Get task
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Double-check that the task belongs to the user
    verify_user_owns_resource(task.user_id, user_id)

    session.delete(task)
    session.commit()


@router.patch("/{user_id}/tasks/{task_id}/complete", response_model=TaskRead)
def update_task_completion(
    user_id: str,
    task_id: int,
    task_status: TaskUpdateStatus,
    current_user_id: str = Depends(get_current_user_id),
    session: Session = Depends(get_session)
):
    # Verify user has permission to update this task
    verify_user_owns_resource(current_user_id, user_id)

    # Get task
    task = session.get(Task, task_id)
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    # Double-check that the task belongs to the user
    verify_user_owns_resource(task.user_id, user_id)

    # Update completion status
    task.completed = task_status.completed
    task.updated_at = datetime.utcnow()

    # Handle recurring tasks
    if task_status.completed and task.recurrence != RecurrenceEnum.none:
        # Check if we should create a new recurring instance
        from ..utils import create_recurring_task, limit_recurrence_generations

        if limit_recurrence_generations(task.__dict__):
            # Create a new task based on the original recurring task
            new_task_data = create_recurring_task({
                'title': task.title,
                'description': task.description,
                'completed': False,  # New instance starts as incomplete
                'priority': task.priority,
                'tags': task.tags,
                'due_date': task.due_date,
                'recurrence': task.recurrence,
                'user_id': task.user_id
            })

            # Create new task instance
            new_task = Task(
                title=new_task_data['title'],
                description=new_task_data['description'],
                completed=new_task_data['completed'],
                priority=new_task_data['priority'],
                tags=new_task_data['tags'],
                due_date=new_task_data['due_date'],
                recurrence=new_task_data['recurrence'],
                user_id=new_task_data['user_id'],
                created_at=new_task_data['created_at'],
                updated_at=new_task_data['updated_at']
            )
            session.add(new_task)

    session.add(task)
    session.commit()
    session.refresh(task)

    return TaskRead.from_orm(task)