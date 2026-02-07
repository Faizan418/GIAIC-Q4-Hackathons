---
name: todo-core-handler
description: Use this agent when the user wants to perform any CRUD operation on tasks in the Python Todo application. This includes adding a new task, updating an existing task, deleting a task, or toggling a task's completed status.
license: MIT
version: 1.0.0
---

# Todo Core Handler Skill

Handle all core in-memory logic operations for the Python Todo application including CRUD operations, task state management, and unique ID generation.

## Purpose

Implement the business logic layer of the Todo application that sits between the CLI interface and the storage layer. This skill focuses purely on task manipulation without UI concerns.

## When to Use

- Adding a new task: "Add a task to buy groceries"
- Updating an existing task: "Update task #2 priority to high"
- Deleting a task: "Delete task #5"
- Toggling completion: "Mark task #3 as complete"
- Listing tasks: "Show me all current tasks"

## Subagent Coordination

| Subagent | Role |
|----------|------|
| `task-model-specialist` | Validate task schema and field values |
| `in-memory-storage-subagent` | Maintain task list and perform storage operations |

## Operation Handlers

### Create (Add Task)
```python
def add_task(title: str, priority: str = "Medium", tags: list = None) -> Task:
    # 1. Validate with task-model-specialist
    # 2. Generate unique ID
    # 3. Store via in-memory-storage-subagent
    # 4. Return created task
```

### Read (List Tasks)
```python
def list_tasks() -> list[Task]:
    # Delegate to in-memory-storage-subagent
    # Return all tasks in display format
```

### Update (Modify Task)
```python
def update_task(task_id: int, **kwargs) -> Task:
    # 1. Validate task_id exists
    # 2. Validate updated fields
    # 3. Apply update via storage
    # 4. Return modified task
```

### Delete (Remove Task)
```python
def delete_task(task_id: int) -> bool:
    # 1. Validate task_id exists
    # 2. Remove from storage
    # 3. Return confirmation
```

### Toggle Complete
```python
def toggle_complete(task_id: int) -> Task:
    # 1. Validate task_id exists
    # 2. Flip completed status
    # 3. Return updated task
```

## Constraints

- Do NOT handle CLI interactions directly
- Do NOT modify task metadata beyond allowed fields
- Do NOT persist data to disk
- Do NOT process non-CRUD commands
- IDs must be unique and never reused within a session

## Edge Cases

| Scenario | Handling |
|----------|----------|
| Empty task list | Return empty list gracefully |
| Invalid task ID | Return clear error |
| Invalid input | Delegate to task-model-specialist |
| Duplicate operations | Handle idempotently |

## Output Format

Return structured data containing:
- Operation result (success/failure)
- Affected task(s)
- Current task list
- Error messages with clear context
