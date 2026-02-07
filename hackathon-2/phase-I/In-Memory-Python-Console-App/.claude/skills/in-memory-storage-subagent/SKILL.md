---
name: in-memory-storage-subagent
description: Use this agent when implementing or modifying in-memory task storage for the Todo application. Manage the task list and provide safe CRUD operations.
license: MIT
version: 1.0.0
---

# In-Memory Storage Subagent Skill

Storage Specialist for managing in-memory task storage and providing safe CRUD operations.

## Purpose

Maintain the authoritative in-memory list of tasks and expose safe operations for add, update, delete, toggle, and retrieval without any CLI or business logic concerns.

## When to Use

- Adding a new task to memory
- Updating an existing task's attributes
- Deleting a task from memory
- Toggling task completion status
- Retrieving all current tasks

## Core Operations

| Operation | Signature | Returns |
|-----------|-----------|---------|
| Add | `add_task(task: Task)` | Task object |
| Update | `update_task(task_id, **kwargs)` | Task or None |
| Delete | `delete_task(task_id)` | bool (deleted?) |
| Toggle | `toggle_complete(task_id)` | Task or None |
| List | `get_tasks()` | list[Task] |

## Implementation Pattern

```python
class InMemoryStorage:
    def __init__(self):
        self.tasks = []  # Authoritative task list

    def add_task(self, task):
        self.tasks.append(task)
        return task

    def update_task(self, task_id, **kwargs):
        for t in self.tasks:
            if t.id == task_id:
                for key, value in kwargs.items():
                    if hasattr(t, key):
                        setattr(t, key, value)
                return t
        return None  # ID not found

    def delete_task(self, task_id):
        original_count = len(self.tasks)
        self.tasks = [t for t in self.tasks if t.id != task_id]
        return len(self.tasks) < original_count

    def toggle_complete(self, task_id):
        for t in self.tasks:
            if t.id == task_id:
                t.completed = not t.completed
                return t
        return None

    def get_tasks(self):
        return self.tasks
```

## Operational Rules

### ID Validation
- Verify task_id exists before update/delete/toggle
- Return None or raise clear error if invalid
- Never proceed with operations on non-existent IDs

### Return Values
- Return updated task after add/update/toggle
- Return None when operations fail
- Return complete task list after every mutation

### State Management
- Maintain single authoritative Python list
- All operations modify this list directly
- No disk I/O or persistence layer
- No multiple storage backends

## Anti-Convergence Checklist

- [ ] Never define the Task model (import from task-model-specialist)
- [ ] Never implement file I/O, JSON, or database connections
- [ ] Never handle CLI argument parsing or user input
- [ ] Never implement validation logic (delegate to appropriate agent)
- [ ] Never maintain multiple storage backends

## Edge Cases

| Scenario | Handling |
|----------|----------|
| Empty storage | get_tasks() returns [] |
| Invalid ID | update/delete/toggle returns None |
| Missing attribute | setattr checks hasattr before setting |
| Duplicate add | Allows (appends to list) |

## Success Criteria

- All CRUD operations complete without errors on valid inputs
- Task IDs validated before updates, deletes, toggles
- In-memory list accurately reflects all operations
- Invalid operations fail gracefully with clear indicators
- No state lost between operations within session
