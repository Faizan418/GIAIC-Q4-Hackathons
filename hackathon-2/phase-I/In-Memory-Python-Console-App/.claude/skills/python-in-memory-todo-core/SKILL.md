---
name: python-in-memory-todo-core
description: Guide for building the core logic of an in-memory Python Todo console application. This skill should be used when implementing task CRUD and state management without persistence.
license: MIT
---

# Python In-Memory Todo Core Skill

Build the core business logic of a Todo application using pure Python with in-memory storage.

## Purpose
Implement task creation, reading, updating, deletion, and completion tracking using in-memory data structures (lists/dictionaries) without any database or file persistence.

## When to Use
- Building the foundational Todo logic layer
- Implementing CRUD operations for tasks
- Managing task state in memory
- Creating service/business logic functions
- Setting up the core data model

## Core Responsibilities
1. **Task Model Definition** - Define the Task data structure
2. **In-Memory Storage** - Manage tasks in lists or dictionaries
3. **CRUD Operations** - Add, retrieve, update, delete tasks
4. **State Management** - Toggle completion status, manage IDs
5. **Data Validation** - Ensure task integrity

## Implementation Guidelines

### 1. Task Model (using dataclasses)
```python
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Optional

@dataclass
class Task:
    id: int
    title: str
    completed: bool = False
    priority: str = "Medium"  # High, Medium, Low
    tags: List[str] = field(default_factory=list)
    created_at: datetime = field(default_factory=datetime.now)

    def to_dict(self):
        """Serialize task to dictionary"""
        return {
            'id': self.id,
            'title': self.title,
            'completed': self.completed,
            'priority': self.priority,
            'tags': self.tags,
            'created_at': self.created_at.isoformat()
        }
```

### 2. In-Memory Storage Structure
```python
# Global in-memory storage
tasks: List[Task] = []
next_id: int = 1

def get_next_id() -> int:
    """Generate unique task ID"""
    global next_id
    current_id = next_id
    next_id += 1
    return current_id
```

### 3. Core CRUD Functions

#### Create Task
```python
def add_task(title: str, priority: str = "Medium", tags: List[str] = None) -> Task:
    """
    Add a new task to memory

    Args:
        title: Task title (required, non-empty)
        priority: Priority level (High/Medium/Low)
        tags: Optional list of tags

    Returns:
        Created Task object

    Raises:
        ValueError: If title is empty or invalid priority
    """
    if not title or not title.strip():
        raise ValueError("Task title cannot be empty")

    valid_priorities = ["High", "Medium", "Low"]
    if priority not in valid_priorities:
        raise ValueError(f"Priority must be one of {valid_priorities}")

    task = Task(
        id=get_next_id(),
        title=title.strip(),
        priority=priority,
        tags=tags or []
    )
    tasks.append(task)
    return task
```

#### Read Tasks
```python
def get_all_tasks() -> List[Task]:
    """Retrieve all tasks"""
    return tasks.copy()

def get_task_by_id(task_id: int) -> Optional[Task]:
    """Find task by ID"""
    for task in tasks:
        if task.id == task_id:
            return task
    return None
```

#### Update Task
```python
def update_task(task_id: int, title: Optional[str] = None,
                priority: Optional[str] = None,
                tags: Optional[List[str]] = None) -> bool:
    """
    Update task attributes

    Returns:
        True if task found and updated, False otherwise
    """
    task = get_task_by_id(task_id)
    if not task:
        return False

    if title is not None:
        if not title.strip():
            raise ValueError("Task title cannot be empty")
        task.title = title.strip()

    if priority is not None:
        valid_priorities = ["High", "Medium", "Low"]
        if priority not in valid_priorities:
            raise ValueError(f"Priority must be one of {valid_priorities}")
        task.priority = priority

    if tags is not None:
        task.tags = tags

    return True
```

#### Delete Task
```python
def delete_task(task_id: int) -> bool:
    """
    Remove task from memory

    Returns:
        True if task was deleted, False if not found
    """
    global tasks
    initial_count = len(tasks)
    tasks = [t for t in tasks if t.id != task_id]
    return len(tasks) < initial_count
```

#### Toggle Completion
```python
def toggle_task_completion(task_id: int) -> bool:
    """
    Toggle task completed status

    Returns:
        True if toggled, False if task not found
    """
    task = get_task_by_id(task_id)
    if not task:
        return False

    task.completed = not task.completed
    return True
```

### 4. Utility Functions
```python
def clear_all_tasks() -> None:
    """Clear all tasks from memory"""
    global tasks, next_id
    tasks = []
    next_id = 1

def get_task_count() -> dict:
    """Get task statistics"""
    total = len(tasks)
    completed = sum(1 for t in tasks if t.completed)
    pending = total - completed

    return {
        'total': total,
        'completed': completed,
        'pending': pending
    }
```

## Testing the Core Logic
```python
# Example usage
if __name__ == "__main__":
    # Add tasks
    task1 = add_task("Buy groceries", priority="High", tags=["shopping"])
    task2 = add_task("Read book", priority="Low")

    # List tasks
    print(f"Total tasks: {len(get_all_tasks())}")

    # Update task
    update_task(task1.id, title="Buy groceries and milk")

    # Toggle completion
    toggle_task_completion(task1.id)

    # Get statistics
    stats = get_task_count()
    print(f"Stats: {stats}")

    # Delete task
    delete_task(task2.id)
```

## Key Principles
- **Single Responsibility** - Each function does one thing
- **Validation First** - Check inputs before modifying state
- **Immutable Returns** - Return copies to prevent external mutation
- **Clear Error Messages** - Raise descriptive exceptions
- **Type Hints** - Use type annotations for clarity
- **No Side Effects** - Functions should be predictable

## Integration Points
- **CLI Layer** - Calls these functions based on user input
- **Organization Layer** - Filters/searches the task list
- **Testing Layer** - Validates core logic behavior

## Common Patterns

### Error Handling
```python
try:
    task = add_task("")
except ValueError as e:
    print(f"Error: {e}")
```

### Safe Task Retrieval
```python
task = get_task_by_id(99)
if task:
    print(task.title)
else:
    print("Task not found")
```

## Best Practices
- Keep storage global but access through functions
- Validate all inputs at the boundary
- Use dataclasses for clean models
- Return success/failure booleans for operations
- Never expose internal list directly
- Document expected behavior clearly

## Anti-Patterns to Avoid
- Direct manipulation of global `tasks` list outside functions
- Returning None instead of empty lists
- Silently failing operations
- Mixing business logic with UI code
- Hardcoded IDs or magic numbers

## Next Steps
Once core logic is implemented:
1. Create CLI interface (python-todo-cli skill)
2. Add organization features (python-todo-organization skill)
3. Write comprehensive tests (python-todo-testing skill)
