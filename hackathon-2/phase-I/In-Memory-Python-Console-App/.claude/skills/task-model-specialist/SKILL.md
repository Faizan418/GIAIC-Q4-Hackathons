---
name: task-model-specialist
description: Use this agent when defining or modifying the Task data structure for a Todo application. Implement Task class with proper attributes, validation, and serialization.
license: MIT
version: 1.0.0
---

# Task Model Specialist Skill

Data Model Specialist focused on defining the Task data structure for a Todo application with clean, validated, and serializable data models.

## Purpose

Create and maintain the core Task data model that represents individual todo items with all required attributes, proper typing, and validation.

## When to Use

- Implementing the core data layer for a Todo app
- Reviewing existing data models
- Adding new attributes to the Task model
- Modifying validation logic
- Implementing serialization/deserialization

## Task Attributes

| Attribute | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| id | str/int | Yes | auto-generated | Unique identifier |
| title | str | Yes | - | Task title (non-empty) |
| completed | bool | No | False | Completion status |
| priority | str | No | "Medium" | Priority level |
| tags | list[str] | No | [] | Category tags |
| created_at | datetime | Yes | auto-generated | Creation timestamp |

### Allowed Values
- **priority**: "High", "Medium", "Low" (case-insensitive)
- **title**: Non-empty string (max 200 chars recommended)
- **tags**: List of non-empty strings

## Implementation Requirements

### Python Class Structure
```python
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional
from enum import Enum

class Priority(Enum):
    HIGH = "High"
    MEDIUM = "Medium"
    LOW = "Low"

@dataclass
class Task:
    title: str
    id: Optional[str] = None
    completed: bool = False
    priority: Priority = Priority.MEDIUM
    tags: List[str] = None
    created_at: datetime = None
```

### Required Methods

| Method | Purpose |
|--------|---------|
| `serialize()` | Return JSON-compatible dictionary |
| `deserialize(data)` | Class method to reconstruct from dict |
| `mark_complete()` | Set completed=True |
| `mark_incomplete()` | Set completed=False |
| `add_tag(tag)` | Add tag if not present |
| `remove_tag(tag)` | Remove tag if present |
| `to_dict()` | Alias for serialize() |
| `__repr__()` | Debugging representation |

### Serialization Format
```python
{
    "id": "1",
    "title": "Buy groceries",
    "completed": False,
    "priority": "Medium",
    "tags": ["shopping", "urgent"],
    "created_at": "2025-01-15T10:30:00Z"
}
```

## Validation Rules

| Field | Validation |
|-------|------------|
| title | Non-empty string, max length check |
| priority | Must be in [High, Medium, Low] |
| tags | List of strings, no empty tags |
| completed | Boolean |
| id | Unique, not None for existing tasks |

## Behavioral Boundaries

**DO:**
- Focus solely on defining Task data structure
- Ensure all attributes are typed and validated
- Provide clean, readable Python code
- Include comprehensive docstrings
- Make model extensible for future attributes

**DO NOT:**
- Implement CLI interaction or UI code
- Implement task storage or persistence
- Create file I/O operations
- Implement filtering, sorting, or querying
- Add features beyond core data model

## Output Format

Provide:
1. Complete `Task` class code in fenced block
2. Example usage demonstrating creation and serialization
3. Brief explanation of key design decisions
4. Limitations or future extension considerations
