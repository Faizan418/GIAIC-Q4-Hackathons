---
name: spec-driven-todo-design
description: Guide for designing Todo applications using spec-driven development principles. This skill should be used when defining structure and rules before coding.
license: MIT
---

# Spec-Driven Todo Design Skill

Design and architect Todo applications using specification-first development methodology.

## Purpose
Create clear, comprehensive specifications and architectural plans before implementation to ensure alignment, reduce rework, and maintain quality standards.

## When to Use
- Starting a new Todo application project
- Planning major features or enhancements
- Defining project structure and architecture
- Establishing coding standards and conventions
- Creating implementation roadmaps

## Core Responsibilities
1. **Requirements Definition** - Capture what needs to be built
2. **Architecture Design** - Define system structure
3. **Interface Design** - Specify APIs and contracts
4. **Task Breakdown** - Create actionable implementation tasks
5. **Quality Standards** - Define acceptance criteria

## Spec-Driven Development Workflow

### Phase 1: Specification (spec.md)
Define WHAT needs to be built.

```markdown
# Todo Application Specification

## Overview
A command-line Todo application for managing tasks in memory.

## Goals
- Simple task management (add, view, update, delete)
- Task organization (priority, tags, search, filter)
- User-friendly CLI interface
- In-memory storage (no persistence)

## Non-Goals
- Database persistence
- Multi-user support
- Web interface
- Cloud synchronization

## Features

### F1: Task Management
**Description:** Core CRUD operations for tasks

**Requirements:**
- Add task with title (required), priority (optional), tags (optional)
- View all tasks
- Update task title, priority, or tags
- Delete task by ID
- Toggle task completion status

**Acceptance Criteria:**
- [ ] User can add task with valid title
- [ ] Empty title is rejected with error
- [ ] User can view list of all tasks
- [ ] User can update any task field
- [ ] User can delete task by ID
- [ ] Deleted tasks are removed from list
- [ ] User can mark tasks as complete/incomplete

### F2: Task Organization
**Description:** Search, filter, and sort capabilities

**Requirements:**
- Search tasks by keyword in title
- Filter by completion status (completed/pending)
- Filter by priority level (High/Medium/Low)
- Filter by tags
- Sort by priority, date, title, or status

**Acceptance Criteria:**
- [ ] Search finds tasks containing keyword (case-insensitive)
- [ ] Filters return correct subset of tasks
- [ ] Sort orders tasks correctly
- [ ] Multiple filters can be combined

### F3: CLI Interface
**Description:** Interactive menu-based interface

**Requirements:**
- Main menu with numbered options
- Input validation and error messages
- Clear task display formatting
- Confirmation for destructive actions
- Exit option

**Acceptance Criteria:**
- [ ] Menu displays all available options
- [ ] Invalid input shows helpful error
- [ ] Tasks display with readable formatting
- [ ] Delete requires confirmation
- [ ] App exits cleanly on user request

## Data Model

### Task Entity
```python
Task:
  - id: int (auto-generated, unique)
  - title: str (required, non-empty)
  - completed: bool (default: False)
  - priority: str (High/Medium/Low, default: Medium)
  - tags: list[str] (default: empty list)
  - created_at: datetime (auto-generated)
```

## User Flows

### Flow 1: Add Task
1. User selects "Add Task" from menu
2. System prompts for task title
3. User enters title
4. System prompts for priority (optional)
5. User selects priority or skips
6. System prompts for tags (optional)
7. User enters tags or skips
8. System creates task and shows confirmation
9. System returns to main menu

### Flow 2: Delete Task
1. User selects "Delete Task" from menu
2. System prompts for task ID
3. User enters ID
4. System displays task details
5. System asks for confirmation
6. User confirms (yes/no)
7. System deletes task (if confirmed)
8. System shows result message
9. System returns to main menu

## Error Handling
- Empty task title → "Task title cannot be empty"
- Invalid priority → "Priority must be High, Medium, or Low"
- Task not found → "Task #X not found"
- Invalid menu choice → "Invalid choice. Please select 0-8"

## Constraints
- All data stored in memory (lost on exit)
- Single-user application
- Console-only interface
- Python 3.8+ required
```

### Phase 2: Architecture Plan (plan.md)
Define HOW it will be built.

```markdown
# Todo Application Architecture Plan

## System Architecture

### Module Structure
```
todo-app/
├── core.py              # Core business logic
├── cli.py               # CLI interface
├── organization.py      # Search/filter/sort
├── main.py             # Application entry point
├── tests/
│   ├── test_core.py
│   ├── test_organization.py
│   └── test_edge_cases.py
└── README.md
```

### Module Responsibilities

#### core.py - Core Logic Layer
**Purpose:** Task data model and CRUD operations

**Public API:**
- `add_task(title, priority, tags) -> Task`
- `get_all_tasks() -> list[Task]`
- `get_task_by_id(id) -> Task | None`
- `update_task(id, title, priority, tags) -> bool`
- `delete_task(id) -> bool`
- `toggle_task_completion(id) -> bool`
- `get_task_count() -> dict`

**Data Structures:**
- `Task` dataclass with attributes
- `tasks: list[Task]` global storage
- `next_id: int` ID generator

**Principles:**
- Single responsibility per function
- Validate inputs before mutation
- Return copies to prevent external modification
- Raise ValueError for invalid inputs

#### cli.py - User Interface Layer
**Purpose:** Menu display and user interaction

**Public API:**
- `display_main_menu() -> None`
- `get_menu_choice() -> str`
- `handle_add_task() -> None`
- `handle_view_all_tasks() -> None`
- `handle_update_task() -> None`
- `handle_delete_task() -> None`
- `display_task(task) -> None`
- `display_task_list(tasks) -> None`
- `run_app() -> None`

**Responsibilities:**
- Display menus and prompts
- Validate user input
- Call core functions
- Format output
- Handle errors gracefully

#### organization.py - Organization Layer
**Purpose:** Advanced task operations

**Public API:**
- `search_tasks_by_keyword(keyword) -> list[Task]`
- `filter_by_status(completed) -> list[Task]`
- `filter_by_priority(priority) -> list[Task]`
- `sort_tasks_by_priority(tasks) -> list[Task]`
- `sort_tasks_by_date(tasks) -> list[Task]`
- `get_all_tags() -> list[str]`
- `add_tag_to_task(id, tag) -> bool`

**Responsibilities:**
- Search and filter logic
- Sorting algorithms
- Tag management
- Non-destructive operations

## Data Flow

### Add Task Flow
```
User Input (cli.py)
  ↓
Validation (cli.py)
  ↓
add_task() (core.py)
  ↓
Task Creation & Storage (core.py)
  ↓
Return Task Object
  ↓
Display Confirmation (cli.py)
```

### Search Flow
```
User Input (cli.py)
  ↓
search_tasks_by_keyword() (organization.py)
  ↓
get_all_tasks() (core.py)
  ↓
Filter Results (organization.py)
  ↓
Return Filtered List
  ↓
display_task_list() (cli.py)
```

## Key Design Decisions

### Decision 1: In-Memory Storage
**Options Considered:**
- In-memory list
- SQLite database
- JSON file persistence

**Choice:** In-memory list

**Rationale:**
- Simplest implementation
- No external dependencies
- Fast operations
- Suitable for learning/prototyping
- Aligns with project constraints

**Trade-offs:**
- Data lost on exit (acceptable per requirements)
- No multi-process support (not needed)
- Limited scalability (acceptable for single user)

### Decision 2: Module Separation
**Options Considered:**
- Single file application
- Two-tier (UI + logic)
- Three-tier (UI + organization + core)

**Choice:** Three-tier separation

**Rationale:**
- Clear separation of concerns
- Easier to test each layer
- Supports future enhancements
- Better code organization
- Follows SOLID principles

### Decision 3: Task Model
**Options Considered:**
- Dictionary
- Named tuple
- Dataclass
- Custom class

**Choice:** Dataclass

**Rationale:**
- Built-in Python feature (3.7+)
- Automatic __init__ and __repr__
- Type hints support
- Immutability options
- Clean syntax

## Error Handling Strategy

### Input Validation
- Validate at CLI layer before calling core
- Core functions validate business rules
- Raise ValueError with descriptive messages
- Never fail silently

### Error Display
- User-friendly error messages
- Show what went wrong
- Suggest correction
- Don't expose internal details

## Testing Strategy

### Unit Tests
- Test each function independently
- Mock dependencies
- Cover happy path and edge cases
- Test error conditions

### Integration Tests
- Test module interactions
- Test complete user flows
- Verify data consistency

### Test Coverage Goals
- Core logic: 100%
- Organization functions: 100%
- CLI handlers: 80% (manual testing supplement)

## Performance Considerations
- Linear search acceptable for small datasets (<1000 tasks)
- No optimization needed initially
- Consider indexing if scaling beyond 1000 tasks

## Security Considerations
- Input sanitization (trim whitespace)
- No SQL injection risk (no database)
- No XSS risk (console only)
- No authentication needed (single user)

## Future Extensibility
**Designed for easy addition of:**
- File persistence (add persistence.py)
- Database backend (replace storage mechanism)
- Due dates (extend Task model)
- Recurring tasks (add recurrence logic)
- Task notes (extend Task model)
```

### Phase 3: Task Breakdown (tasks.md)
Define detailed implementation steps.

```markdown
# Todo Application Implementation Tasks

## Phase 1: Core Foundation

### Task 1.1: Set up project structure
**Description:** Create directory structure and files

**Steps:**
1. Create project directory
2. Create core.py, cli.py, organization.py, main.py
3. Create tests/ directory
4. Create README.md

**Acceptance:**
- [ ] All files created
- [ ] Directory structure matches plan

### Task 1.2: Implement Task model
**File:** core.py

**Implementation:**
```python
from dataclasses import dataclass, field
from datetime import datetime
from typing import List

@dataclass
class Task:
    id: int
    title: str
    completed: bool = False
    priority: str = "Medium"
    tags: List[str] = field(default_factory=list)
    created_at: datetime = field(default_factory=datetime.now)
```

**Acceptance:**
- [ ] Task dataclass defined with all fields
- [ ] Default values set correctly
- [ ] Type hints included

### Task 1.3: Implement add_task function
**File:** core.py

**Implementation:**
- Initialize global storage variables
- Implement get_next_id()
- Implement add_task() with validation

**Test Cases:**
- Add task with title only
- Add task with all fields
- Empty title raises ValueError
- Invalid priority raises ValueError

**Acceptance:**
- [ ] Function creates and returns task
- [ ] Task added to global list
- [ ] ID auto-generated and unique
- [ ] Validation works correctly

[Continue with all tasks...]

## Phase 2: CLI Interface
[Tasks for CLI implementation...]

## Phase 3: Organization Features
[Tasks for search/filter/sort...]

## Phase 4: Testing
[Tasks for test implementation...]
```

## Design Principles for Todo Apps

### 1. Separation of Concerns
- **Core Logic**: Pure business logic, no UI
- **CLI Layer**: User interaction, no business logic
- **Organization**: Advanced features, uses core

### 2. Single Responsibility
Each function does ONE thing well:
- `add_task()` - creates task
- `display_task()` - shows task
- `search_tasks()` - finds tasks

### 3. Data Encapsulation
- Don't expose internal storage directly
- Use getter functions
- Return copies, not references
- Validate at boundaries

### 4. Error Handling
- Validate early (fail fast)
- Descriptive error messages
- Handle errors at appropriate level
- Don't silently fail

### 5. Testability
- Small, focused functions
- No hidden dependencies
- Predictable behavior
- Setup/teardown support

## Common Patterns

### Pattern 1: CRUD Operations
```python
# Create
def add_entity(...) -> Entity

# Read
def get_all_entities() -> list[Entity]
def get_entity_by_id(id) -> Entity | None

# Update
def update_entity(id, ...) -> bool

# Delete
def delete_entity(id) -> bool
```

### Pattern 2: CLI Handler
```python
def handle_action():
    """Handle user action"""
    print("--- Action Title ---")

    try:
        # Get input
        data = get_user_input()

        # Call core function
        result = core_function(data)

        # Show result
        display_result(result)

    except ValueError as e:
        print(f"Error: {e}")
```

### Pattern 3: Filter Pipeline
```python
# Get all items
items = get_all_items()

# Apply filters
if keyword:
    items = filter_by_keyword(items, keyword)

if status:
    items = filter_by_status(items, status)

# Sort
items = sort_items(items, sort_by)

# Display
display_items(items)
```

## Pre-Implementation Checklist

Before writing code:
- [ ] Requirements clearly defined
- [ ] Data model documented
- [ ] Module responsibilities defined
- [ ] Public APIs specified
- [ ] Error cases identified
- [ ] Test cases outlined
- [ ] User flows mapped
- [ ] Acceptance criteria written

## Benefits of Spec-Driven Approach

1. **Clarity** - Everyone knows what's being built
2. **Alignment** - Reduces misunderstandings
3. **Quality** - Catches issues before coding
4. **Testability** - Clear acceptance criteria
5. **Documentation** - Specs serve as docs
6. **Efficiency** - Less rework
7. **Maintainability** - Easier to modify

## Anti-Patterns to Avoid

- Starting to code without spec
- Mixing responsibilities between layers
- Over-engineering simple features
- Insufficient error handling
- No acceptance criteria
- Skipping test planning
- Vague requirements

## Integration with Development

### Workflow
1. Write spec.md (what to build)
2. Write plan.md (how to build)
3. Write tasks.md (step by step)
4. Implement tasks in order
5. Test against acceptance criteria
6. Update docs as needed

### When to Update Specs
- Requirements change
- New features added
- Architecture decisions made
- Bugs reveal design flaws
- Feedback requires changes

## Next Steps
1. Use this skill to create spec.md
2. Design architecture in plan.md
3. Break down tasks in tasks.md
4. Implement using other skills
5. Test against acceptance criteria
