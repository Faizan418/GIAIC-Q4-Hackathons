# Technical Research: Todo In-Memory Python Console App

**Feature**: Todo In-Memory Python Console App
**Branch**: `001-todo-app`
**Date**: 2025-12-29
**Purpose**: Research technical approaches for key implementation decisions

---

## 1. UV Package Manager Integration

### Decision
Use UV for all Python dependency management instead of pip or Poetry.

### Research Findings

**UV Overview**:
- Modern Python package installer and resolver (created by Astral)
- Rust-based for performance (10-100x faster than pip)
- Compatible with pip, requirements.txt, and pyproject.toml
- Provides lock files for reproducible builds

**Installation**:
```bash
# Windows (PowerShell)
irm https://astral.sh/uv/install.ps1 | iex

# macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**Project Initialization**:
```bash
uv init todo-app
cd todo-app
uv add colorama pytest
```

**Configuration** (pyproject.toml):
```toml
[project]
name = "todo-app"
version = "1.0.0"
requires-python = ">=3.13"
dependencies = []

[project.optional-dependencies]
dev = ["pytest>=8.0.0", "colorama>=0.4.6"]
```

**Running the App**:
```bash
uv run python src/main.py
```

**Running Tests**:
```bash
uv run pytest tests/
```

### Rationale
- **Performance**: 10-100x faster than pip
- **Modern**: Better dependency resolution
- **Explicit Requirement**: User specified UV usage
- **Lock Files**: Ensures reproducible builds

### Alternatives Considered
1. **pip + requirements.txt**: Traditional but slower, less reliable resolution
2. **Poetry**: Good alternative but heavier, slower than UV
3. **UV**: Selected for speed and modern features

---

## 2. In-Memory Storage Strategy

### Decision
Use Python list as primary storage + dict for ID indexing (hybrid approach).

### Research Findings

**Approach 1: List Only**
```python
tasks: list[Task] = []

def get_task_by_id(task_id: int) -> Optional[Task]:
    for task in tasks:
        if task.id == task_id:
            return task
    return None  # O(n) lookup
```
- **Pros**: Simple, maintains insertion order
- **Cons**: O(n) lookups by ID

**Approach 2: Dict Only**
```python
tasks: dict[int, Task] = {}

def get_all_tasks() -> list[Task]:
    return list(tasks.values())  # Maintains order in Python 3.7+
```
- **Pros**: O(1) lookups by ID
- **Cons**: Before Python 3.7, dict didn't guarantee order

**Approach 3: List + Dict Hybrid (SELECTED)**
```python
tasks_list: list[Task] = []
tasks_index: dict[int, Task] = {}

def add_task(task: Task):
    tasks_list.append(task)
    tasks_index[task.id] = task  # O(1) insert

def get_task_by_id(task_id: int) -> Optional[Task]:
    return tasks_index.get(task_id)  # O(1) lookup

def get_all_tasks() -> list[Task]:
    return tasks_list.copy()  # O(n) but maintains order
```

**Performance Analysis**:
| Operation | List Only | Dict Only | Hybrid |
|-----------|-----------|-----------|--------|
| Add | O(1) | O(1) | O(1) |
| Get by ID | O(n) | O(1) | O(1) |
| Get all | O(1) | O(n) | O(1) |
| Delete | O(n) | O(1) | O(n) + O(1) |
| Memory | 1x | 1x | ~2x refs |

### Rationale
- **Fast Lookups**: O(1) ID lookups critical for update/delete operations
- **Order Preservation**: List maintains creation order for display
- **Acceptable Trade-off**: Slight memory overhead for better performance
- **Scalability**: Handles 1000+ tasks efficiently

### Implementation
```python
# storage.py
tasks_list: list[Task] = []
tasks_index: dict[int, Task] = {}
next_id: int = 1

def get_next_id() -> int:
    global next_id
    current = next_id
    next_id += 1
    return current

def add_task_to_storage(task: Task) -> None:
    tasks_list.append(task)
    tasks_index[task.id] = task

def delete_task_from_storage(task_id: int) -> bool:
    if task_id not in tasks_index:
        return False

    task = tasks_index[task_id]
    tasks_list.remove(task)  # O(n) but rare operation
    del tasks_index[task_id]
    return True
```

---

## 3. Recurring Task Implementation

### Decision
Calculate next occurrence when task marked complete, store pattern with task.

### Research Findings

**datetime Arithmetic for Recurrence Patterns**:

```python
from datetime import datetime, timedelta
from dateutil.relativedelta import relativedelta  # For month arithmetic

def calculate_next_occurrence(due_date: datetime, frequency: str) -> datetime:
    """Calculate next occurrence based on frequency."""
    if frequency == "Daily":
        return due_date + timedelta(days=1)
    elif frequency == "Weekly":
        return due_date + timedelta(weeks=1)
    elif frequency == "Monthly":
        # Use relativedelta for proper month handling
        return due_date + relativedelta(months=1)
    else:
        return due_date

# Example usage
original = datetime(2025, 12, 29, 14, 0)
next_daily = calculate_next_occurrence(original, "Daily")
# Result: 2025-12-30 14:00

next_monthly = calculate_next_occurrence(original, "Monthly")
# Result: 2026-01-29 14:00
```

**Recurrence Pattern Storage**:
```python
@dataclass
class RecurrencePattern:
    frequency: str  # "Daily", "Weekly", "Monthly"
    original_due_date: datetime

@dataclass
class Task:
    # ... other fields
    due_date: Optional[datetime] = None
    recurrence: Optional[RecurrencePattern] = None
```

**Regeneration Logic**:
```python
def regenerate_recurring_task(task: Task) -> Optional[Task]:
    """Create next occurrence of recurring task."""
    if not task.recurrence:
        return None

    # Calculate next due date
    next_due = calculate_next_occurrence(
        task.due_date,
        task.recurrence.frequency
    )

    # Create new task instance
    new_task = Task(
        id=get_next_id(),
        title=task.title,
        completed=False,  # Reset completion
        priority=task.priority,
        tags=task.tags.copy(),
        due_date=next_due,
        recurrence=task.recurrence
    )

    add_task_to_storage(new_task)
    return new_task
```

### Rationale
- **Simplicity**: No background scheduler needed
- **User Control**: Regeneration happens when user marks complete
- **Immediate Feedback**: User sees next occurrence immediately
- **Testability**: Easier to test than background processes
- **Alignment**: Matches spec assumption #2

### Edge Cases Handled
1. **Past Due Dates**: Accept but mark as overdue
2. **Month Boundaries**: Use relativedelta for proper month arithmetic
3. **Leap Years**: relativedelta handles automatically
4. **DST Changes**: Preserved in datetime objects

---

## 4. Console Reminder System

### Decision
Check due dates on each menu display, show inline notifications.

### Research Findings

**datetime Comparison**:
```python
from datetime import datetime, timedelta

def is_due_now(due_date: datetime, tolerance_minutes: int = 5) -> bool:
    """Check if task is due within tolerance window."""
    now = datetime.now()
    diff = abs((due_date - now).total_seconds() / 60)
    return diff <= tolerance_minutes

def is_overdue(due_date: datetime) -> bool:
    """Check if task is overdue."""
    return datetime.now() > due_date
```

**Reminder Display**:
```python
def check_reminders() -> list[Task]:
    """Get tasks due now for reminder display."""
    due_tasks = []
    for task in get_all_tasks():
        if task.due_date and not task.completed:
            if is_due_now(task.due_date):
                due_tasks.append(task)
    return due_tasks

def display_reminders():
    """Show reminder notifications in console."""
    reminders = check_reminders()
    if reminders:
        print("\n" + "="*50)
        print("⏰ REMINDERS")
        print("="*50)
        for task in reminders:
            print(f"[{task.id}] {task.title} - Due: {task.due_date.strftime('%Y-%m-%d %H:%M')}")
        print("="*50 + "\n")
```

**Integration with Menu Display**:
```python
def display_main_menu():
    """Display menu with inline reminders."""
    # Check and display reminders first
    display_reminders()

    # Then show main menu
    print("\n[1] Add Task")
    print("[2] View Tasks")
    # ... rest of menu
```

### Rationale
- **No Background Threads**: Simpler architecture
- **Console-Appropriate**: Fits CLI interface paradigm
- **User Awareness**: Reminders appear when user is actively using app
- **Alignment**: Matches spec assumption #1

### Console Formatting Best Practices
```python
# Use colorama for cross-platform color support (optional)
from colorama import Fore, Style, init
init()  # Initialize colorama

def display_overdue_task(task: Task):
    """Display overdue task with red formatting."""
    print(f"{Fore.RED}[OVERDUE] {task.title}{Style.RESET_ALL}")

def display_task_with_priority(task: Task):
    """Display task with priority indicator."""
    priority_icon = {
        "High": f"{Fore.RED}🔴",
        "Medium": f"{Fore.YELLOW}🟡",
        "Low": f"{Fore.GREEN}🟢"
    }
    icon = priority_icon.get(task.priority, "")
    print(f"{icon} [{task.id}] {task.title}{Style.RESET_ALL}")
```

---

## 5. ID Generation Strategy

### Decision
Global counter starting at 1, never reused.

### Research Findings

**Simple Counter Approach (SELECTED)**:
```python
# storage.py
next_id: int = 1

def get_next_id() -> int:
    """Generate next unique task ID."""
    global next_id
    current_id = next_id
    next_id += 1
    return current_id
```

**Pros**:
- **Simple**: Easy to understand and implement
- **Predictable**: IDs are sequential (1, 2, 3, ...)
- **Unique**: No collisions
- **Matches Spec**: Aligns with assumption #4

**Thread-Safe Alternative (Future-Proofing)**:
```python
import threading

class IDGenerator:
    def __init__(self):
        self._next_id = 1
        self._lock = threading.Lock()

    def get_next_id(self) -> int:
        with self._lock:
            current_id = self._next_id
            self._next_id += 1
            return current_id

id_generator = IDGenerator()
```

**Note**: Thread-safety not required for current single-user design, but structure allows easy future enhancement.

### ID Reuse Policy
IDs are NEVER reused after deletion:
- Task ID 5 deleted → ID 5 never used again
- Next task gets next available ID from counter
- Matches spec assumption #4

---

## 6. Date/Time Input Parsing

### Decision
Support ISO 8601 (YYYY-MM-DD HH:MM) as primary format with common variants.

### Research Findings

**Parsing Strategy**:
```python
from datetime import datetime
from typing import Optional

def parse_datetime(date_str: str) -> Optional[datetime]:
    """Parse datetime string with multiple format support."""
    formats = [
        "%Y-%m-%d %H:%M",      # ISO 8601: 2025-12-30 14:00
        "%Y-%m-%d %H:%M:%S",   # With seconds: 2025-12-30 14:00:00
        "%Y-%m-%d",            # Date only: 2025-12-30 (assumes 00:00)
        "%m/%d/%Y %H:%M",      # US format: 12/30/2025 14:00
        "%d-%m-%Y %H:%M",      # EU format: 30-12-2025 14:00
    ]

    for fmt in formats:
        try:
            return datetime.strptime(date_str.strip(), fmt)
        except ValueError:
            continue

    return None  # None if no format matches

# Validation wrapper
def validate_and_parse_date(date_str: str) -> tuple[bool, Optional[datetime]]:
    """Validate and parse date string."""
    if not date_str or not date_str.strip():
        return False, None

    parsed = parse_datetime(date_str)
    if parsed is None:
        return False, None

    return True, parsed
```

**User Input Handling**:
```python
def get_due_date_from_user() -> Optional[datetime]:
    """Prompt user for due date with validation."""
    print("Enter due date (YYYY-MM-DD HH:MM):")
    date_str = input("> ").strip()

    if not date_str:
        return None  # Allow empty for no due date

    valid, parsed = validate_and_parse_date(date_str)
    if not valid:
        print("❌ Invalid date format. Use: YYYY-MM-DD HH:MM")
        return None

    return parsed
```

### Rationale
- **Standard Format**: ISO 8601 is international standard
- **Flexibility**: Support common variations for usability
- **Clear Errors**: Explicit format in error messages
- **Alignment**: Matches spec assumption #9

### Edge Cases
1. **Date Only**: Assumes 00:00 if time not provided
2. **Invalid Dates**: Caught by strptime (e.g., Feb 30)
3. **Past Dates**: Accepted but flagged as overdue
4. **Future Dates**: Accepted normally

---

## Summary of Decisions

| Decision Area | Selected Approach | Key Benefit |
|---------------|------------------|-------------|
| Package Manager | UV | 10-100x faster than pip |
| Storage Structure | List + Dict hybrid | O(1) lookups, maintains order |
| Recurring Tasks | Regenerate on completion | Simple, no background threads |
| Reminders | Check on menu display | Console-appropriate |
| ID Generation | Global counter | Simple, predictable |
| Date Parsing | ISO 8601 + variants | Standard, flexible |

All decisions align with:
- ✅ Constitution principles (in-memory, modular, reproducible)
- ✅ Specification requirements (30 functional requirements)
- ✅ User assumptions (12 documented assumptions)
- ✅ Success criteria (10 measurable outcomes)

---

## Next Steps

1. ✅ Research complete - all technical decisions documented
2. ⏭️ Phase 1: Create design artifacts:
   - data-model.md
   - contracts/ (module APIs)
   - quickstart.md
3. ⏭️ Phase 2: Generate tasks.md (/sp.tasks command)
4. ⏭️ Implementation: Begin coding following task order
